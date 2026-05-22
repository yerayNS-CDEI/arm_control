#!/usr/bin/env python3

##################################################
#### ROS2 HYPERSPECTRAL SENSOR NODE
##################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from arm_control.srv import HyperspectralCommand, HyperspectralConfig
from sensors_lib.lenz_client import LenzClient
import threading
import time
import numpy as np
import csv
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class HyperspectralNode(Node):

    def __init__(self):
        super().__init__("hyperspectral_node")

        # Paràmetres — valors per defecte alineats amb les conclusions d'R+D
        self.declare_parameter("vis_ip",  "193.167.0.30")
        self.declare_parameter("nir_ip",  "193.167.0.31")
        self.declare_parameter("vis_mtr", 200000)  # MTR >= MTI (marge per Soft Ceiling VIS)
        self.declare_parameter("vis_mti", 140000)  # Zona lineal òptima per defecte
        self.declare_parameter("nir_mtr",  30000)  # MTR > sweet spot NIR
        self.declare_parameter("nir_mti",  23750)  # Sweet spot NIR: lineal i independent
        self.declare_parameter("auto_configure",     True)
        self.declare_parameter("enable_plotting",    False)
        self.declare_parameter("enable_csv_logging", False)

        try:
            pkg_dir = get_package_share_directory('arm_control')
            default_csv_dir = os.path.abspath(
                os.path.join(pkg_dir, '..', '..', '..', '..', 'src', 'arm_control', 'resource')
            )
        except Exception:
            default_csv_dir = os.path.expanduser("~")
        self.declare_parameter("csv_directory", default_csv_dir)

        vis_ip                = self.get_parameter("vis_ip").value
        nir_ip                = self.get_parameter("nir_ip").value
        self.vis_mtr          = self.get_parameter("vis_mtr").value
        self.vis_mti          = self.get_parameter("vis_mti").value
        self.nir_mtr          = self.get_parameter("nir_mtr").value
        self.nir_mti          = self.get_parameter("nir_mti").value
        self.auto_configure   = self.get_parameter("auto_configure").value
        self.enable_plotting  = self.get_parameter("enable_plotting").value
        self.enable_csv_logging = self.get_parameter("enable_csv_logging").value
        csv_directory         = self.get_parameter("csv_directory").value

        # Publishers
        self.vis_pub = self.create_publisher(Float32MultiArray, "/hyperspectral/vis_spectrum", 10)
        self.nir_pub = self.create_publisher(Float32MultiArray, "/hyperspectral/nir_spectrum", 10)

        # Longituds d'ona (256 punts per sensor)
        self.vis_wavelengths = np.linspace(325.3, 792.6, 256)
        self.nir_wavelengths = np.linspace(991.0, 1707.0, 256)

        # Calibracions en memòria (inicialitzades a None fins al GDS/GRF de l'inici)
        self.calibration_data = {
            "GDS": {"vis": None, "nir": None, "timestamp": None},
            "GRF": {"vis": None, "nir": None, "timestamp": None},
        }

        # CSV: comptador de files i ruta
        self._csv_row_counter = 0
        self.csv_file_path = None
        if self.enable_csv_logging:
            self._initialize_csv_file(csv_directory)

        # Plotting: importació i objectes matplotlib diferits fins que siguin necessaris
        self.fig = self.ax_vis = self.ax_nir = None
        if self.enable_plotting:
            self._setup_matplotlib_backend()
            if self.enable_plotting:
                import matplotlib.pyplot as plt
                self.fig, (self.ax_vis, self.ax_nir) = plt.subplots(2, 1, figsize=(10, 8))
                self.fig.suptitle('Hyperspectral Node — Raw Intensity', fontsize=14)
                for ax, title in [(self.ax_vis, 'VIS (325–793 nm)'), (self.ax_nir, 'NIR (991–1707 nm)')]:
                    ax.set_title(title)
                    ax.set_xlabel('Wavelength (nm)')
                    ax.set_ylabel('Intensity (raw counts)')
                    ax.grid(True, alpha=0.4)
                plt.ion()
                plt.show(block=False)
                self.get_logger().info("✓ Plotting actiu")

        # Serveis ROS 2 — MutuallyExclusiveCallbackGroup garanteix accés seqüencial al maquinari
        self.hardware_cb_group = MutuallyExclusiveCallbackGroup()
        self.configure_srv = self.create_service(
            HyperspectralConfig,
            "hyperspectral/configure",
            self.configure_callback,
            callback_group=self.hardware_cb_group,
        )
        self.measurement_srv = self.create_service(
            HyperspectralCommand,
            "hyperspectral/measurement",
            self.measurement_callback,
            callback_group=self.hardware_cb_group,
        )

        # Connexió als sensors
        self.get_logger().info("Connectant als sensors VIS i NIR...")
        self.vis = LenzClient(vis_ip, timeout=20.0)  # VIS: marge extra per latència de xarxa
        self.nir = LenzClient(nir_ip, timeout=15.0)

        try:
            self.vis.connect()
            self.nir.connect()
            self.get_logger().info("✓ VIS & NIR connectats. Keepalive PNG actiu.")
            time.sleep(2)

            self.get_logger().info("Configurant paràmetres de hardware...")
            self._configure_sensors(self.vis_mtr, self.vis_mti, self.nir_mtr, self.nir_mti)

            self.get_logger().info("Executant seqüència de calibratge obligatòria (GDS → GRF)...")
            self._calibrate_sensors()

            self.get_logger().info("✓ Sensors configurats, calibrats i llestos.")

        except Exception as e:
            self.get_logger().error(f"✗ Fallada a l'inici: {e}")
            raise

        self.get_logger().info("HyperspectralNode llest.")

    # ----------------------------------------------------------
    # Helper: Matplotlib backend (Ubuntu / headless)
    # ----------------------------------------------------------
    def _setup_matplotlib_backend(self):
        """Detecta el millor backend disponible. Desactiva plotting si no hi ha GUI."""
        import matplotlib
        has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        if not has_display:
            matplotlib.use('Agg')
            self.enable_plotting = False
            self.get_logger().info("Sense GUI — plotting desactivat (Agg)")
            return

        for backend in ['TkAgg', 'Qt5Agg', 'GTK3Agg', 'Agg']:
            try:
                matplotlib.use(backend)
                import matplotlib.pyplot as _plt
                fig = _plt.figure()
                _plt.close(fig)
                if backend == 'Agg':
                    self.enable_plotting = False
                    self.get_logger().info("Backend Agg — plotting desactivat (no interactiu)")
                else:
                    self.get_logger().info(f"✓ Backend matplotlib: {backend}")
                return
            except (ImportError, RuntimeError):
                continue

        self.enable_plotting = False
        self.get_logger().warning("Cap backend matplotlib interactiu disponible")

    # ----------------------------------------------------------
    # Helper: Lectura paral·lela de sensor (thread-safe via dict compartit)
    # ----------------------------------------------------------
    def _read_sensor_into(self, client, label, results_dict):
        """Llegeix un frame del sensor i el desa al dict de resultats compartit."""
        try:
            results_dict[label] = client.read_frame()
        except Exception as e:
            results_dict[label] = f"ERROR: {e}"

    # ----------------------------------------------------------
    # Helper: Validació estricta de trama (hardware-level)
    # ----------------------------------------------------------
    def _is_valid_frame(self, sensor_dict, sensor_name):
        """
        Comprova les condicions mínimes d'una trama vàlida:
          - Ha de ser un dict amb clau 'spectrum' de 256 punts
          - Tots els valors han de ser finits
          - NIR: menys del 10% de zeros absoluts (buffer TCP incomplet)
          - Intensitat mitjana > 100 counts (sub-exposició severa)
        """
        if not isinstance(sensor_dict, dict) or "spectrum" not in sensor_dict:
            return False

        arr = np.array(sensor_dict["spectrum"], dtype=np.float64)

        if len(arr) != 256:
            self.get_logger().warning(f"{sensor_name} longitud invàlida: {len(arr)}")
            return False

        if not np.all(np.isfinite(arr)):
            self.get_logger().warning(f"{sensor_name} conté valors no finits")
            return False

        if sensor_name == "NIR":
            zero_ratio = float(np.sum(arr == 0.0)) / len(arr)
            if zero_ratio > 0.1:
                self.get_logger().warning(
                    f"NIR trama corrupta: {zero_ratio*100:.1f}% zeros (buffer TCP incomplet)"
                )
                return False

        if np.mean(arr) < 100.0:
            self.get_logger().warning(
                f"{sensor_name} sub-exposat: mean={np.mean(arr):.1f} < 100 counts"
            )
            return False

        return True

    # ----------------------------------------------------------
    # Helper: Finestra de captura segura (protocol keepalive)
    # ----------------------------------------------------------
    def _prepare_capture_window(self):
        """
        Obre una finestra d'adquisició sincronitzada:
          - VIS: keepalive SEMPRE actiu; sincronitzar post-ping per no col·lidir
          - NIR: pausar keepalive ABANS de la comanda; el sensor es talla si rep PNGs durant captura
        """
        self.vis.flush_buffer()
        self.nir.flush_buffer()
        time.sleep(0.1)
        self.vis.wait_after_ping(0.5)
        self.nir.pause_keepalive()
        self.nir.wait_after_ping(0.5)

    def _close_capture_window(self):
        """Restaura el keepalive NIR i neteja els buffers residuals."""
        try:
            self.nir.resume_keepalive()
        except Exception as e:
            self.get_logger().warning(f"No s'ha pogut restaurar keepalive NIR: {e}")
        time.sleep(0.1)
        self.vis.flush_buffer()
        self.nir.flush_buffer()

    # ----------------------------------------------------------
    # Helper: Enviar comanda de configuració (protocol TCP segur)
    # ----------------------------------------------------------
    def _send_config_safe(self, sensor_name, client, cmd, value):
        """
        Envia MTI/MTR/THP amb el protocol correcte per cada sensor.
        El flush post-enviament consumeix l''OK' de confirmació del sensor
        per evitar que es confongui amb la capçalera del proper espectre.
        """
        if sensor_name == "VIS":
            client.flush_buffer()
            time.sleep(0.1)
            self.vis.wait_after_ping(0.5)
            client.send_config(cmd, value)
            time.sleep(0.2)
            client.flush_buffer()
            return

        client.flush_buffer()
        time.sleep(0.1)
        self.nir.pause_keepalive()
        try:
            self.nir.wait_after_ping(0.5)
            client.send_config(cmd, value)
            time.sleep(0.2)
            client.flush_buffer()
        finally:
            try:
                self.nir.resume_keepalive()
            except Exception as e:
                self.get_logger().warning(f"No s'ha pogut restaurar keepalive NIR post-config: {e}")

    # ----------------------------------------------------------
    # Helper: Configurar sensors
    # ----------------------------------------------------------
    def _configure_sensors(self, vis_mtr, vis_mti, nir_mtr, nir_mti):
        """Envia MTR, MTI i THP a ambdós sensors i actualitza l'estat intern."""
        self._send_config_safe("VIS", self.vis, "MTR", vis_mtr)
        self._send_config_safe("VIS", self.vis, "MTI", vis_mti)
        self._send_config_safe("VIS", self.vis, "THP", 1)
        self._send_config_safe("NIR", self.nir, "MTR", nir_mtr)
        self._send_config_safe("NIR", self.nir, "MTI", nir_mti)
        self._send_config_safe("NIR", self.nir, "THP", 1)
        self.vis_mtr = vis_mtr
        self.vis_mti = vis_mti
        self.nir_mtr = nir_mtr
        self.nir_mti = nir_mti
        time.sleep(1.5)  # Estabilització del hardware post-configuració

    # ----------------------------------------------------------
    # Helper: Seqüència de calibratge obligatòria amb retry
    # ----------------------------------------------------------
    def _calibrate_sensors(self):
        """
        Executa GDS → GRF amb fins a 3 reintents per pas.
        Si cap intent retorna una trama vàlida, llança RuntimeError
        per evitar que el node arrenqui amb calibracions nul·les.
        """
        if self.auto_configure:
            self.get_logger().info("Auto-Configurador ACTIU → executant Pas 0 (MTI Bracketing)...")
            self._auto_calibrate_mti()
        else:
            self.get_logger().warning(
                f"Auto-Configurador DESACTIVAT: "
                f"MTI_VIS={self.vis_mti} µs, MTI_NIR={self.nir_mti} µs (manuals)."
            )

        sequence = [
            ("GDS", "Dark Current  → POSA LA TAPA A LA CÀMERA"),
            ("GRF", "White Reference → TREU LA TAPA I POSA EL PAPER BLANC a Z=30 mm"),
        ]

        for cmd, instruction in sequence:
            input(f"\n>>> {instruction}\n    Prem ENTER quan estiguis a punt...")
            self.get_logger().info(f"Capturant {cmd}...")

            # Temps d'integració adaptatiu: max(MTI) en µs → segons + marge de xarxa
            acq_wait = max(self.vis_mti, self.nir_mti) / 1_000_000.0 + 0.5

            success = False
            for attempt in range(3):
                results = {}
                try:
                    self._prepare_capture_window()
                    self.vis.send_simple(cmd)
                    self.nir.send_simple(cmd)
                    time.sleep(acq_wait)

                    t_vis = threading.Thread(
                        target=self._read_sensor_into, args=(self.vis, "VIS", results)
                    )
                    t_nir = threading.Thread(
                        target=self._read_sensor_into, args=(self.nir, "NIR", results)
                    )
                    t_vis.start(); t_nir.start()
                    t_vis.join();  t_nir.join()
                finally:
                    self._close_capture_window()

                if (self._is_valid_frame(results.get("VIS"), "VIS") and
                        self._is_valid_frame(results.get("NIR"), "NIR")):
                    self.calibration_data[cmd] = {
                        "vis":       list(results["VIS"]["spectrum"]),
                        "nir":       list(results["NIR"]["spectrum"]),
                        "timestamp": datetime.now().isoformat(),
                    }
                    self.get_logger().info(f"✓ {cmd} capturat i guardat en memòria.")
                    success = True
                    break

                self.get_logger().warning(
                    f"[Calibratge {attempt+1}/3] Trama {cmd} invàlida. Reintentant..."
                )
                time.sleep(0.5)

            if not success:
                raise RuntimeError(
                    f"No s'ha pogut obtenir una trama vàlida per a {cmd} "
                    "després de 3 intents. Comprova la càmera i reinicia el node."
                )

            time.sleep(1.5)  # Pausa entre GDS i GRF per estabilitzar el sensor

    # ----------------------------------------------------------
    # Helper: Pas 0 — Fixació d'MTI per a la sessió (MTI Bracketing)
    # ----------------------------------------------------------
    def _auto_calibrate_mti(self):
        """
        Menú interactiu per escollir l'MTI fix de la sessió.
        Cap auto-ajust matemàtic: regles fixes basades en R+D.
        NIR sempre fixat al sweet spot (23750 µs = 23.75 ms).
        """
        print("\n" + "=" * 60)
        print("  PAS 0: FIXACIÓ D'EXPOSICIÓ — MTI SESSION")
        print("=" * 60)
        print("  Quina família de materials inspeccionaràs?")
        print("  [1] Clars / Metalls               →  VIS MTI: 150.000 µs (150 ms)")
        print("  [2] Estàndard (Zona Lineal Pura)  →  VIS MTI: 140.000 µs (140 ms)  ← recomanat")
        print("  [3] Foscos / Absorbents            →  VIS MTI: 210.000 µs (210 ms)  ← Soft Ceiling moderat")

        mti_options = {'1': 150000, '2': 140000, '3': 210000}
        while True:
            opcio = input("  Selecciona [1], [2] o [3]: ").strip()
            if opcio in mti_options:
                new_mti_vis = mti_options[opcio]
                break
            print("  ❌ Opció invàlida. Torna a intentar-ho.")

        new_mti_nir = 23750  # Sweet spot NIR: lineal, independent de la temperatura

        self._send_config_safe("VIS", self.vis, "MTI", new_mti_vis)
        self._send_config_safe("NIR", self.nir, "MTI", new_mti_nir)

        self.vis_mti = new_mti_vis
        self.nir_mti = new_mti_nir

        self.get_logger().info(
            f"✓ MTI fixat → VIS: {new_mti_vis} µs ({new_mti_vis/1000:.1f} ms) | "
            f"NIR: {new_mti_nir} µs ({new_mti_nir/1000:.2f} ms)"
        )
        time.sleep(1.0)  # Estabilització tèrmica post-configuració

    # ----------------------------------------------------------
    # SERVICE: Configuració d'un sensor
    # ----------------------------------------------------------
    def configure_callback(self, request, response):
        """
        Configura MTR, MTI o THP d'un sensor.
        Request:  sensor ('VIS'/'NIR'), command ('MTR'/'MTI'/'THP'), value (int)
        Response: success (bool), message (str)
        """
        sensor_name = request.sensor.upper()
        cmd         = request.command.upper()
        value       = request.value
        self.get_logger().info(f"Config: {sensor_name} {cmd}={value}")

        client = self.vis if sensor_name == "VIS" else self.nir
        try:
            self._send_config_safe(sensor_name, client, cmd, value)

            if sensor_name == "VIS":
                if cmd == "MTR": self.vis_mtr = value
                elif cmd == "MTI": self.vis_mti = value
            else:
                if cmd == "MTR": self.nir_mtr = value
                elif cmd == "MTI": self.nir_mti = value

            response.success = True
            response.message = f"OK: {sensor_name} {cmd}={value}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"ERROR: {e}"
            self.get_logger().error(response.message)

        return response

    # ----------------------------------------------------------
    # SERVICE: Mesura (GDS / GRF / GSM) amb retry automàtic
    # ----------------------------------------------------------
    def measurement_callback(self, request, response):
        """
        Captura un espectre hiperspectral amb protocol TCP segur i fins a 3 reintents.
        Request:  command ('GDS'/'GRF'/'GSM'), x_coord, y_coord, z_coord
        Response: vis_ok, vis_spectrum, vis_status, nir_ok, nir_spectrum, nir_status, message
        Format missatge: 'OK|<csv_path>' o 'ERROR|<detall>'
        """
        cmd     = request.command.upper()
        x_coord = request.x_coord
        y_coord = request.y_coord
        z_coord = request.z_coord
        self.get_logger().info(f"Mesura: {cmd} @ ({x_coord:.3f}, {y_coord:.3f}, {z_coord:.3f})")

        # --- Comandes meta (no toquen el maquinari) ---
        if cmd == "GET_MTI":
            response.vis_ok  = True
            response.nir_ok  = True
            response.message = f"{self.vis_mti}|{self.nir_mti}"
            return response

        if cmd.startswith("GET_"):
            return self._get_stored_calibration(cmd[4:], response)

        # Temps d'espera adaptatiu: max(MTI en µs) → s + 0.5 s de marge de xarxa
        acq_wait = max(self.vis_mti, self.nir_mti) / 1_000_000.0 + 0.5

        results = {}
        for attempt in range(3):
            # RESET CRÍTIC: evita barrejar frames de captures d'intents diferents
            results = {}
            try:
                self._prepare_capture_window()

                self.get_logger().info(f"[Intent {attempt+1}/3] Enviant {cmd}...")
                self.vis.send_simple(cmd)
                self.nir.send_simple(cmd)
                time.sleep(acq_wait)

                t_vis = threading.Thread(
                    target=self._read_sensor_into, args=(self.vis, "VIS", results)
                )
                t_nir = threading.Thread(
                    target=self._read_sensor_into, args=(self.nir, "NIR", results)
                )
                t_vis.start(); t_nir.start()
                t_vis.join();  t_nir.join()

                if (self._is_valid_frame(results.get("VIS"), "VIS") and
                        self._is_valid_frame(results.get("NIR"), "NIR")):
                    break

                self.get_logger().warning(f"[Intent {attempt+1}/3] Trama invàlida.")
                if attempt == 2:
                    response.vis_ok  = False
                    response.nir_ok  = False
                    response.message = "ERROR|Trama invàlida després de 3 intents"
                    return response

                # Pausa entre reintents: evita buffer overflow (coll d'ampolla de maquinari)
                time.sleep(0.5)

            except (BrokenPipeError, ConnectionError, OSError, Exception) as e:
                self.get_logger().warning(f"[Intent {attempt+1}/3] Excepció TCP: {e}")
                if attempt < 2:
                    self.get_logger().info("Reconnectant als sensors...")
                    try:
                        self.vis.close(); self.nir.close()
                    except Exception:
                        pass
                    time.sleep(2)
                    self.vis = LenzClient(self.get_parameter("vis_ip").value, timeout=20.0)
                    self.nir = LenzClient(self.get_parameter("nir_ip").value, timeout=15.0)
                    self.vis.connect(); self.nir.connect()
                    time.sleep(3)
                    self._configure_sensors(self.vis_mtr, self.vis_mti, self.nir_mtr, self.nir_mti)
                    time.sleep(1)
                else:
                    response.vis_ok  = False
                    response.nir_ok  = False
                    response.message = f"ERROR|Fallada TCP definitiva: {e}"
                    return response
            finally:
                self._close_capture_window()

        # --- Processar i publicar resultats VIS ---
        if isinstance(results.get("VIS"), dict) and "spectrum" in results["VIS"]:
            response.vis_ok       = True
            response.vis_spectrum = list(results["VIS"]["spectrum"])
            response.vis_status   = results["VIS"].get("status", 0)
            vis_msg = Float32MultiArray()
            vis_msg.data = [float(v) for v in response.vis_spectrum]
            self.vis_pub.publish(vis_msg)
        else:
            response.vis_ok       = False
            response.vis_spectrum = []
            response.vis_status   = -1
            if isinstance(results.get("VIS"), str):
                self.get_logger().warning(f"VIS error: {results['VIS']}")

        # --- Processar i publicar resultats NIR ---
        if isinstance(results.get("NIR"), dict) and "spectrum" in results["NIR"]:
            response.nir_ok       = True
            response.nir_spectrum = list(results["NIR"]["spectrum"])
            response.nir_status   = results["NIR"].get("status", 0)
            nir_msg = Float32MultiArray()
            nir_msg.data = [float(v) for v in response.nir_spectrum]
            self.nir_pub.publish(nir_msg)
        else:
            response.nir_ok       = False
            response.nir_spectrum = []
            response.nir_status   = -1
            if isinstance(results.get("NIR"), str):
                self.get_logger().warning(f"NIR error: {results['NIR']}")

        # Actualitzar calibracions en memòria si la comanda és GDS o GRF
        if cmd in ("GDS", "GRF") and response.vis_ok and response.nir_ok:
            self.calibration_data[cmd] = {
                "vis":       response.vis_spectrum,
                "nir":       response.nir_spectrum,
                "timestamp": datetime.now().isoformat(),
            }
            self.get_logger().info(f"✓ Calibration {cmd} actualitzada en memòria.")

        # Missatge de resposta amb pic espectral (útil per a diagnosi ràpida)
        if response.vis_ok and response.nir_ok:
            vis_peak_wl = self.vis_wavelengths[np.argmax(response.vis_spectrum)]
            nir_peak_wl = self.nir_wavelengths[np.argmax(response.nir_spectrum)]
            self.get_logger().info(
                f"✓ {cmd} OK  |  Pic VIS: {vis_peak_wl:.1f} nm  |  Pic NIR: {nir_peak_wl:.1f} nm"
            )
            if self.enable_plotting:
                self._update_plots(response.vis_spectrum, response.nir_spectrum, cmd)
            csv_path = ""
            if self.enable_csv_logging:
                csv_path = self._log_to_csv(
                    cmd, x_coord, y_coord, z_coord,
                    response.vis_spectrum, response.nir_spectrum,
                ) or ""
            response.message = f"OK|{csv_path}"
        else:
            response.message = (
                f"ERROR|PARTIAL  VIS={'OK' if response.vis_ok else 'FAIL'}  "
                f"NIR={'OK' if response.nir_ok else 'FAIL'}"
            )
            self.get_logger().warning(response.message)

        return response

    # ----------------------------------------------------------
    # Helper: Retornar calibracions guardades en memòria
    # ----------------------------------------------------------
    def _get_stored_calibration(self, calib_type, response):
        """Retorna GDS o GRF guardats a l'inici. Usa la comanda GET_GDS / GET_GRF."""
        data = self.calibration_data.get(calib_type)
        if data and data["vis"] is not None and data["nir"] is not None:
            response.vis_ok       = True
            response.nir_ok       = True
            response.vis_spectrum = data["vis"]
            response.nir_spectrum = data["nir"]
            response.message      = f"OK: {calib_type} recuperat de memòria ({data['timestamp']})"
            self.get_logger().info(response.message)
        else:
            response.vis_ok  = False
            response.nir_ok  = False
            response.message = f"ERROR: Calibratge {calib_type} no disponible (és None)"
            self.get_logger().error(response.message)
        return response

    # ----------------------------------------------------------
    # Helper: Log intern del node (Hyperspectral_data_DD_MM_YYYY.csv)
    # ----------------------------------------------------------
    def _initialize_csv_file(self, directory):
        """Crea el fitxer CSV diari del node si no existeix."""
        try:
            os.makedirs(directory, exist_ok=True)
            now = datetime.now()
            filename = f"Hyperspectral_data_{now.day:02d}_{now.month:02d}_{now.year}.csv"
            self.csv_file_path = os.path.join(directory, filename)

            if not os.path.isfile(self.csv_file_path):
                with open(self.csv_file_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Format canònic: Type, Date, Time, Counter, X, Y, Z, ...longituds d'ona
                    header = ["Type", "Date", "Time", "Counter", "X", "Y", "Z"]
                    header += [f"{wl:.1f}" for wl in self.vis_wavelengths]
                    header += [f"{wl:.1f}" for wl in self.nir_wavelengths]
                    writer.writerow(header)
                self.get_logger().info(f"✓ CSV creat: {self.csv_file_path}")
            else:
                self.get_logger().info(f"✓ CSV existent: {self.csv_file_path}")
        except Exception as e:
            self.get_logger().error(f"✗ Error inicialitzant CSV: {e}")
            self.csv_file_path = None

    def _log_to_csv(self, cmd, x, y, z, vis_spectrum, nir_spectrum):
        """Afegeix una fila al CSV del node. Retorna la ruta o None si hi ha error."""
        if not self.csv_file_path:
            return None
        try:
            self._csv_row_counter += 1
            now = datetime.now()
            with open(self.csv_file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                row = [
                    cmd,
                    now.strftime("%Y-%m-%d"),
                    now.strftime("%H:%M:%S.%f")[:-3],
                    self._csv_row_counter,
                    x, y, z,
                ]
                row += list(vis_spectrum)
                row += list(nir_spectrum)
                writer.writerow(row)
            return self.csv_file_path
        except Exception as e:
            self.get_logger().error(f"✗ Error escrivint CSV: {e}")
            return None

    # ----------------------------------------------------------
    # Helper: Plot en temps real (raw intensity del node)
    # ----------------------------------------------------------
    def _update_plots(self, vis_spectrum, nir_spectrum, cmd):
        """Actualitza les finestres matplotlib amb el darrer espectre capturat."""
        try:
            self.ax_vis.clear()
            self.ax_nir.clear()

            self.ax_vis.plot(self.vis_wavelengths, vis_spectrum, 'b-', lw=1.5, label=cmd)
            self.ax_vis.set_title(f'VIS — {cmd}')
            self.ax_vis.set_xlabel('Wavelength (nm)')
            self.ax_vis.set_ylabel('Intensity (raw counts)')
            self.ax_vis.set_xlim(325.3, 792.6)
            self.ax_vis.grid(True, alpha=0.3)
            self.ax_vis.legend()

            self.ax_nir.plot(self.nir_wavelengths, nir_spectrum, 'r-', lw=1.5, label=cmd)
            self.ax_nir.set_title(f'NIR — {cmd}')
            self.ax_nir.set_xlabel('Wavelength (nm)')
            self.ax_nir.set_ylabel('Intensity (raw counts)')
            self.ax_nir.set_xlim(991.0, 1707.0)
            self.ax_nir.grid(True, alpha=0.3)
            self.ax_nir.legend()

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
        except Exception as e:
            self.get_logger().warning(f"Plot update fallat: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HyperspectralNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.vis.close()
        node.nir.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

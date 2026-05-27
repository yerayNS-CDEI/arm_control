#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import csv
import numpy as np
from datetime import datetime, timezone
import sys
import argparse
from rclpy.utilities import remove_ros_args
import time
import matplotlib.pyplot as plt

from arm_control.srv import HyperspectralCommand, HyperspectralConfig, PredictMaterial

# Longitud esperada dels espectres (ha de coincidir amb hyperspectral_node.py)
SPECTRUM_LENGTH = 256

class InspectionManager(Node):
    def __init__(self, enable_plot=False):
        super().__init__('inspection_manager')
        self.camera_client = self.create_client(HyperspectralCommand, 'hyperspectral/measurement')
        self.config_client  = self.create_client(HyperspectralConfig,  'hyperspectral/configure')
        self.ml_client      = self.create_client(PredictMaterial,       'hyperspectral/predict_material')
        self.counter = 0

        self._vis_wls = np.linspace(325.3, 792.6, SPECTRUM_LENGTH)
        self._nir_wls = np.linspace(991.0, 1707.0, SPECTRUM_LENGTH)
        self._last_mti = (None, None)

        self.enable_plot = enable_plot
        if self.enable_plot:
            plt.ion() # Mode interactiu (no bloqueja ROS 2)
            # Creem dos subgràfics (un al costat de l'altre) per VIS i NIR
            self.fig, (self.ax_vis, self.ax_nir) = plt.subplots(1, 2, figsize=(12, 5))
            try:
                self.fig.canvas.manager.set_window_title('Signatura Química - Reflectància')
            except Exception:
                pass

    # ----------------------------------------------------------
    # Helper: Configuracio d'un sensor
    # ----------------------------------------------------------
    def configure_sensor(self, sensor, command, value):
        """Configura un parametre (MTR, MTI, THP) d'un sensor VIS o NIR."""
        req2 = HyperspectralConfig.Request()
        req2.sensor  = sensor.upper()
        req2.command = command.upper()
        req2.value   = int(value)

        self.get_logger().info(f"  Configurant {sensor.upper()} {command.upper()}={value}...")
        future = self.config_client.call_async(req2)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is None:
            self.get_logger().error(f"  X Cap resposta en configurar {sensor} {command}.")
            return False
        if response.success:
            self.get_logger().info(f"  OK {response.message}")
        else:
            self.get_logger().error(f"  X {response.message}")
        return response.success

    # ----------------------------------------------------------
    # Helper: Actualitza la finestra del gràfic amb la nova mesura.
    # ----------------------------------------------------------
    def _update_plot(self, vis_norm, nir_norm, title_info):
        if not self.enable_plot: return
        
        vis_wls = self._vis_wls
        nir_wls = self._nir_wls

        self.ax_vis.clear()
        self.ax_nir.clear()
        
        # Plot VIS
        self.ax_vis.plot(vis_wls, vis_norm, color='#1f77b4', linewidth=2)
        self.ax_vis.set_title('Espectre VIS')
        self.ax_vis.set_xlabel('Longitud d\'ona (nm)')
        self.ax_vis.set_ylabel('Reflectància')
        self.ax_vis.set_ylim(-0.1, 1.2) # Limitem de 0 a 1 (amb una mica de marge visual)
        self.ax_vis.grid(True, linestyle='--', alpha=0.6)
        
        # Plot NIR
        self.ax_nir.plot(nir_wls, nir_norm, color='#d62728', linewidth=2)
        self.ax_nir.set_title('Espectre NIR')
        self.ax_nir.set_xlabel('Longitud d\'ona (nm)')
        self.ax_nir.set_ylim(-0.1, 1.2)
        self.ax_nir.grid(True, linestyle='--', alpha=0.6)
        
        # Títol global amb el material detectat o l'etiqueta
        self.fig.suptitle(f"Inspecció: {title_info}", fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.pause(0.05)

    # ----------------------------------------------------------
    # Helper: Obtenir calibracio guardada al node
    # ----------------------------------------------------------
    def get_calibration_data(self, cmd_type):
        """Demana les dades de calibracio a la memoria del hyperspectral_node."""
        req = HyperspectralCommand.Request()
        req.command = f"GET_{cmd_type}"
        future = self.camera_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None:
            self.get_logger().error(f"Cap resposta per GET_{cmd_type}.")
        return result

    # ----------------------------------------------------------
    # Helper: Parsejar el missatge de resposta del node
    # Format nou: "OK|/path/to/csv"  o  "ERROR|detall"
    # ----------------------------------------------------------
    def _parse_response_message(self, message):
        """Retorna (ok: bool, detail: str) a partir del format 'STATUS|detail'."""
        if '|' in message:
            status, detail = message.split('|', 1)
            return status.strip().upper() == 'OK', detail.strip()
        # Compatibilitat cap enrere amb el format antic
        return message.upper().startswith('OK'), message

    # ----------------------------------------------------------
    # Helper: Validar longitud dels espectres
    # ----------------------------------------------------------
    def _validate_spectrum(self, spectrum, label):
        """Comprova que l'espectre tingui SPECTRUM_LENGTH punts."""
        if len(spectrum) != SPECTRUM_LENGTH:
            self.get_logger().error(
                f"Longitud invalida per a {label}: {len(spectrum)} (esperats {SPECTRUM_LENGTH})"
            )
            return False
        return True

    # ----------------------------------------------------------
    # Helper: Log calibration data only when it changes (hash-based)
    # ----------------------------------------------------------
    def _log_calibration_if_new(self, res_gds, res_grf, counter):
        """Logs calibration data (GDS, GRF) only when it changes from previous calibration."""
        csv_dir = os.path.expanduser("~/Documents/Hyperspectral_camera/src/arm_control/resource/raw_data")
        os.makedirs(csv_dir, exist_ok=True)
        
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        file_path = os.path.join(csv_dir, f"raw_intensity_{date_str}.csv")
        state_file = os.path.join(csv_dir, ".calib_state")
        
        # Hash to detect if calibration has changed
        current_calib_sum = (sum(res_grf.vis_spectrum) + sum(res_gds.vis_spectrum) + sum(res_grf.nir_spectrum) + sum(res_gds.nir_spectrum))
        saved_calib_sum = None
        if os.path.exists(state_file):
            try:
                with open(state_file, 'r') as f:
                    saved_calib_sum = float(f.read().strip())
            except:
                pass
        
        # Only log if calibration has changed
        if saved_calib_sum != current_calib_sum:
            with open(state_file, 'w') as f:
                f.write(str(current_calib_sum))
            
            file_exists = os.path.isfile(file_path)
            time_str = now.strftime("%H:%M:%S.%f")[:-3]
            
            with open(file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    header = ["Type", "Date", "Time", "Counter", "Label", "Class"]
                    header.extend([f'{wl:.4f}' for wl in self._vis_wls])
                    header.extend([f'{wl:.0f}' for wl in self._nir_wls])
                    writer.writerow(header)

                writer.writerow(["GDS", date_str, time_str, counter, "Calibration", "Calibration"] + list(res_gds.vis_spectrum) + list(res_gds.nir_spectrum))
                writer.writerow(["GRF", date_str, time_str, counter, "Calibration", "Calibration"] + list(res_grf.vis_spectrum) + list(res_grf.nir_spectrum))
            
            self.get_logger().info("🔄 New calibration detected! Saved GDS and GRF to raw_intensity record.")

    # ----------------------------------------------------------
    # Helper: Log raw intensity measurements
    # ----------------------------------------------------------
    def _log_raw_intensity(self, vis_spectrum, nir_spectrum, counter, label="Unknown"):
        csv_dir = os.path.expanduser("~/Documents/Hyperspectral_camera/src/arm_control/resource/raw_data")
        os.makedirs(csv_dir, exist_ok=True)

        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S.%f")[:-3]

        file_path = os.path.join(csv_dir, f"raw_intensity_{date_str}.csv")
        file_exists = os.path.isfile(file_path)

        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                header = ["Type", "Date", "Time", "Counter", "Label", "Class"]
                header.extend([f'{wl:.4f}' for wl in self._vis_wls])
                header.extend([f'{wl:.0f}' for wl in self._nir_wls])
                writer.writerow(header)

            writer.writerow(["GSM", date_str, time_str, counter, label, label] + list(vis_spectrum) + list(nir_spectrum))

    # ----------------------------------------------------------
    # Helper: Log reflectance measurements
    # ----------------------------------------------------------
    def _log_reflectance(self, vis_norm, nir_norm, counter, label="Unknown"):
        csv_dir = os.path.expanduser("~/Documents/Hyperspectral_camera/src/arm_control/resource/reflectance")
        os.makedirs(csv_dir, exist_ok=True)
        
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S.%f")[:-3]
        
        file_path = os.path.join(csv_dir, f"reflectance_{date_str}.csv")
        file_exists = os.path.isfile(file_path)
        
        with open(file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                header = ["Date", "Time", "Counter", "Label", "Class"]
                header.extend([f'{wl:.4f}' for wl in self._vis_wls])
                header.extend([f'{wl:.0f}' for wl in self._nir_wls])
                writer.writerow(header)

            writer.writerow([date_str, time_str, counter, label, label] + vis_norm.tolist() + nir_norm.tolist())

    # ----------------------------------------------------------
    # Helper: Guarda valors MTI al log (per monitoritzar estabilitat i calibració al llarg del temps)
    # ----------------------------------------------------------
    def _log_mti(self, vis_mti, nir_mti):
        if (vis_mti, nir_mti) == self._last_mti:
            return
        self._last_mti = (vis_mti, nir_mti)
        file_path = os.path.expanduser("~/Documents/Hyperspectral_camera/src/arm_control/resource/mti_log.json")

        entry = {
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "VIS_MTI": vis_mti,
            "NIR_MTI": nir_mti
        }

        data = []
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as f:
                    data = json.load(f)
            except:
                data = []

        data.append(entry)

        with open(file_path, 'w') as f:
            json.dump(data, f, indent=4)

    # ----------------------------------------------------------
    # Helper: Comprova l'estabilitat de les mesures
    # ----------------------------------------------------------
    def _check_stability(self, vis, nir):
        vis_check = np.array(vis)[40:-20]
        nir_check = np.array(nir)[20:-20]

        # 1. Soroll i Salts sobtats (pics)
        vis_noise = np.std(np.diff(vis_check))
        nir_noise = np.std(np.diff(nir_check))
        vis_max_jump = np.max(np.abs(np.diff(vis_check)))
        nir_max_jump = np.max(np.abs(np.diff(nir_check)))

        # 2. Energia, Saturació i extrems negatius
        vis_mean = np.mean(vis_check)
        vis_max = np.max(vis_check)
        nir_max = np.max(nir_check)
        vis_min = np.min(vis_check)

        # 3. Zeros TCP
        nir_zero_ratio = np.sum(np.array(nir) == 0) / len(nir)

        if vis_noise > 0.2:
            self.get_logger().error(f"❌ Rebutjat: Excés de soroll VIS ({vis_noise:.3f} > 0.200)")
            return False
        if nir_noise > 0.15:
            self.get_logger().error(f"❌ Rebutjat: Excés de soroll NIR ({nir_noise:.3f} > 0.200)")
            return False
        if vis_max_jump > 0.6:
            self.get_logger().error(f"❌ Rebutjat: Pic sobtat de soroll al VIS ({vis_max_jump:.3f})")
            return False
        if nir_max_jump > 0.5:
            self.get_logger().error(f"❌ Rebutjat: Pic sobtat de soroll al NIR ({nir_max_jump:.3f})")
            return False
        if vis_mean < 0.01:
            self.get_logger().error(f"❌ Rebutjat: Sub-exposició VIS ({vis_mean:.3f})")
            return False
        if vis_max > 2.0 or nir_max > 2.5:
            self.get_logger().error(f"❌ Rebutjat: Reflexió especular extremada o saturació (VIS: {vis_max:.2f}, NIR: {nir_max:.2f})")
            return False
        if vis_min < -0.2:
            self.get_logger().error(f"❌ Rebutjat: Valors negatius severs per mal calibratge ({vis_min:.2f})")
            return False
        if nir_zero_ratio > 0.1:
            self.get_logger().error(f"❌ Rebutjat: Massa zeros a la trama NIR ({nir_zero_ratio*100:.1f}%)")
            return False
        
        return True

    def _capture_gsm_once(self, x, y, z):
        req_cam = HyperspectralCommand.Request()
        req_cam.command = "GSM"
        req_cam.x_coord = float(x)
        req_cam.y_coord = float(y)
        req_cam.z_coord = float(z)

        future_cam = self.camera_client.call_async(req_cam)
        rclpy.spin_until_future_complete(self, future_cam)
        res_cam = future_cam.result()

        if res_cam is None:
            self.get_logger().warning("GSM capture failed: no response from camera.")
            return None

        cam_ok, cam_detail = self._parse_response_message(res_cam.message)

        if not res_cam.vis_ok or not res_cam.nir_ok or not cam_ok:
            self.get_logger().warning(
                f"GSM capture rejected: "
                f"VIS={'OK' if res_cam.vis_ok else f'FAIL status={res_cam.vis_status}'}, "
                f"NIR={'OK' if res_cam.nir_ok else f'FAIL status={res_cam.nir_status}'} | "
                f"{cam_detail}"
            )
            return None

        if not self._validate_spectrum(res_cam.vis_spectrum, "GSM VIS"):
            return None
        if not self._validate_spectrum(res_cam.nir_spectrum, "GSM NIR"):
            return None

        return res_cam
    
    def _capture_gsm_multicapture(self, x, y, z, captures=5, capture_delay=0.7):
        valid_captures = []

        captures = max(1, int(captures))

        self.get_logger().info(
            f"Capturant GSM multi-capture: {captures} captures, delay={capture_delay:.2f}s"
        )

        for i in range(captures):
            self.get_logger().info(f"  GSM capture {i + 1}/{captures}")
            res = self._capture_gsm_once(x, y, z)

            if res is not None:
                valid_captures.append(res)
            else:
                self.get_logger().warning(f"  Capture {i + 1}/{captures} descartada.")

            if i < captures - 1:
                time.sleep(capture_delay)

        min_valid = max(1, int(np.ceil(captures * 0.6)))

        if len(valid_captures) < min_valid:
            self.get_logger().error(
                f"Massa poques captures GSM vàlides: {len(valid_captures)}/{captures}. "
                f"Mínim requerit: {min_valid}."
            )
            return None

        vis_stack = np.array(
            [r.vis_spectrum for r in valid_captures],
            dtype=np.float64
        )
        nir_stack = np.array(
            [r.nir_spectrum for r in valid_captures],
            dtype=np.float64
        )

        # Mediana robusta: millor que la mitjana si hi ha una captura puntual dolenta.
        vis_robust = np.median(vis_stack, axis=0)
        nir_robust = np.median(nir_stack, axis=0)

        # Diagnòstic temporal: variabilitat relativa entre captures.
        vis_temporal_cv = float(np.mean(np.std(vis_stack, axis=0) / (np.mean(vis_stack, axis=0) + 1e-9)))
        nir_temporal_cv = float(np.mean(np.std(nir_stack, axis=0) / (np.mean(nir_stack, axis=0) + 1e-9)))

        self.get_logger().info(
            f"Multi-capture OK: {len(valid_captures)}/{captures} vàlides | "
            f"VIS temporal CV={vis_temporal_cv:.4f}, NIR temporal CV={nir_temporal_cv:.4f}"
        )

        return {
            "vis_spectrum": vis_robust,
            "nir_spectrum": nir_robust,
            "valid_count": len(valid_captures),
            "requested_count": captures,
            "vis_temporal_cv": vis_temporal_cv,
            "nir_temporal_cv": nir_temporal_cv,
            "representative_response": valid_captures[-1],
        }
    # ----------------------------------------------------------
    # Inspeccio principal
    # ----------------------------------------------------------
    def run_inspection(self, x, y, z,
                       mode='predict', label='Unknown',
                       zone='default',
                       vis_mtr=None, vis_mti=None,
                       nir_mtr=None, nir_mti=None,
                       vis_thp=None, nir_thp=None,
                       captures = 5, capture_delay = 0.7):
        
        self.get_logger().info("1. Esperant que els serveis estiguin actius...")
        self.camera_client.wait_for_service()
        self.config_client.wait_for_service()
        if mode == 'predict':
            self.ml_client.wait_for_service()

        # --- Configuracio opcional dels sensors ---
        sensor_configs = [
            ("VIS", "MTR", vis_mtr),
            ("VIS", "MTI", vis_mti),
            ("VIS", "THP", vis_thp),
            ("NIR", "MTR", nir_mtr),
            ("NIR", "MTI", nir_mti),
            ("NIR", "THP", nir_thp),
        ]
        configs_a_enviar = [(s, c, v) for s, c, v in sensor_configs if v is not None]

        if configs_a_enviar:
            self.get_logger().info("2. Configurant sensors...")
            for sensor, cmd, val in configs_a_enviar:
                if not self.configure_sensor(sensor, cmd, val):
                    self.get_logger().warning(
                        f"  Avertencia: {sensor} {cmd}={val} ha fallat. "
                        "Es continua amb la configuracio actual del node."
                    )
        else:
            self.get_logger().info("2. Cap configuracio de sensors especificada. S'usa la del node.")

        # --- Recuperar calibracions ---
        self.get_logger().info("3. Recuperant calibracions (GDS, GRF) del node...")
        res_gds = self.get_calibration_data("GDS")
        res_grf = self.get_calibration_data("GRF")
        
        # 1. RECUPERAR L'MTI REAL DEL NODE
        self.get_logger().info("   Recuperant la configuració MTI final del node...")
        req_mti = HyperspectralCommand.Request()
        req_mti.command = "GET_MTI"
        future_mti = self.camera_client.call_async(req_mti)
        rclpy.spin_until_future_complete(self, future_mti)
        res_mti = future_mti.result()
        
        if res_mti and res_mti.vis_ok and res_mti.nir_ok:
            try:
                real_vis_mti, real_nir_mti = res_mti.message.split('|')
                mti_vis = int(real_vis_mti)
                mti_nir = int(real_nir_mti)
            except ValueError:
                mti_vis = vis_mti if vis_mti is not None else "node_default"
                mti_nir = nir_mti if nir_mti is not None else "node_default"
        else:
            mti_vis = vis_mti if vis_mti is not None else "node_default"
            mti_nir = nir_mti if nir_mti is not None else "node_default"

        # --- LOG MTI ---
        self._log_mti(mti_vis, mti_nir)

        # 2. VALIDACIÓ ESTRICТА DE LA CALIBRACIÓ
        if (
            res_gds is None or res_grf is None or
            not res_gds.vis_ok or not res_gds.nir_ok or
            not res_grf.vis_ok or not res_grf.nir_ok or
            len(res_gds.vis_spectrum) != SPECTRUM_LENGTH or
            len(res_gds.nir_spectrum) != SPECTRUM_LENGTH or
            len(res_grf.vis_spectrum) != SPECTRUM_LENGTH or
            len(res_grf.nir_spectrum) != SPECTRUM_LENGTH
        ):
            self.get_logger().error(
                "Calibracions invàlides o incompletes (GDS/GRF VIS o NIR). "
                "La inspecció no pot continuar."
            )
            return

        # --- LOG CALIBRACIONS ---
        self.get_logger().info("   Guardant dades de calibracio (GDS, GRF)...")
        self._log_calibration_if_new(res_gds, res_grf, self.counter)

        # --- Mesura GSM ---
        self.get_logger().info(f"4. Cridant la camera hiperespectral a ({x}, {y}, {z}) amb multi-capture...")

        gsm_multi = self._capture_gsm_multicapture(
            x=x,
            y=y,
            z=z,
            captures=captures,
            capture_delay=capture_delay,
        )

        if gsm_multi is None:
            self.get_logger().error("Error: No s'ha pogut obtenir una mesura GSM robusta.")
            return

        vis_gsm = np.array(gsm_multi["vis_spectrum"], dtype=np.float64)
        nir_gsm = np.array(gsm_multi["nir_spectrum"], dtype=np.float64)

        # Màxims de la intensitat crua (ABANS de la reflectança) per detectar Hard Ceiling
        # a train_model.py: df[(df.GSM_VIS_max < 63000) & (df.GSM_NIR_max < 60000)]
        gsm_vis_max = float(np.max(vis_gsm))
        gsm_nir_max = float(np.max(nir_gsm))

        # Camps d'estat de la resposta representativa (no mutem l'objecte ROS)
        rep_res = gsm_multi["representative_response"]
        vis_gsm_int = vis_gsm.astype(int).tolist()
        nir_gsm_int = nir_gsm.astype(int).tolist()

        self.get_logger().info(f"  Mesura robusta OK: {gsm_multi['valid_count']}/{gsm_multi['requested_count']} captures vàlides")

        # Parsejar el nou format de missatge "OK|detall" o "ERROR|detall"
        cam_ok, cam_detail = self._parse_response_message(rep_res.message)

        if not rep_res.vis_ok or not rep_res.nir_ok:
            self.get_logger().error(
                f"Error: La camera no ha pogut prendre les dades. "
                f"VIS={'OK' if rep_res.vis_ok else f'FAIL (status={rep_res.vis_status})'}, "
                f"NIR={'OK' if rep_res.nir_ok else f'FAIL (status={rep_res.nir_status})'} | "
                f"{cam_detail}"
            )
            return

        self.get_logger().info(f"  Mesura OK: {cam_detail}")

        # --- Validacio de longituds ---
        for spectrum, spectrum_label in [
            (vis_gsm, "GSM VIS"),
            (nir_gsm, "GSM NIR"),
            (res_grf.vis_spectrum, "GRF VIS"),
            (res_grf.nir_spectrum, "GRF NIR"),
        ]:
            if not self._validate_spectrum(spectrum, spectrum_label):
                return

        # --- Normalitzacio ---
        self.get_logger().info("5. Normalitzant l'espectre (Reflectancia: (GSM-GDS) / (GRF-GDS))...")
        
        vis_grf = np.array(res_grf.vis_spectrum, dtype=np.float64)
        vis_gds = np.array(res_gds.vis_spectrum, dtype=np.float64)
        nir_grf = np.array(res_grf.nir_spectrum, dtype=np.float64)
        nir_gds = np.array(res_gds.nir_spectrum, dtype=np.float64)

        # 2. Fem la "Tara": Restem el soroll a la mostra i al blanc
        vis_gsm_net = vis_gsm - vis_gds
        vis_grf_net = vis_grf - vis_gds
        
        nir_gsm_net = nir_gsm - nir_gds
        nir_grf_net = nir_grf - nir_gds

        # 3. Dividim les dades netes. (Usem > 0 al divisor per evitar errors matemàtics)
        vis_norm = np.divide(vis_gsm_net, vis_grf_net, out=np.zeros(SPECTRUM_LENGTH), where=vis_grf_net > 0)
        nir_norm = np.divide(nir_gsm_net, nir_grf_net, out=np.zeros(SPECTRUM_LENGTH), where=nir_grf_net > 0)

        # Doble validacio: la normalitzacio no ha de canviar la longitud
        if not self._validate_spectrum(vis_norm, "VIS normalitzat") or \
           not self._validate_spectrum(nir_norm, "NIR normalitzat"):
            return

        if not self._check_stability(vis_norm, nir_norm):
            self.get_logger().error("❌ Sample rejected due to instability")
            return
        
        # SI ÉS ESTABLE, LLAVORS GUARDEM ELS REGISTRES OFICIALS
        self.get_logger().info("✅ Mostra estable. Guardant dades crues i reflectància...")
        self.counter += 1
        self._log_raw_intensity(vis_gsm_int, nir_gsm_int, self.counter, label)
        self._log_reflectance(vis_norm, nir_norm, self.counter, label)

        # --- Prediccio ML ---
        # ==========================================================
        # BIFURCACIÓ DELS DOS MODES
        # ==========================================================
        if mode == 'predict':
            # --- Prediccio ML ---
            self.get_logger().info("6. [MODE PREDICT] Enviant dades al model de Machine Learning...")
            req_ml = PredictMaterial.Request()
            req_ml.vis_spectrum = vis_norm.tolist()
            req_ml.nir_spectrum = nir_norm.tolist()

            future_ml = self.ml_client.call_async(req_ml)
            rclpy.spin_until_future_complete(self, future_ml)
            res_ml = future_ml.result()

            if res_ml is None or res_ml.material.startswith("ERROR"):
                self.get_logger().error("Error: Problema amb el model de ML.")
                return

            self.get_logger().info(f"  Material: {res_ml.material} | Confianca: {res_ml.confidence:.3f}")
            self._update_plot(vis_norm, nir_norm, f"{res_ml.material} (Conf: {res_ml.confidence*100:.1f}%)")

            # ----------------------------------------------------------
            # A) CSV DE PREDICCIONS (EL DIARI)
            # ----------------------------------------------------------
            self.get_logger().info("7. Generant el CSV i l'estructura JSON...")
            now = datetime.now()
            date_str = now.strftime("%Y-%m-%d")
            time_str = now.strftime("%H:%M:%S.%f")[:-3]
            filename = f"Hyperspectral_data_{date_str}.csv"

            csv_dir = os.path.expanduser("~/hyperspectral_inspections")
            os.makedirs(csv_dir, exist_ok=True)
            csv_path = os.path.join(csv_dir, filename)
            file_exists = os.path.isfile(csv_path)

            # Comptem quantes files GSM té ja el CSV per calcular el daily_count
            daily_count = 1
            if file_exists:
                with open(csv_path, 'r') as f:
                    daily_count = sum(1 for row in csv.reader(f) if row and row[1] == "GSM") + 1

            row_index = daily_count  # índex per al JSON (coincideix amb la fila real)

            with open(csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                if not file_exists:
                    header = ['Daily_Count', 'Measure_Type', 'Date', 'Time',
                            'X_Coordinate', 'Y_Coordinate', 'Z_Coordinate',
                            'Material', 'Confidence']
                    header.extend([f'{wl:.1f}' for wl in self._vis_wls])
                    header.extend([f'{wl:.1f}' for wl in self._nir_wls])
                    writer.writerow(header)

                data_row = [daily_count, "GSM", date_str, time_str,
                            x, y, z, res_ml.material, res_ml.confidence]
                data_row.extend(vis_norm.tolist())
                data_row.extend(nir_norm.tolist())
                writer.writerow(data_row)

            # ----------------------------------------------------------
            # B) JSON DE PREDICCIONS
            # ----------------------------------------------------------
            temps_actual = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
            nou_registre = {
                "id": f"hsc-{daily_count:03d}",  # Format canònic del schema JSON
                "timestamp": temps_actual,
                "point_coordinates": {
                    "x": {"value": float(x), "error": 0.0, "unit": "m"},
                    "y": {"value": float(y), "error": 0.0, "unit": "m"},
                    "z": {"value": float(z), "error": 0.0, "unit": "m"},
                },
                "row": row_index,
                "material": res_ml.material,
                "confidence": float(res_ml.confidence),
                "raw_data": {
                    # Intensitats crues per a traçabilitat forense (sense normalitzar)
                    "gsm_vis": [int(v) for v in vis_gsm.tolist()],
                    "gsm_nir": [int(v) for v in nir_gsm.tolist()],
                    "grf_vis": list(res_grf.vis_spectrum),
                    "grf_nir": list(res_grf.nir_spectrum),
                    "gds_vis": list(res_gds.vis_spectrum),
                    "gds_nir": list(res_gds.nir_spectrum),
                }
            }

            json_dir       = os.path.expanduser("~/hyperspectral_inspections")
            os.makedirs(json_dir, exist_ok=True)
            nom_arxiu_json = os.path.join(json_dir, "registre_inspeccions.json")

            # Estructura jeràrquica per zones (wall1, floor, etc.)
            dades_historial = {}
            if os.path.exists(nom_arxiu_json):
                try:
                    with open(nom_arxiu_json, 'r') as f:
                        dades_historial = json.load(f)
                except json.JSONDecodeError:
                    dades_historial = {}

            if zone not in dades_historial:
                dades_historial[zone] = {}
            if "hyperspectral_camera" not in dades_historial[zone]:
                dades_historial[zone]["hyperspectral_camera"] = {
                    "data_pointer_csv": csv_path,
                    "measurements": []
                }

            dades_historial[zone]["hyperspectral_camera"]["data_pointer_csv"] = csv_path
            dades_historial[zone]["hyperspectral_camera"]["measurements"].append(nou_registre)

            with open(nom_arxiu_json, 'w') as f:
                json.dump(dades_historial, f, indent=4)

            self.get_logger().info(f"8. Predicció #{daily_count} guardada a zona '{zone}'!")

        elif mode == 'collect':
            # ==========================================================
            # MODE CAPTURA (NOMÉS PER CREAR DATASET D'ENTRENAMENT)
            # ==========================================================
            self.get_logger().info(f"6. [MODE COLLECT] Mostra processada per a la classe: '{label}'...")
            self._update_plot(vis_norm, nir_norm, f"Mode Collect - Etiqueta: {label}")
            self._log_training_dataset(
                vis_norm, nir_norm, label, mti_vis, mti_nir, gsm_vis_max, gsm_nir_max
            )
            self.get_logger().info("   (Dades desades a raw_intensity, reflectance i Dataset_Entrenament_Nou.csv).")
            self.get_logger().info("   (Mode Collect: NO s'actualitza el JSON d'inspeccions).")
    
    # ----------------------------------------------------------
    # Helper: Exporta al CSV d'entrenament compatible amb train_model.py
    # ----------------------------------------------------------
    def _log_training_dataset(self, vis_norm, nir_norm, label, mti_vis, mti_nir,
                               gsm_vis_max, gsm_nir_max):
        """
        Guarda l'espectre de reflectança a Dataset_Entrenament_Nou.csv amb:
          - Delimitador ';' i decimals ',' (compatibilitat amb train_model.py)
          - Columnes MTI_VIS, MTI_NIR com a metadades de traçabilitat (NO al model)
          - Columnes GSM_VIS_max, GSM_NIR_max per filtrar Hard Ceiling a train_model.py
        """
        csv_dir = os.path.expanduser(
            "~/Documents/Hyperspectral_camera/src/arm_control/resource"
        )
        os.makedirs(csv_dir, exist_ok=True)

        file_path = os.path.join(csv_dir, "Dataset_Entrenament_Nou.csv")
        file_exists = os.path.isfile(file_path)

        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d")
        time_str = now.strftime("%H:%M:%S.%f")[:-3]

        # Primera lletra de l'etiqueta com a classe (ex: "Plastic1" → "P")
        class_label = label[0].upper() if label else "U"

        with open(file_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f, delimiter=';')
            if not file_exists:
                header = ["Date", "Time", "Label", "Target_ML",
                          "MTI_VIS", "MTI_NIR", "GSM_VIS_max", "GSM_NIR_max"]
                header.extend([f'{wl:.4f}' for wl in self._vis_wls])
                header.extend([f'{wl:.0f}' for wl in self._nir_wls])
                writer.writerow(header)

            # Decimals amb coma per al format EU (necessari per pd.read_csv a train_model.py)
            def eu(v):
                return f"{float(v):.6f}".replace('.', ',')

            row = [date_str, time_str, label, class_label,
                   str(mti_vis), str(mti_nir),
                   str(int(gsm_vis_max)), str(int(gsm_nir_max))]
            row.extend([eu(v) for v in vis_norm])
            row.extend([eu(v) for v in nir_norm])
            writer.writerow(row)

        self.get_logger().info(
            f"✅ Dataset_Entrenament_Nou.csv actualitzat | Label={label} "
            f"(MTI_VIS={mti_vis}, MTI_NIR={mti_nir}, "
            f"VIS_max={gsm_vis_max:.0f}, NIR_max={gsm_nir_max:.0f})"
        )

    def run_manual_focus_test(self):
        """
        Assistent interactiu per trobar l'enfocament òptim movent el material a mà.
        """
        self.get_logger().info("="*60)
        self.get_logger().info("🚀 INICIANT ASSISTENT DE FOCUS MANUAL (KNIFE-EDGE)")
        self.get_logger().info("="*60)

        # Bucle infinit fins que l'usuari vulgui sortir
        while True:
            # 1. Demanem a quina alçada estem
            z_input = input("\n📏 Quina alçada (Z) vols provar ara? (Escriu 'q' per sortir): ")
            if z_input.lower() == 'q':
                self.get_logger().info("Surtint del mode focus. Bon treball!")
                break

            # 2. Preparem l'usuari
            print(f"\n📍 Configura el robot a Z = {z_input}.")
            print("1. Posa la meitat NEGRA del paper just sota la càmera.")
            input("2. Prem [ENTER] i comença a moure el paper lentament cap al BLANC...")

            self.get_logger().info("📸 Capturant 30 mesures (Mou el paper a poc a poc!)...")
            intensities = []

            # 3. Fem l'escombrat temporal (uns 6 segons de captura contínua)
            for i in range(30):
                req = HyperspectralCommand.Request()
                req.command = 'GSM'
                req.x_coord = 0.0 # Valors default ja que és un test manual
                req.y_coord = 0.0
                try:
                    req.z_coord = float(z_input)
                except ValueError:
                    req.z_coord = 0.0

                future = self.camera_client.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                
                res = future.result()
                
                # PROTECCIÓ CRÍTICA AFEGIDA:
                if res is not None and res.vis_ok and res.nir_ok:
                    vis_sum = np.sum(np.array(res.vis_spectrum, dtype=np.float64))
                    nir_sum = np.sum(np.array(res.nir_spectrum, dtype=np.float64))
                    total_energy = vis_sum + nir_sum
                    intensities.append(total_energy)
                else:
                    self.get_logger().warning(f"Fallada en la captura {i+1}/30. Es descarta aquest punt.")
                
                # Petita pausa perquè et doni temps a moure el paper a mà
                time.sleep(0.5) 

            # 4. Càlcul de la Nitidesa
            if len(intensities) > 1:
                # Fem la derivada (la diferència entre cada captura i la següent)
                sharpness = np.max(np.abs(np.diff(intensities)))
                
                self.get_logger().info("="*60)
                self.get_logger().info(f"✅ Escombrat finalitzat amb {len(intensities)} punts vàlids.")
                self.get_logger().info(f"🎯 NITIDESA DETECTADA A Z={z_input}:  {sharpness:.2f}")
                self.get_logger().info("="*60)
                print("Pots comparar aquest valor amb les següents alçades. Busca el número més alt!")
            else:
                self.get_logger().error("❌ No s'han pogut recollir prou dades per calcular la nitidesa.")


# ----------------------------------------------------------
# Argument parsing (compatible amb els args interns de ROS2)
# ----------------------------------------------------------
def parse_args():
    parser = argparse.ArgumentParser(
        description='Inspection Manager - captura i analitza dades hiperspectrals.'
    )
    # Posicionals
    parser.add_argument('x', type=float, nargs='?', default=0.0, help='Coordenada X (m)')
    parser.add_argument('y', type=float, nargs='?', default=0.0, help='Coordenada Y (m)')
    parser.add_argument('z', type=float, nargs='?', default=0.0, help='Coordenada Z (m)')

    #--- ARGUMENTS PER ALS MODES ---
    parser.add_argument('--mode', type=str, choices=['predict', 'collect', 'focus'], default='predict',
                        help="Mode: 'predict', 'collect', o 'focus' per test manual.")
    parser.add_argument('--label', type=str, default='Unknown',
                        help="Classe del material per entrenar (ex: P1, Fora1). Només s'usa en mode 'collect'.")
    
    # Configuracio VIS
    parser.add_argument('--vis-mtr', type=int, default=None,
                        help='VIS: temps maxim de mesura (MTR)')
    parser.add_argument('--vis-mti', type=int, default=None,
                        help='VIS: temps d integracio (MTI, ex: 65000)')
    parser.add_argument('--vis-thp', type=int, default=None,
                        help='VIS: threshold de pic (THP)')

    # Configuracio NIR
    parser.add_argument('--nir-mtr', type=int, default=None,
                        help='NIR: temps maxim de mesura (MTR, ex: 25000)')
    parser.add_argument('--nir-mti', type=int, default=None,
                        help='NIR: temps d integracio (MTI)')
    parser.add_argument('--nir-thp', type=int, default=None,
                        help='NIR: threshold de pic (THP)')
    
    parser.add_argument('--plot', action='store_true',
                        help="Mostra un gràfic en temps real de la reflectància.")

    parser.add_argument('--focus-speed', type=int, default=140000,
                        help="MTI del VIS exclusiu per al mode 'focus' (defecte: 140000).")
    parser.add_argument('--zone', type=str, default='default',
                        help="Zona d'inspecció per al JSON jeràrquic (ex: 'wall1', 'floor'). Defecte: 'default'.")
    parser.add_argument('--bracket', action='store_true',
                        help="[Mode collect] Fa 3 captures automàtiques amb MTI escalonat (80K, 140K, 200K VIS).")

    parser.add_argument('--captures', type=int, default=5,
                        help="Nombre de captures GSM per fer mitjana robusta.")

    parser.add_argument('--capture-delay', type=float, default=0.7,
                        help="Pausa en segons entre captures GSM consecutives.")
    
    # parse_known_args evita conflictes amb els args internals de ROS2
    clean_args = remove_ros_args(sys.argv)[1:]
    known, _ = parser.parse_known_args(clean_args)
    return known


def main(args=None):
    rclpy.init(args=args)
    params = parse_args()

    manager = InspectionManager(enable_plot=params.plot)
    
    if params.mode == 'focus':
        # --focus-speed és l'MTI dedicat al test de focus (defecte 140K, zona lineal pura)
        vis_speed = params.focus_speed
        nir_speed = params.nir_mti if params.nir_mti else 23750

        manager.get_logger().info("Esperant que el node de la càmera estigui actiu...")
        manager.config_client.wait_for_service()
        manager.camera_client.wait_for_service()

        manager.get_logger().info(f"Accelerant càmera per al test... (VIS MTI={vis_speed}, NIR MTI={nir_speed})")
        manager.configure_sensor("VIS", "MTI", vis_speed)
        manager.configure_sensor("NIR", "MTI", nir_speed)
        manager.run_manual_focus_test()

    elif params.mode == 'collect' and params.bracket:
        # Protocol MTI Bracketing: 3 captures (sub-exposada, òptima, Soft Ceiling moderat)
        # La calibració GDS/GRF del node es manté. Només canviem l'exposició del GSM.
        bracket_vis_mtis = [80000, 140000, 200000]
        manager.config_client.wait_for_service()
        manager.camera_client.wait_for_service()

        manager.get_logger().info(
            f"=== MODE BRACKET: 3 captures per a '{params.label}' (80K, 140K, 200K VIS) ==="
        )
        for vis_mti_b in bracket_vis_mtis:
            input(
                f"\n>>> BRACKET {vis_mti_b}: Prem ENTER per capturar "
                f"'{params.label}' amb VIS MTI={vis_mti_b}..."
            )
            manager.configure_sensor("VIS", "MTI", vis_mti_b)
            time.sleep(1.0)  # Estabilització tèrmica mínima
            manager.run_inspection(
                x=params.x, y=params.y, z=params.z,
                mode='collect', label=params.label, zone=params.zone,
                captures=params.captures, capture_delay=params.capture_delay,
            )
        manager.get_logger().info(
            f"✅ MTI Bracketing completat (80K→140K→200K) per a '{params.label}'!"
        )

    else:
        manager.run_inspection(
            x=params.x,
            y=params.y,
            z=params.z,
            mode=params.mode,
            label=params.label,
            zone=params.zone,
            vis_mtr=params.vis_mtr,
            vis_mti=params.vis_mti,
            vis_thp=params.vis_thp,
            nir_mtr=params.nir_mtr,
            nir_mti=params.nir_mti,
            nir_thp=params.nir_thp,
            captures=params.captures,
            capture_delay=params.capture_delay,
        )

    if params.plot and params.mode != 'focus':
        manager.get_logger().info("Tanca manualment la finestra del gràfic per finalitzar el programa.")
        plt.ioff()
        plt.show()

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

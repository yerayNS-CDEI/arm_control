#!/usr/bin/env python3

##################################################
#### ROS2 HYPERSPECTRAL SENSOR NODE
##################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from arm_control.srv import HyperspectralCommand, HyperspectralConfig, PredictMaterial
from sensors_lib.lenz_client import LenzClient
import threading
import time
import sys
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import csv
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

class HyperspectralNode(Node):

    def __init__(self):
        super().__init__("hyperspectral_node")

        # Parameters
        self.declare_parameter("vis_ip", "193.167.0.30")
        self.declare_parameter("nir_ip", "193.167.0.31")
        self.declare_parameter("vis_mtr", 60000)
        self.declare_parameter("vis_mti", 60000)
        self.declare_parameter("nir_mtr", 20000)
        self.declare_parameter("nir_mti", 20000)
        self.declare_parameter("auto_configure", True)
        self.declare_parameter("enable_plotting", False)
        self.declare_parameter("enable_csv_logging", False)
        
        # Default CSV directory: package's resource folder
        try:
            package_name = 'arm_control'
            package_dir = get_package_share_directory(package_name)
            default_csv_dir = os.path.join(package_dir, '..', '..', '..', '..', 'src', package_name, 'resource')
            default_csv_dir = os.path.abspath(default_csv_dir)
        except:
            # Fallback to home directory if package not found
            default_csv_dir = os.path.expanduser("~")
        
        self.declare_parameter("csv_directory", default_csv_dir)

        vis_ip = self.get_parameter("vis_ip").value
        nir_ip = self.get_parameter("nir_ip").value
        self.vis_mtr = self.get_parameter("vis_mtr").value
        self.vis_mti = self.get_parameter("vis_mti").value
        self.nir_mtr = self.get_parameter("nir_mtr").value
        self.nir_mti = self.get_parameter("nir_mti").value
        auto_configure = self.get_parameter("auto_configure").value
        self.enable_plotting = self.get_parameter("enable_plotting").value
        self.enable_csv_logging = self.get_parameter("enable_csv_logging").value
        csv_directory = self.get_parameter("csv_directory").value

        # Publishers (optional - publish after successful measurements)
        self.vis_pub = self.create_publisher(Float32MultiArray, "/hyperspectral/vis_spectrum", 10)
        self.nir_pub = self.create_publisher(Float32MultiArray, "/hyperspectral/nir_spectrum", 10)

        # Wavelength arrays (256 points each)
        self.vis_wavelengths = np.linspace(325.3, 792.6, 256)  # VIS: 325.3-792.6 nm
        self.nir_wavelengths = np.linspace(991, 1707, 256)     # NIR: 991-1707 nm
        
        # Variables per emmagatzemar les calibracions
        self.calibration_data = {
            "GDS": {"vis": None, "nir": None, "timestamp": None},
            "GDR": {"vis": None, "nir": None, "timestamp": None}, 
            "GRF": {"vis": None, "nir": None, "timestamp": None}
        }

        # Initialize CSV logging
        self.csv_file_path = None
        if self.enable_csv_logging:
            self._initialize_csv_file(csv_directory)

        # Initialize plotting if enabled
        self.fig = None
        self.ax_vis = None
        self.ax_nir = None
        
        if self.enable_plotting:
            self._setup_matplotlib_backend()
            if self.enable_plotting:  # Could be disabled if no backend found
                self.fig, (self.ax_vis, self.ax_nir) = plt.subplots(2, 1, figsize=(10, 8))
                self.fig.suptitle('Hyperspectral Measurements', fontsize=14)
                self.ax_vis.set_title('VIS Spectrum')
                self.ax_vis.set_xlabel('Wavelength (nm)')
                self.ax_vis.set_ylabel('Intensity')
                self.ax_vis.grid(True)
                self.ax_nir.set_title('NIR Spectrum')
                self.ax_nir.set_xlabel('Wavelength (nm)')
                self.ax_nir.set_ylabel('Intensity')
                self.ax_nir.grid(True)
                plt.ion()  # Enable interactive mode
                plt.show(block=False)
                self.get_logger().info("✓ Plotting enabled")

        # Services
        self.hardware_cb_group = MutuallyExclusiveCallbackGroup()

        self.configure_srv = self.create_service(
            HyperspectralConfig, 
            "hyperspectral/configure", 
            self.configure_callback,
            callback_group=self.hardware_cb_group
        )
        self.measurement_srv = self.create_service(
            HyperspectralCommand, 
            "hyperspectral/measurement", 
            self.measurement_callback,
            callback_group=self.hardware_cb_group
        )
        
        # ML Prediction service
        self.ml_srv = self.create_service(
            PredictMaterial,
            "hyperspectral/predict_material",
            self.predict_material_callback
        )      

        # Connect sensors (LenzClient handles keepalive automatically)
        self.get_logger().info("Connecting to VIS and NIR sensors...")
        self.vis = LenzClient(vis_ip)
        self.nir = LenzClient(nir_ip)

        try:
            self.vis.connect()
            self.nir.connect()
            self.get_logger().info("✓ VIS & NIR connected. PNG keepalive running automatically.")
            
            # Auto-configure if enabled
            # Auto-configure if enabled
            if auto_configure:
                time.sleep(2)  # Stabilization
                self.get_logger().info("Auto-configuring sensors...")
                self._configure_sensors(self.vis_mtr, self.vis_mti, self.nir_mtr, self.nir_mti)
                self.get_logger().info("Running mandatory calibration sequence...")
                self._calibrate_sensors() # <-- NOVA LÍNIA
                self.get_logger().info("✓ Sensors configured, calibrated and ready.")
        except Exception as e:
            self.get_logger().error(f"✗ Connection failed: {e}")
            raise

        self.get_logger().info("HyperspectralNode ready with ML prediction service.")


    # ----------------------------------------------------------
    # Helper: Setup matplotlib backend
    # ----------------------------------------------------------
    def _setup_matplotlib_backend(self):
        """Setup the best available matplotlib backend for Ubuntu."""
        # Check if we have a display (GUI environment)
        has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        
        if not has_display:
            # No GUI - use non-interactive backend
            matplotlib.use('Agg')
            self.enable_plotting = False
            self.get_logger().info("No GUI detected - plotting disabled (using Agg backend)")
            return
        
        # Try interactive backends in order of preference for Ubuntu
        backends = ['TkAgg', 'Qt5Agg', 'GTK3Agg', 'Agg']
        
        for backend in backends:
            try:
                matplotlib.use(backend)
                # Test if backend actually works
                import matplotlib.pyplot as plt
                fig = plt.figure()
                plt.close(fig)
                
                if backend == 'Agg':
                    # Agg means no interactive plotting
                    self.enable_plotting = False
                    self.get_logger().info(f"Using {backend} backend - plotting disabled (non-interactive)")
                else:
                    self.get_logger().info(f"✓ Using {backend} backend for interactive plotting")
                break
                
            except (ImportError, RuntimeError) as e:
                self.get_logger().debug(f"Backend {backend} not available: {e}")
                continue
        else:
            # No backend worked
            matplotlib.use('Agg')
            self.enable_plotting = False
            self.get_logger().warning("No interactive matplotlib backend available - plotting disabled")

    # ----------------------------------------------------------
    # Helper: Configure sensors
    # ----------------------------------------------------------
    def _configure_sensors(self, vis_mtr, vis_mti, nir_mtr, nir_mti):
        """Internal configuration helper."""
        self.vis.send_config("MTR", vis_mtr); time.sleep(0.2)
        self.vis.send_config("MTI", vis_mti); time.sleep(0.2)
        self.vis.send_config("THP", 1);       time.sleep(0.2)
        
        self.nir.send_config("MTR", nir_mtr); time.sleep(0.2)
        self.nir.send_config("MTI", nir_mti); time.sleep(0.2)
        self.nir.send_config("THP", 1);       time.sleep(0.2)
        
        time.sleep(3)  # Stabilization

    # ----------------------------------------------------------
    # Helper: Calibrate sensors (Mandatory Sequence)
    # ----------------------------------------------------------
    # ----------------------------------------------------------
    # Helper: Calibrate sensors (Mandatory Sequence)
    # ----------------------------------------------------------
    def _calibrate_sensors(self):
        """Send the mandatory GDS -> GDR -> GRF sequence before any GSM."""
        sequence = [
            ("GDS", "Dark Sample (POSA LA TAPA A LA CÀMERA)"), 
            ("GDR", "Dark Reference (MANTÉN LA TAPA POSADA)"), 
            ("GRF", "White Reference (TREU LA TAPA I POSA EL PAPER BLANC)")
        ]
        
        for cmd, label in sequence:
            # EL TRUC: El programa s'atura aquí i espera que premis ENTER a la Terminal 1
            input(f"\n>>> PREPARACIÓ: {label} - Prem ENTER quan estiguis llesta...")
            
            self.get_logger().info(f"Sending mandatory calibration command: {cmd}...")
            
            # Neteja de buffers i espera com a l'exemple
            self.vis.flush_buffer()
            self.nir.flush_buffer()
            time.sleep(0.1)
            self.vis.wait_after_ping(0.5)
            self.nir.wait_after_ping(0.5)
            
            # Enviar la comanda de calibració
            self.vis.send_simple(cmd)
            self.nir.send_simple(cmd)
            
            # Temps d'espera i lectura de resultats
            time.sleep(1.2)
            
            results = {}
            def read_sensor(client, sensor_label):
                try:
                    results[sensor_label] = client.read_frame([cmd])
                except Exception as e:
                    results[sensor_label] = f"ERROR: {e}"

            t_vis = threading.Thread(target=read_sensor, args=(self.vis, "VIS"))
            t_nir = threading.Thread(target=read_sensor, args=(self.nir, "NIR"))
            t_vis.start()
            t_nir.start()
            t_vis.join()
            t_nir.join()
            
            # GUARDEM LES DADES DE CALIBRACIÓ!
            if (isinstance(results["VIS"], dict) and "spectrum" in results["VIS"] and 
                isinstance(results["NIR"], dict) and "spectrum" in results["NIR"]):
                
                self.calibration_data[cmd] = {
                    "vis": list(results["VIS"]["spectrum"]),
                    "nir": list(results["NIR"]["spectrum"]),
                    "timestamp": datetime.now().isoformat()
                }
                self.get_logger().info(f"✓ Calibration {cmd} guardada en memòria.")
            else:
                self.get_logger().error(f"✗ Error llegint dades de calibració {cmd}")
            
            time.sleep(0.1)
            self.vis.flush_buffer()
            self.nir.flush_buffer()
            
            self.get_logger().info(f"✓ Calibration {cmd} complete.")
            time.sleep(1.5) # Espera curta entre comandes
            
    # ----------------------------------------------------------
    # SERVICE: Configuration
    # ----------------------------------------------------------
    def configure_callback(self, request, response):
        """
        Configure sensor parameters.
        Request:
            sensor: "VIS" or "NIR"
            command: "MTR", "MTI", or "THP"
            value: int (e.g., 60000 for MTI)
        Response:
            success: bool
            message: str
        """
        sensor_name = request.sensor.upper()
        cmd = request.command.upper()
        value = request.value

        self.get_logger().info(f"Config request: {sensor_name} {cmd}={value}")

        client = self.vis if sensor_name == "VIS" else self.nir

        try:
            client.send_config(cmd, value)
            time.sleep(0.2)
            
            response.success = True
            response.message = f"OK: {sensor_name} {cmd}={value}"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"ERROR: {str(e)}"
            self.get_logger().error(response.message)

        return response


    # ----------------------------------------------------------
    # SERVICE: Measurement
    # ----------------------------------------------------------
    def measurement_callback(self, request, response):
        """
        Execute measurement command (GDS, GDR, GRF, GSM) with automatic retry.
        Request:
            command: str ("GDS", "GDR", "GRF", "GSM")
        Response:
            vis_ok: bool
            vis_spectrum: uint16[]
            vis_status: int32
            nir_ok: bool
            nir_spectrum: uint16[]
            nir_status: int32
            message: str
        """
        cmd = request.command.upper()
        x_coord = request.x_coord
        y_coord = request.y_coord
        z_coord = request.z_coord
        self.get_logger().info(f"Measurement request: {cmd} at ({x_coord}, {y_coord}, {z_coord})")
        
        # Comprovar si és una petició de calibració guardada
        if cmd.startswith("GET_"):
            calib_type = cmd[4:]  # Treu "GET_" -> GDS, GDR, GRF
            return self._get_stored_calibration(calib_type, response)

        # Try with automatic retry (up to 3 attempts)
        for attempt in range(3):
            try:
                # Clear buffers
                self.vis.flush_buffer()
                self.nir.flush_buffer()
                time.sleep(0.1)

                # Wait for safe timing window after PNG
                self.vis.wait_after_ping(0.5)
                self.nir.wait_after_ping(0.5)

                # Send commands
                self.get_logger().info(f"[Attempt {attempt+1}/3] Sending {cmd}...")
                self.vis.send_simple(cmd)
                self.nir.send_simple(cmd)
                break  # Success

            except (BrokenPipeError, ConnectionError, OSError) as e:
                self.get_logger().warning(f"[Attempt {attempt+1}/3] FAILED: {e}")
                
                if attempt < 2:  # Retry
                    self.get_logger().info("Reconnecting...")
                    try:
                        self.vis.close()
                        self.nir.close()
                    except:
                        pass
                    
                    time.sleep(2)
                    self.vis = LenzClient(self.get_parameter("vis_ip").value)
                    self.nir = LenzClient(self.get_parameter("nir_ip").value)
                    self.vis.connect()
                    self.nir.connect()
                    time.sleep(3)

                    # Reconfigure
                    self._configure_sensors(
                        self.vis_mtr, self.vis_mti,
                        self.nir_mtr, self.nir_mti
                    )
                    time.sleep(2)
                else:
                    # All attempts failed
                    response.vis_ok = False
                    response.nir_ok = False
                    response.message = f"Failed after 3 attempts: {e}"
                    return response

        # Wait for sensor to complete measurement
        time.sleep(1.2)

        # Read results in parallel threads (com en la calibració que funciona)
        results = {}
        def read_sensor(client, label):
            try:
                self.get_logger().info(f"Reading {label} sensor response...")
                results[label] = client.read_frame([cmd])
                self.get_logger().info(f"{label} response type: {type(results[label])}, keys: {list(results[label].keys()) if isinstance(results[label], dict) else 'Not a dict'}")
            except Exception as e:
                results[label] = f"ERROR: {e}"
                self.get_logger().error(f"{label} read_frame failed: {e}")

        t_vis = threading.Thread(target=read_sensor, args=(self.vis, "VIS"))
        t_nir = threading.Thread(target=read_sensor, args=(self.nir, "NIR"))

        t_vis.start()
        t_nir.start()
        t_vis.join()
        t_nir.join()

        # Clean up remaining bytes
        time.sleep(0.1)
        self.vis.flush_buffer()
        self.nir.flush_buffer()

        # Process VIS results
        if isinstance(results["VIS"], dict) and "spectrum" in results["VIS"]:
            response.vis_ok = True
            response.vis_spectrum = list(results["VIS"]["spectrum"])
            response.vis_status = results["VIS"].get("status", 0)  # Agafa l'status si hi és, sinó 0
            
            # Publish VIS spectrum
            vis_msg = Float32MultiArray()
            vis_msg.data = [float(x) for x in response.vis_spectrum]
            self.vis_pub.publish(vis_msg)
        else:
            response.vis_ok = False
            response.vis_spectrum = []
            response.vis_status = -1
            if isinstance(results["VIS"], str):
                self.get_logger().warning(f"VIS error: {results['VIS']}")

        # Process NIR results
        if isinstance(results["NIR"], dict) and "spectrum" in results["NIR"]:
            response.nir_ok = True
            response.nir_spectrum = list(results["NIR"]["spectrum"])
            response.nir_status = results["NIR"].get("status", 0)  # Agafa l'status si hi és, sinó 0
            
            # Publish NIR spectrum
            nir_msg = Float32MultiArray()
            nir_msg.data = [float(x) for x in response.nir_spectrum]
            self.nir_pub.publish(nir_msg)
        else:
            response.nir_ok = False
            response.nir_spectrum = []
            response.nir_status = -1
            if isinstance(results["NIR"], str):
                self.get_logger().warning(f"NIR error: {results['NIR']}")

        # Guardar dades de calibració si és una comanda de calibració
        if cmd in ["GDS", "GDR", "GRF"] and response.vis_ok and response.nir_ok:
            self.calibration_data[cmd] = {
                "vis": response.vis_spectrum,
                "nir": response.nir_spectrum,
                "timestamp": datetime.now().isoformat()
            }
            self.get_logger().info(f"✓ Calibration {cmd} actualitzada en memòria.")

        # Set response message
        if response.vis_ok and response.nir_ok:
            vis_max_idx = np.argmax(response.vis_spectrum)
            nir_max_idx = np.argmax(response.nir_spectrum)
            vis_max_wl = self.vis_wavelengths[vis_max_idx]
            nir_max_wl = self.nir_wavelengths[nir_max_idx]
            
            # Formatar el missatge de resposta
            response.message = f"OK: {cmd} completat. Pic VIS: {vis_max_wl:.1f} nm | Pic NIR: {nir_max_wl:.1f} nm"
            self.get_logger().info(response.message)

        else:
            response.message = f"PARTIAL: VIS={response.vis_ok}, NIR={response.nir_ok}"
            self.get_logger().warning(response.message)

        # Update plots if enabled
        if self.enable_plotting and response.vis_ok and response.nir_ok:
            self._update_plots(response.vis_spectrum, response.nir_spectrum, cmd)

        # Log to CSV if enabled
        csv_path = ""
        if self.enable_csv_logging and response.vis_ok and response.nir_ok:
            csv_path = self._log_to_csv(cmd, x_coord, y_coord, z_coord, response.vis_spectrum, response.nir_spectrum)

        if response.vis_ok and response.nir_ok:
            response.message = f"OK|{csv_path}"
        else:
            response.message = f"ERROR|PARTIAL: VIS={response.vis_ok}, NIR={response.nir_ok}"

        return response

    # ----------------------------------------------------------
    # SERVICE: Material Prediction
    # ----------------------------------------------------------
    def predict_material_callback(self, request, response):
        """
        Predict material based on VIS and NIR spectra.
        Request:
            vis_spectrum: float32[]
            nir_spectrum: float32[]
        Response:
            material: str
            confidence: float32
        """
        try:
            vis_spectrum = np.array(request.vis_spectrum)
            nir_spectrum = np.array(request.nir_spectrum)
            
            self.get_logger().info(f"ML prediction request: VIS({len(vis_spectrum)}), NIR({len(nir_spectrum)})")
            
            # Validació de les dades d'entrada
            if len(vis_spectrum) != 256 or len(nir_spectrum) != 256:
                response.material = "ERROR"
                response.confidence = 0.0
                self.get_logger().error(f"Invalid spectrum length: VIS={len(vis_spectrum)}, NIR={len(nir_spectrum)} (expected 256 each)")
                return response
            
            # Implementació bàsica de classificació per materials
            # Aquesta és una implementació simplificada que es pot millorar amb models ML reals
            material, confidence = self._classify_material(vis_spectrum, nir_spectrum)
            
            response.material = material
            response.confidence = float(confidence)
            
            self.get_logger().info(f"ML prediction result: {material} (confidence: {confidence:.3f})")
            
        except Exception as e:
            self.get_logger().error(f"ML prediction failed: {e}")
            response.material = "ERROR"
            response.confidence = 0.0
        
        return response

    # ----------------------------------------------------------
    # Helper: Material Classification
    # ----------------------------------------------------------
    def _classify_material(self, vis_spectrum, nir_spectrum):
        """
        Classificació bàsica de materials basada en característiques espectrals.
        Aquesta implementació es pot substituir per models ML més sofisticats.
        """
        try:
            # Normalització dels espectres
            vis_norm = vis_spectrum / np.max(vis_spectrum) if np.max(vis_spectrum) > 0 else vis_spectrum
            nir_norm = nir_spectrum / np.max(nir_spectrum) if np.max(nir_spectrum) > 0 else nir_spectrum
            
            # Càlcul de característiques espectrals
            vis_peak_idx = np.argmax(vis_norm)
            nir_peak_idx = np.argmax(nir_norm)
            vis_peak_wl = self.vis_wavelengths[vis_peak_idx]
            nir_peak_wl = self.nir_wavelengths[nir_peak_idx]
            
            # Càlcul d'índexs espectrals
            red_idx = np.argmin(np.abs(self.vis_wavelengths - 680))  # Vermell
            green_idx = np.argmin(np.abs(self.vis_wavelengths - 550))  # Verd
            nir_idx = np.argmin(np.abs(self.nir_wavelengths - 800))   # NIR inicial
            
            # Relacions espectrals
            if len(vis_norm) > red_idx and len(vis_norm) > green_idx:
                red_green_ratio = vis_norm[red_idx] / (vis_norm[green_idx] + 1e-6)
            else:
                red_green_ratio = 1.0
                
            if len(nir_norm) > 50:
                nir_vis_ratio = np.mean(nir_norm[:50]) / (np.mean(vis_norm[-50:]) + 1e-6)
            else:
                nir_vis_ratio = 1.0
            
            # Classificació basada en regles heurístiques
            # (Aquesta lògica es pot substituir per models ML entrenats)
            
            if vis_peak_wl > 600 and red_green_ratio > 1.5:
                # Materials vermellosos o amb alta reflectància en vermell
                if nir_vis_ratio > 2.0:
                    material = "Vegetació" if vis_peak_wl < 700 else "Material orgànic"
                    confidence = min(0.85, red_green_ratio / 3.0)
                else:
                    material = "Plàstic vermell" if vis_peak_wl > 650 else "Ceràmica"
                    confidence = min(0.75, red_green_ratio / 2.0)
            
            elif vis_peak_wl < 500 and nir_vis_ratio < 0.8:
                # Materials blavosos o amb baixa reflectància NIR
                material = "Metall" if np.std(nir_norm) < 0.1 else "Plàstic blau"
                confidence = 0.70
            
            elif 500 <= vis_peak_wl <= 600 and nir_vis_ratio > 1.5:
                # Materials verdosos amb bona reflectància NIR
                material = "Vegetació verda" if red_green_ratio < 1.2 else "Teixit verd"
                confidence = 0.80
            
            elif nir_peak_wl > 1300 and np.mean(nir_norm) > 0.6:
                # Alta reflectància en NIR llunyà
                material = "Paper" if vis_peak_wl > 500 else "Teixit blanc"
                confidence = 0.75
            
            else:
                # Classificació genèrica per materials no identificats
                if np.mean(vis_norm) > 0.5:
                    material = "Material clar"
                elif np.mean(vis_norm) < 0.2:
                    material = "Material fosc"
                else:
                    material = "Material mixt"
                confidence = 0.50
            
            # Assegurar que la confiança estigui en el rang [0, 1]
            confidence = max(0.0, min(1.0, confidence))
            
            return material, confidence
            
        except Exception as e:
            self.get_logger().error(f"Classification error: {e}")
            return "ERROR", 0.0

    # ----------------------------------------------------------
    # Helper: Get stored calibration data
    # ----------------------------------------------------------
    def _get_stored_calibration(self, calib_type, response):
        """Retorna les dades de calibració emmagatzemades."""
        if calib_type in self.calibration_data:
            calib_data = self.calibration_data[calib_type]
            if calib_data["vis"] is not None and calib_data["nir"] is not None:
                response.vis_ok = True
                response.nir_ok = True
                response.vis_spectrum = calib_data["vis"]
                response.nir_spectrum = calib_data["nir"]
                response.message = f"OK: Calibration {calib_type} retrieved from memory"
                self.get_logger().info(response.message)
            else:
                response.vis_ok = False
                response.nir_ok = False
                response.message = f"ERROR: No {calib_type} calibration data available"
                self.get_logger().error(response.message)
        else:
            response.vis_ok = False
            response.nir_ok = False
            response.message = f"ERROR: Unknown calibration type {calib_type}"
            self.get_logger().error(response.message)
        
        return response

    # ----------------------------------------------------------
    # Helper: Initialize CSV file
    # ----------------------------------------------------------
    def _initialize_csv_file(self, directory):
        """Create or open CSV file with date-based filename."""
        try:
            # Create filename with current date: Hyperspectral_data_DD_MM_YYYY.csv
            now = datetime.now()
            filename = f"Hyperspectral_data_{now.day:02d}_{now.month:02d}_{now.year}.csv"
            self.csv_file_path = os.path.join(directory, filename)
            
            # Check if file exists to determine if we need to write header
            file_exists = os.path.isfile(self.csv_file_path)
            
            if not file_exists:
                # Create new file with header
                with open(self.csv_file_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    
                    # Build header: Daily_Count, Measure Type, Date, Time, X, Y, Z, Material, Confidence, VIS wavelengths, NIR wavelengths
                    header = ['Daily_Count', 'Measure_Type', 'Date', 'Time', 'X_Coordinate', 'Y_Coordinate', 'Z_Coordinate', 'Material', 'Confidence']
                    
                    # Add VIS wavelength columns
                    for wl in self.vis_wavelengths:
                        header.append(f'{wl:.1f}')
                    
                    # Add NIR wavelength columns
                    for wl in self.nir_wavelengths:
                        header.append(f'{wl:.1f}')
                    
                    writer.writerow(header)
                
                self.get_logger().info(f"✓ CSV file created: {self.csv_file_path}")
            else:
                self.get_logger().info(f"✓ CSV file exists, will append data: {self.csv_file_path}")
                
        except Exception as e:
            self.get_logger().error(f"✗ Failed to initialize CSV file: {e}")
            self.csv_file_path = None


    # ----------------------------------------------------------
    # Helper: Log measurement to CSV
    # ----------------------------------------------------------
    def _log_to_csv(self, cmd, x, y, z, vis_spectrum, nir_spectrum):
        """Append measurement data to CSV file."""
        if self.csv_file_path is None:
            return
            
        try:
            now = datetime.now()
            date_str = now.strftime("%Y-%m-%d")
            time_str = now.strftime("%H:%M:%S.%f")[:-3]  # milliseconds precision
            
            with open(self.csv_file_path, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                
                # Build row: daily_count, cmd, date, time, x, y, z, Material, Confidence, vis_data..., nir_data...
                # Note: Material and Confidence are empty here since this node doesn't do ML inference
                # Daily_Count will be handled by inspection_manager
                row = [0, cmd, date_str, time_str, x, y, z, "", ""]
                row.extend(vis_spectrum)
                row.extend(nir_spectrum)
                
                writer.writerow(row)
            
            self.get_logger().info(f"✓ Data logged to CSV: {cmd} at ({x}, {y}, {z})")
            return self.csv_file_path
            
        except Exception as e:
            self.get_logger().error(f"✗ Failed to log to CSV: {e}")


    # ----------------------------------------------------------
    # Helper: Update plots
    # ----------------------------------------------------------
    def _update_plots(self, vis_spectrum, nir_spectrum, cmd):
        """Update matplotlib plots with new spectrum data."""
        try:
            # Clear previous plots
            self.ax_vis.clear()
            self.ax_nir.clear()

            # Plot VIS with wavelength axis (325.3-792.6 nm)
            self.ax_vis.plot(self.vis_wavelengths, vis_spectrum, 'b-', linewidth=1.5, label=f'{cmd}')
            self.ax_vis.set_title(f'VIS Spectrum - {cmd}')
            self.ax_vis.set_xlabel('Wavelength (nm)')
            self.ax_vis.set_ylabel('Intensity')
            self.ax_vis.set_xlim(325.3, 792.6)
            self.ax_vis.grid(True, alpha=0.3)
            self.ax_vis.legend()

            # Plot NIR with wavelength axis (991-1707 nm)
            self.ax_nir.plot(self.nir_wavelengths, nir_spectrum, 'r-', linewidth=1.5, label=f'{cmd}')
            self.ax_nir.set_title(f'NIR Spectrum - {cmd}')
            self.ax_nir.set_xlabel('Wavelength (nm)')
            self.ax_nir.set_ylabel('Intensity')
            self.ax_nir.set_xlim(991, 1707)
            self.ax_nir.grid(True, alpha=0.3)
            self.ax_nir.legend()

            # Update display
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().warning(f"Plot update failed: {e}")


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

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

from arm_control.srv import HyperspectralCommand, HyperspectralConfig, PredictMaterial

# Longitud esperada dels espectres (ha de coincidir amb hyperspectral_node.py)
SPECTRUM_LENGTH = 256


class InspectionManager(Node):
    def __init__(self):
        super().__init__('inspection_manager')
        self.camera_client = self.create_client(HyperspectralCommand, 'hyperspectral/measurement')
        self.config_client  = self.create_client(HyperspectralConfig,  'hyperspectral/configure')
        self.ml_client      = self.create_client(PredictMaterial,       'hyperspectral/predict_material')

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
    # Inspeccio principal
    # ----------------------------------------------------------
    def run_inspection(self, x, y, z,
                       mode='predict', label='Unknown',
                       vis_mtr=None, vis_mti=None,
                       nir_mtr=None, nir_mti=None,
                       vis_thp=None, nir_thp=None):

        self.get_logger().info("1. Esperant que els serveis estiguin actius...")
        self.camera_client.wait_for_service()
        self.config_client.wait_for_service()
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
                    self.get_logger().warn(
                        f"  Avertencia: {sensor} {cmd}={val} ha fallat. "
                        "Es continua amb la configuracio actual del node."
                    )
        else:
            self.get_logger().info("2. Cap configuracio de sensors especificada. S'usa la del node.")

        # --- Recuperar calibracions ---
        self.get_logger().info("3. Recuperant calibracions (GDS, GDR, GRF) del node...")
        res_gds = self.get_calibration_data("GDS")
        res_gdr = self.get_calibration_data("GDR")
        res_grf = self.get_calibration_data("GRF")

        if res_grf is None or not res_grf.vis_ok:
            self.get_logger().error(
                "Error: Falten les dades del White Reference (GRF). "
                "Executa la sequencia de calibracio al hyperspectral_node primer."
            )
            return

        # --- Mesura GSM ---
        self.get_logger().info(f"4. Cridant la camera hiperespectral a ({x}, {y}, {z})...")
        req_cam = HyperspectralCommand.Request()
        req_cam.command = "GSM"
        req_cam.x_coord = float(x)
        req_cam.y_coord = float(y)
        req_cam.z_coord = float(z)

        future_cam = self.camera_client.call_async(req_cam)
        rclpy.spin_until_future_complete(self, future_cam)
        res_cam = future_cam.result()

        if res_cam is None:
            self.get_logger().error("Error: Cap resposta de la camera (timeout o servei caigut).")
            return

        # Parsejar el nou format de missatge "OK|detall" o "ERROR|detall"
        cam_ok, cam_detail = self._parse_response_message(res_cam.message)

        if not res_cam.vis_ok or not res_cam.nir_ok:
            self.get_logger().error(
                f"Error: La camera no ha pogut prendre les dades. "
                f"VIS={'OK' if res_cam.vis_ok else f'FAIL (status={res_cam.vis_status})'}, "
                f"NIR={'OK' if res_cam.nir_ok else f'FAIL (status={res_cam.nir_status})'} | "
                f"{cam_detail}"
            )
            return

        self.get_logger().info(f"  Mesura OK: {cam_detail}")

        # --- Validacio de longituds (el node rebutja espectres != 256) ---
        for spectrum, spectrum_label in [
            (res_cam.vis_spectrum, "GSM VIS"),
            (res_cam.nir_spectrum, "GSM NIR"),
            (res_grf.vis_spectrum, "GRF VIS"),
            (res_grf.nir_spectrum, "GRF NIR"),
        ]:
            if not self._validate_spectrum(spectrum, spectrum_label):
                return

        # --- Normalitzacio ---
        self.get_logger().info("5. Normalitzant l'espectre (Reflectancia: (GSM-GDS) / (GRF-GDS))...")
        
        # 1. Passem a Numpy les dades de la Mostra (GSM), del Blanc (GRF) i del Soroll (GDS)
        vis_gsm = np.array(res_cam.vis_spectrum, dtype=float)
        vis_grf = np.array(res_grf.vis_spectrum, dtype=float)
        vis_gds = np.array(res_gds.vis_spectrum, dtype=float)
        nir_gsm = np.array(res_cam.nir_spectrum, dtype=float)
        nir_grf = np.array(res_grf.nir_spectrum, dtype=float)
        nir_gds = np.array(res_gds.nir_spectrum, dtype=float)

        # 2. Fem la "Tara": Restem el soroll a la mostra i al blanc
        vis_gsm_net = vis_gsm - vis_gds
        vis_grf_net = vis_grf - vis_gds
        
        nir_gsm_net = nir_gsm - nir_gds
        nir_grf_net = nir_grf - nir_gds

        # 3. Dividim les dades netes. (Usem > 0 al divisor per evitar errors matemàtics)
        vis_norm = np.divide(vis_gsm_net, vis_grf_net, out=np.zeros(SPECTRUM_LENGTH), where=vis_grf_net > 0)
        vis_norm = np.maximum(vis_norm, 0)  # Assegurem que no hi ha valors negatius
        
        nir_norm = np.divide(nir_gsm_net, nir_grf_net, out=np.zeros(SPECTRUM_LENGTH), where=nir_grf_net > 0)
        nir_norm = np.maximum(nir_norm, 0)  # Assegurem que no hi ha valors negatius

        # Doble validacio: la normalitzacio no ha de canviar la longitud
        if not self._validate_spectrum(vis_norm, "VIS normalitzat") or \
           not self._validate_spectrum(nir_norm, "NIR normalitzat"):
            return

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

            # ----------------------------------------------------------
            # A) CSV DE PREDICCIONS (EL DIARI)
            # ----------------------------------------------------------
            self.get_logger().info("7. Generant el CSV i l'estructura JSON...")
            now = datetime.now()
            filename = f"Hyperspectral_data_{now.day:02d}_{now.month:02d}_{now.year}.csv"
            csv_dir = os.path.expanduser("~/Documents/Hyperspectral_camera/src/arm_control/resource")
            os.makedirs(csv_dir, exist_ok=True)
            csv_path = os.path.join(csv_dir, filename)

            file_exists = os.path.isfile(csv_path)
            daily_count = 1
            row_index   = 1

            if file_exists:
                with open(csv_path, 'r') as f:
                    reader = csv.reader(f)
                    lines  = list(reader)
                    row_index   = len(lines)
                    daily_count = sum(1 for row in lines if len(row) > 1 and row[1] == "GSM") + 1

            with open(csv_path, 'a', newline='') as csvfile:
                writer   = csv.writer(csvfile)
                date_str = now.strftime("%Y-%m-%d")
                time_str = now.strftime("%H:%M:%S.%f")[:-3]

                if not file_exists:
                    vis_wavelengths = np.linspace(325.3, 792.6, SPECTRUM_LENGTH)
                    nir_wavelengths = np.linspace(991,   1707,  SPECTRUM_LENGTH)
                    header = ['Daily_Count', 'Measure_Type', 'Date', 'Time', 'X_Coordinate', 'Y_Coordinate', 'Z_Coordinate', 'Material', 'Confidence']
                    header.extend([f'{wl:.1f}' for wl in vis_wavelengths])
                    header.extend([f'{wl:.1f}' for wl in nir_wavelengths])
                    writer.writerow(header)

                data_row = [daily_count, "GSM", date_str, time_str, x, y, z, res_ml.material, res_ml.confidence]
                data_row.extend(vis_norm.tolist())
                data_row.extend(nir_norm.tolist())
                writer.writerow(data_row)

            # ----------------------------------------------------------
            # B) JSON DE PREDICCIONS
            # ----------------------------------------------------------
            temps_actual = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
            nou_registre = {
                "id": daily_count,
                "timestamp": temps_actual,
                "point_coordinates": {
                    "x": {"value": float(x), "error": 0.0, "unit": "m"},
                    "y": {"value": float(y), "error": 0.0, "unit": "m"},
                    "z": {"value": float(z), "error": 0.0, "unit": "m"},
                },
                "row": row_index,
                "material": res_ml.material,
                "confidence": float(res_ml.confidence),
            }

            json_dir       = os.path.expanduser("~/hyperspectral_inspections")
            os.makedirs(json_dir, exist_ok=True)
            nom_arxiu_json = os.path.join(json_dir, "registre_inspeccions.json")

            dades_historial = {"hyperspectral_camera": {"data_pointer_csv": csv_path, "measurements": []}}

            if os.path.exists(nom_arxiu_json):
                try:
                    with open(nom_arxiu_json, 'r') as f:
                        historial_antic = json.load(f)
                    if "hyperspectral_camera" in historial_antic:
                        if isinstance(historial_antic["hyperspectral_camera"], list):
                            dades_historial["hyperspectral_camera"]["measurements"] = historial_antic["hyperspectral_camera"]
                        elif isinstance(historial_antic["hyperspectral_camera"], dict):
                            dades_historial = historial_antic
                except json.JSONDecodeError:
                    pass

            dades_historial["hyperspectral_camera"]["data_pointer_csv"] = csv_path
            dades_historial["hyperspectral_camera"]["measurements"].append(nou_registre)

            with open(nom_arxiu_json, 'w') as f:
                json.dump(dades_historial, f, indent=4)

            self.get_logger().info(f"8. Predicció #{daily_count} guardada correctament!")

        elif mode == 'collect':
            # ==========================================================
            # MODE CAPTURA (NOMÉS PER CREAR DATASET D'ENTRENAMENT)
            # ==========================================================
            self.get_logger().info(f"6. [MODE COLLECT] Guardant espectre d'entrenament per a la classe: '{label}'...")
            
            now = datetime.now()
            # Guardem tot en un arxiu específic per entrenar, independent de les inspeccions diàries
            csv_dir = os.path.expanduser("~/Documents/Hyperspectral_camera/src/arm_control/resource")
            os.makedirs(csv_dir, exist_ok=True)
            dataset_path = os.path.join(csv_dir, "Dataset_Entrenament_Nou.csv")
            
            file_exists = os.path.isfile(dataset_path)
            
            with open(dataset_path, 'a', newline='') as csvfile:
                # Fem servir el mateix format que el teu arxiu 'Dades.csv' original
                writer = csv.writer(csvfile, delimiter=';')
                date_str = now.strftime("%Y-%m-%d")
                time_str = now.strftime("%H:%M:%S.%f")[:-3]

                if not file_exists:
                    vis_wavelengths = np.linspace(325.3, 792.6, SPECTRUM_LENGTH)
                    nir_wavelengths = np.linspace(991,   1707,  SPECTRUM_LENGTH)
                    header = ['Measure Type', 'Date', 'Time', 'Counter', 'Label', 'Class']
                    # Guardem els decimals amb comes perquè quadri amb el teu codi d'entrenament
                    header.extend([f'{wl:.4f}'.replace('.', ',') for wl in vis_wavelengths])
                    header.extend([f'{wl:.0f}' for wl in nir_wavelengths])
                    writer.writerow(header)

                # Format de la fila: Measure Type, Date, Time, Counter (0), Label, Class, ...espectres
                data_row = ["GSM", date_str, time_str, 0, label, label]
                
                # Transformem els punts a comes perquè el teu train_model.py els llegeixi bé com abans
                espectre_complet = vis_norm.tolist() + nir_norm.tolist()
                espectre_strings = [str(val).replace('.', ',') for val in espectre_complet]
                data_row.extend(espectre_strings)
                
                writer.writerow(data_row)
                
            self.get_logger().info(f"7. Dades guardades correctament a '{dataset_path}'!")
            self.get_logger().info("   (Recorda que en Mode Collect NO s'actualitza el JSON d'inspeccions).")


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
    parser.add_argument('--mode', type=str, choices=['predict', 'collect'], default='predict',
                        help="Mode de funcionament: 'predict' (per defecte) o 'collect' per crear dataset.")
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

    # parse_known_args evita conflictes amb els args interns de ROS2
    known, _ = parser.parse_known_args()
    return known


def main(args=None):
    rclpy.init(args=args)
    params = parse_args()

    manager = InspectionManager()
    manager.run_inspection(
        x=params.x,
        y=params.y,
        z=params.z,
        mode=params.mode,      
        label=params.label,
        vis_mtr=params.vis_mtr,
        vis_mti=params.vis_mti,
        vis_thp=params.vis_thp,
        nir_mtr=params.nir_mtr,
        nir_mti=params.nir_mti,
        nir_thp=params.nir_thp,
    )

    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
import csv
import numpy as np
from datetime import datetime, timezone
import sys

# Importem els serveis
from arm_control.srv import HyperspectralCommand, PredictMaterial

class InspectionManager(Node):
    def __init__(self):
        super().__init__('inspection_manager')

        # Creem els clients
        self.camera_client = self.create_client(HyperspectralCommand, 'hyperspectral/measurement')
        self.ml_client = self.create_client(PredictMaterial, 'hyperspectral/predict_material')

    # 1. AFEGIM x, y, z com a paràmetres de la funció
    def run_inspection(self, x, y, z):
        self.get_logger().info("1. Esperant que els serveis estiguin actius...")
        self.camera_client.wait_for_service()
        self.ml_client.wait_for_service()

        self.get_logger().info(f"2. Cridant la càmera hiperespectral a coordenades ({x}, {y}, {z})...")
        req_cam = HyperspectralCommand.Request()
        req_cam.command = "GSM"
        # Enviem les coordenades cap a la petició de la càmera
        req_cam.x_coord = float(x)
        req_cam.y_coord = float(y)
        req_cam.z_coord = float(z)

        # Cridem a la càmera
        future_cam = self.camera_client.call_async(req_cam)
        rclpy.spin_until_future_complete(self, future_cam)
        res_cam = future_cam.result()

        if not res_cam.vis_ok or not res_cam.nir_ok:
            self.get_logger().error("Error: La càmera no ha pogut prendre les dades.")
            return

        self.get_logger().info("3. Enviant dades al model de Machine Learning...")
        req_ml = PredictMaterial.Request()
        # Convertir de uint16 a float32 per compatibilitat amb el ML
        req_ml.vis_spectrum = [float(x) for x in res_cam.vis_spectrum]
        req_ml.nir_spectrum = [float(x) for x in res_cam.nir_spectrum]

        # Cridem al ML
        future_ml = self.ml_client.call_async(req_ml)
        rclpy.spin_until_future_complete(self, future_ml)
        res_ml = future_ml.result()

        self.get_logger().info("4. Generant el CSV i l'estructura JSON...")

        # ---------------------------------------------------------
        # A) ESCRIPTURA DEL CSV AMB IA (Material i Confidence)
        # ---------------------------------------------------------
        now = datetime.now()
        filename = f"Hyperspectral_data_{now.day:02d}_{now.month:02d}_{now.year}.csv"
        csv_dir = os.path.expanduser("~/Documents/Hyperspectral_camera/src/arm_control/resource")
        os.makedirs(csv_dir, exist_ok=True)
        csv_path = os.path.join(csv_dir, filename)

        file_exists = os.path.isfile(csv_path)

        # Comptem quantes línies té el CSV per saber quina 'row' estem escrivint
        row_index = 1
        if file_exists:
            with open(csv_path, 'r') as f:
                row_index = sum(1 for _ in f)

        with open(csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            if not file_exists:
                vis_wavelengths = np.linspace(325.3, 792.6, 256)
                nir_wavelengths = np.linspace(991, 1707, 256)
                header = ['Measure_Type', 'Date', 'Time', 'X_Coordinate', 'Y_Coordinate', 'Z_Coordinate', 'Material', 'Confidence']
                header.extend([f'{wl:.1f}' for wl in vis_wavelengths])
                header.extend([f'{wl:.1f}' for wl in nir_wavelengths])
                writer.writerow(header)

            date_str = now.strftime("%Y-%m-%d")
            time_str = now.strftime("%H:%M:%S.%f")[:-3]
            
            # 2. AFEGIM x, y, z reals directament al CSV!
            data_row = ["GSM", date_str, time_str, x, y, z, res_ml.material, res_ml.confidence]
            data_row.extend(res_cam.vis_spectrum)
            data_row.extend(res_cam.nir_spectrum)
            writer.writerow(data_row)


        # ---------------------------------------------------------
        # B) ACTUALITZACIÓ DE L'ESTRUCTURA JSON
        # ---------------------------------------------------------
        temps_actual = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")
        
        # 3. EL JSON ES QUEDA AMB "null" (None a Python)
        nou_registre = {
            "id": None,
            "timestamp": temps_actual,
            "point_coordinates": {
                "x": { "value": None, "error": 0.0, "unit": "m" },
                "y": { "value": None, "error": 0.0, "unit": "m" },
                "z": { "value": None, "error": 0.0, "unit": "m" }
            },
            "row": row_index,
            "material": res_ml.material,
            "confidence": float(res_ml.confidence)
        }

        json_dir = os.path.expanduser("~/hyperspectral_inspections")
        os.makedirs(json_dir, exist_ok=True)
        nom_arxiu_json = os.path.join(json_dir, "registre_inspeccions.json")

        dades_historial = {
            "hyperspectral_camera": {
                "data_pointer_csv": csv_path,
                "measurements": []
            }
        }

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

        self.get_logger().info(f"5. Dades afegides amb èxit al CSV i al JSON!")

        print("\n" + "="*60)
        print(" NOVA MESURA AFEGIDA:")
        print("="*60)
        print(json.dumps(nou_registre, indent=4))
        print("="*60 + "\n")


def main(args=None):
    rclpy.init(args=args)
    manager = InspectionManager()
    
    # 4. LLEGIM ELS PARÀMETRES DE LA TERMINAL I ELS PASSEM A LA FUNCIÓ
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    z = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    manager.run_inspection(x, y, z)
    
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
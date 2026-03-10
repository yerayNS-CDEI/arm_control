#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import os
from datetime import datetime, timezone
import sys

# Importem els serveis
from arm_control.srv import HyperspectralCommand, PredictMaterial

class InspectionManager(Node):
    def __init__(self):
        super().__init__('inspection_manager')

        # Creem els clients per parlar amb la càmera i amb el teu node de ML
        self.camera_client = self.create_client(HyperspectralCommand, 'hyperspectral/measurement')
        self.ml_client = self.create_client(PredictMaterial, 'hyperspectral/predict_material')

    def run_inspection(self, x, y, z):
        self.get_logger().info("1. Esperant que els serveis estiguin actius...")
        self.camera_client.wait_for_service()
        self.ml_client.wait_for_service()

        self.get_logger().info(f"2. Cridant la càmera hiperespectral a les coordenades ({x}, {y}, {z})...")
        req_cam = HyperspectralCommand.Request()
        req_cam.command = "GSM"
        req_cam.x_coord = float(x)
        req_cam.y_coord = float(y)
        req_cam.z_coord = float(z)

        # Cridem a la càmera (Això dispararà VIS i NIR al laboratori)
        future_cam = self.camera_client.call_async(req_cam)
        rclpy.spin_until_future_complete(self, future_cam)
        res_cam = future_cam.result()

        if not res_cam.vis_ok or not res_cam.nir_ok:
            self.get_logger().error("Error: La càmera no ha pogut prendre les dades.")
            return

        # Extraiem la ruta del CSV (recorda que vam fer que el missatge fos "OK|/ruta.csv")
        try:
            csv_path = res_cam.message.split("|")[1]
        except:
            csv_path = "ruta_no_trobada.csv"

        self.get_logger().info("3. Enviant dades al model de Machine Learning...")
        req_ml = PredictMaterial.Request()
        req_ml.vis_spectrum = res_cam.vis_spectrum
        req_ml.nir_spectrum = res_cam.nir_spectrum

        # Cridem al ML i esperem el material
        future_ml = self.ml_client.call_async(req_ml)
        rclpy.spin_until_future_complete(self, future_ml)
        res_ml = future_ml.result()

        self.get_logger().info("4. Generant JSON final...")
        temps_actual = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        # Construïm l'estructura JSON exacta demanada
        nou_registre = {
            "id": "hsc-001",
            "point_coordinates": {
                "x": { "value": float(x), "error": 0.0375, "unit": "m" },
                "y": { "value": float(y), "error": 0.0375, "unit": "m" },
                "z": { "value": float(z), "error": 0.0375, "unit": "m" }
            },
            "data_pointer_csv": csv_path,
            "material": res_ml.material,
            "confidence": float(res_ml.confidence),
            "timestamp": temps_actual
        }

        # Crear ruta segura per al JSON a Ubuntu
        json_dir = os.path.expanduser("~/hyperspectral_inspections")
        os.makedirs(json_dir, exist_ok=True)
        nom_arxiu_json = os.path.join(json_dir, "registre_inspeccions.json")

        if os.path.exists(nom_arxiu_json):
            try:
                with open(nom_arxiu_json, 'r') as f:
                    dades_historial = json.load(f)
            except json.JSONDecodeError:
                # Si l'arxiu estigués trencat o buit, el reiniciem
                dades_historial = {"hyperspectral_camera": []}
        else:
            # Si és la primera vegada que executem, creem l'estructura buida
            dades_historial = {"hyperspectral_camera": []}
        
        dades_historial["hyperspectral_camera"].append(nou_registre)

        try:
            with open(nom_arxiu_json, 'w') as f:
                json.dump(dades_historial, f, indent=4)
            self.get_logger().info(f"5. Dades afegides correctament a l'arxiu: {nom_arxiu_json}")
        except Exception as e:
            self.get_logger().error(f"Error al guardar el JSON: {e}")

        # L'imprimim per pantalla
        print("\n" + "="*60)
        print(" RESULTAT DE LA INSPECCIÓ:")
        print("="*60)
        print(json.dumps(nou_registre, indent=4))
        print("="*60 + "\n")



def main(args=None):
    rclpy.init(args=args)
    manager = InspectionManager()
    
    # Llegim les coordenades de la terminal si ens les passen
    if len(sys.argv) >= 4:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
        except ValueError:
            print("Error: Les coordenades han de ser números.")
            x, y, z = 0.0, 0.0, 0.0
    else:
        # Valors per defecte si ho executem sense arguments
        x, y, z = 0.45, -0.12, 0.30

    # Simulem que el robot ens dóna unes coordenades d'exemple
    manager.run_inspection(x, y, z)
    
    manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

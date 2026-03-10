#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import pickle
import os
from ament_index_python.packages import get_package_share_directory

# ATENCIÓ: Canvia 'arm_control' pel nom del teu paquet si és diferent
from arm_control.srv import PredictMaterial

class MLInferenceNode(Node):
    def __init__(self):
        super().__init__('ml_inference_node')
        
        # Paràmetre per indicar on estarà el model guardat
        try:
            package_dir = get_package_share_directory('arm_control')
            default_model_path = os.path.join(package_dir, '..', '..', '..', '..', 'src', 'arm_control', 'models', 'model_entrenat.pkl')
            default_model_path = os.path.abspath(default_model_path)
        except:
            default_model_path = os.path.expanduser('~/model_entrenat.pkl')

        self.declare_parameter('model_path', default_model_path)
        self.model_path = self.get_parameter('model_path').value
        
        # Intentem carregar el model real
        self.model = None
        self._load_model()

        # Creem el servidor del servei
        self.srv = self.create_service(
            PredictMaterial, 
            'hyperspectral/predict_material', 
            self.predict_callback
        )
        self.get_logger().info(" Node de ML a punt! Esperant dades per analitzar...")

    def _load_model(self):
        """Carrega el model a la memòria. Si no existeix, avisa que farà simulacions."""
        if os.path.exists(self.model_path):
            try:
                with open(self.model_path, 'rb') as f:
                    self.model = pickle.load(f)
                self.get_logger().info(f"✓ Model de ML carregat correctament des de: {self.model_path}")
            except Exception as e:
                self.get_logger().error(f"✗ Error carregant el model: {e}")
        else:
            self.get_logger().warn(f"⚠ Model '{self.model_path}' no trobat.")
            self.get_logger().warn("S'utilitzarà un model FALS (Mock) per poder fer proves de xarxa.")

    def predict_callback(self, request, response):
        """S'executa cada cop que el Gestor demana predir un material."""
        self.get_logger().info("Rebuda petició de predicció...")

        # 1. Comprovació de seguretat
        if not request.vis_spectrum or not request.nir_spectrum:
            self.get_logger().error("Error: S'han rebut espectres buits.")
            response.material = "ERROR_DADES_BUIDES"
            response.confidence = 0.0
            return response

        # 2. Mode Simulació (si estem provant a casa sense el .pkl)
        if self.model is None:
            self.get_logger().info("Processant dades... (SIMULACIÓ)")
            response.material = "Cartró (Mock)"
            response.confidence = 0.98
            return response

        # 3. Mode Real (quan tinguis el .pkl)
        try:
            # Convertim les dades de ROS a Numpy
            vis_data = np.array(request.vis_spectrum)
            nir_data = np.array(request.nir_spectrum)
            
            # Les ajuntem en format taula per a Scikit-Learn (1 fila, N columnes)
            X_input = np.concatenate((vis_data, nir_data)).reshape(1, -1)
            
            # Fem la predicció
            prediction = self.model.predict(X_input)
            
            # Extreure la confiança (si l'algorisme ho permet)
            if hasattr(self.model, "predict_proba"):
                probabilities = self.model.predict_proba(X_input)
                confidence = float(np.max(probabilities))
            else:
                confidence = 1.0

            # Retornem el resultat
            response.material = str(prediction[0])
            response.confidence = confidence
            self.get_logger().info(f"✓ Resultat: {response.material} (Confiança: {confidence:.2f})")
            
        except Exception as e:
            self.get_logger().error(f"Error fatal durant la inferència: {e}")
            response.material = "ERROR_INFERENCIA"
            response.confidence = 0.0

        return response

def main(args=None):
    rclpy.init(args=args)
    node = MLInferenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
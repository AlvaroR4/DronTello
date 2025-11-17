#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class FiltroColorBrilloNode(Node):

    def __init__(self):
        super().__init__('filtro_color_brillo_node')
                
        self.AJUSTE_BRILLO = -50
        
        self.COLOR_MIN = np.array([0, 82, 134])
        self.COLOR_MAX = np.array([3, 209, 255])
        
        
        self.bridge = CvBridge()

        # Suscriptor principal
        self.suscripcion_imagen = self.create_subscription(
            Image,
            '/tello/imagen',
            self.procesar_imagen_callback,
            10)
        
        # Publicador para la imagen con brillo ajustado (¡NUEVO!)
        self.publicador_brillo = self.create_publisher(
            Image,
            '/tello/debug/brillo_ajustado',
            10)
        
        # Publicador para la máscara resultante
        self.publicador_mascara = self.create_publisher(
            Image,
            '/tello/debug/mascara_color',
            10)

        self.get_logger().info("Nodo de filtro (Brillo + Mascara) iniciado.")
        self.get_logger().info(f"Publicando imagen oscurecida en /tello/debug/brillo_ajustado")
        self.get_logger().info(f"Publicando máscara final en /tello/debug/mascara_color")

    def publicar_imagen_cv(self, imagen_cv, publicador, cabecera):
        """Publica una imagen de OpenCV (BGR o Gris) en un topic de ROS."""
        try:
            # Si es 1 canal (mascara), la convierte a BGR para verla fácil
            if len(imagen_cv.shape) == 2:
                imagen_cv_bgr = cv2.cvtColor(imagen_cv, cv2.COLOR_GRAY2BGR)
            else:
                # Si ya es BGR (brillo_ajustado), la usa directamente
                imagen_cv_bgr = imagen_cv
                
            msg_salida = self.bridge.cv2_to_imgmsg(imagen_cv_bgr, "bgr8")
            msg_salida.header = cabecera # Usar la misma cabecera
            publicador.publish(msg_salida)
            
        except CvBridgeError as e:
            self.get_logger().error(f"Error de CvBridge al publicar: {e}")

    def procesar_imagen_callback(self, msg):
        try:
            imagen_cv = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cabecera_msg = msg.header
        except CvBridgeError as e:
            self.get_logger().error(f"Error de CvBridge al recibir: {e}")
            return

        # 1. Aplicar ajuste de brillo
        imagen_oscura = cv2.convertScaleAbs(imagen_cv, alpha=1.0, beta=self.AJUSTE_BRILLO)
        
        # --- Publicar imagen oscurecida (¡NUEVO!) ---
        self.publicar_imagen_cv(imagen_oscura, self.publicador_brillo, cabecera_msg)
        # -----------------------------------------------

        # 2. Convertir la imagen OSCURECIDA a HSV
        imagen_hsv = cv2.cvtColor(imagen_oscura, cv2.COLOR_BGR2HSV)

        # 3. Aplicar la máscara de color (inRange)
        mascara = cv2.inRange(imagen_hsv, self.COLOR_MIN, self.COLOR_MAX)

        # 4. Publicar la máscara resultante
        self.publicar_imagen_cv(mascara, self.publicador_mascara, cabecera_msg)


def main(args=None):
    rclpy.init(args=args)
    
    filtro_nodo = FiltroColorBrilloNode()
    
    try:
        rclpy.spin(filtro_nodo)
    except KeyboardInterrupt:
        pass
    finally:
        filtro_nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
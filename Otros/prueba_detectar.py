#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ProcesadorDebugNode(Node):

    def __init__(self):
        super().__init__('procesador_debug_node')
        
        # --- Constantes Globales para Ajustes ---
        
        self.AJUSTE_BRILLO = -200
        
        self.FACTOR_CONTRASTE = 0.7
        
        self.FACTOR_SATURACION = 0.5
        
        self.CANNY_UMBRAL_BAJO = 50
        self.CANNY_UMBRAL_ALTO = 150
        
        self.bridge = CvBridge()

        # Suscriptor principal
        self.suscripcion_imagen = self.create_subscription(
            Image,
            '/tello/imagen',
            self.procesar_imagen_callback,
            10)
        
        # --- Publicadores de DEBUG ---
        
        # 1. Ajustes de imagen
        self.pub_brillo = self.create_publisher(Image, '/tello/debug/brillo', 10)
        self.pub_contraste = self.create_publisher(Image, '/tello/debug/contraste', 10)
        self.pub_saturacion = self.create_publisher(Image, '/tello/debug/saturacion', 10)
        
        # 2. Bordes sobre ajustes
        self.pub_bordes_brillo = self.create_publisher(Image, '/tello/debug/bordes_brillo', 10)
        self.pub_bordes_contraste = self.create_publisher(Image, '/tello/debug/bordes_contraste', 10)
        self.pub_bordes_saturacion = self.create_publisher(Image, '/tello/debug/bordes_saturacion', 10)

        self.get_logger().info("Nodo de depuración de procesamiento de imagen iniciado.")

    def publicar_imagen_cv(self, imagen_cv, publicador, cabecera):
        """Publica una imagen de OpenCV (BGR o Gris) en un topic de ROS."""
        try:
            # Convertir imágenes de 1 canal (gris/bordes) a BGR para fácil visualización
            if len(imagen_cv.shape) == 2:
                imagen_cv_bgr = cv2.cvtColor(imagen_cv, cv2.COLOR_GRAY2BGR)
            else:
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

        # --- 1. Ajuste de Brillo ---
        # alpha=1.0 (sin cambio contraste), beta=AJUSTE_BRILLO (cambio brillo)
        imagen_brillo = cv2.convertScaleAbs(imagen_cv, alpha=1.0, beta=self.AJUSTE_BRILLO)
        self.publicar_imagen_cv(imagen_brillo, self.pub_brillo, cabecera_msg)

        # --- 2. Ajuste de Contraste ---
        # alpha=FACTOR_CONTRASTE (cambio contraste), beta=0 (sin cambio brillo)
        imagen_contraste = cv2.convertScaleAbs(imagen_cv, alpha=self.FACTOR_CONTRASTE, beta=0)
        self.publicar_imagen_cv(imagen_contraste, self.pub_contraste, cabecera_msg)

        # --- 3. Ajuste de Saturación ---
        imagen_hsv = cv2.cvtColor(imagen_cv, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(imagen_hsv)
        # Multiplicamos la saturación y aseguramos que quede entre 0 y 255
        s = np.clip(s * self.FACTOR_SATURACION, 0, 255).astype("uint8")
        imagen_hsv_mod = cv2.merge([h, s, v])
        imagen_saturacion = cv2.cvtColor(imagen_hsv_mod, cv2.COLOR_HSV2BGR)
        self.publicar_imagen_cv(imagen_saturacion, self.pub_saturacion, cabecera_msg)

        # --- 4. Detección de Bordes (Canny) ---
        
        # A. Bordes sobre imagen de brillo
        gris_brillo = cv2.cvtColor(imagen_brillo, cv2.COLOR_BGR2GRAY)
        blur_brillo = cv2.GaussianBlur(gris_brillo, (5, 5), 0)
        bordes_brillo = cv2.Canny(blur_brillo, self.CANNY_UMBRAL_BAJO, self.CANNY_UMBRAL_ALTO)
        self.publicar_imagen_cv(bordes_brillo, self.pub_bordes_brillo, cabecera_msg)

        # B. Bordes sobre imagen de contraste
        gris_contraste = cv2.cvtColor(imagen_contraste, cv2.COLOR_BGR2GRAY)
        blur_contraste = cv2.GaussianBlur(gris_contraste, (5, 5), 0)
        bordes_contraste = cv2.Canny(blur_contraste, self.CANNY_UMBRAL_BAJO, self.CANNY_UMBRAL_ALTO)
        self.publicar_imagen_cv(bordes_contraste, self.pub_bordes_contraste, cabecera_msg)

        # C. Bordes sobre imagen de saturación
        gris_saturacion = cv2.cvtColor(imagen_saturacion, cv2.COLOR_BGR2GRAY)
        blur_saturacion = cv2.GaussianBlur(gris_saturacion, (5, 5), 0)
        bordes_saturacion = cv2.Canny(blur_saturacion, self.CANNY_UMBRAL_BAJO, self.CANNY_UMBRAL_ALTO)
        self.publicar_imagen_cv(bordes_saturacion, self.pub_bordes_saturacion, cabecera_msg)


def main(args=None):
    rclpy.init(args=args)
    
    procesador_nodo = ProcesadorDebugNode()
    
    try:
        rclpy.spin(procesador_nodo)
    except KeyboardInterrupt:
        pass
    finally:
        procesador_nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
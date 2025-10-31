#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

ROS_TOPIC_IMAGEN_INPUT = '/tello/imagen'
ROS_TOPIC_POSE_DRON_INPUT = '/tello/pose_corregida'
ROS_TOPIC_PUNTO_ANGULO_OUTPUT = '/tello/punto_y_angulo'
ROS_TOPIC_IMAGEN_DEBUG_OUTPUT = '/tello/imagen_puertas'
ROS_TOPIC_PUNTO_MUNDO_OUTPUT = '/tello/punto_mundo'

ANCHO_IMAGEN = 960
ALTO_IMAGEN = 720
# Naranja
COLOR_MIN = np.array([0, 191, 63])  
COLOR_MAX = np.array([10, 255, 142])
MIN_CORNER_AREA = 100

ALTO_REAL_PUERTA_M = 0.285 
FOCAL_PIXELS = 617.0

class NodoDeteccionLeds(Node):
    def __init__(self):
        super().__init__('nodo_deteccion_leds')
        self.get_logger().info("Iniciando el nodo de detección de LEDs (4 esquinas).")

        self.pos_dron_mundo = np.zeros(3)
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        self.bridge = CvBridge()

        self.camera_matrix = np.array([
            [FOCAL_PIXELS, 0, ANCHO_IMAGEN / 2],
            [0, FOCAL_PIXELS, ALTO_IMAGEN / 2],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub_imagen = self.create_subscription(Image, ROS_TOPIC_IMAGEN_INPUT, self.callback_imagen, qos_best_effort)
        self.sub_pose = self.create_subscription(Float32MultiArray, ROS_TOPIC_POSE_DRON_INPUT, self.callback_pose, qos_reliable)
        self.pub_punto_angulo = self.create_publisher(Float32MultiArray, ROS_TOPIC_PUNTO_ANGULO_OUTPUT, 10)
        self.pub_imagen_debug = self.create_publisher(Image, ROS_TOPIC_IMAGEN_DEBUG_OUTPUT, 10)
        self.pub_punto_mundo = self.create_publisher(PointStamped, ROS_TOPIC_PUNTO_MUNDO_OUTPUT, 10)

        self.get_logger().info("Nodo inicializado y listo para detectar puertas.")

    def callback_pose(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.pos_dron_mundo = np.array(msg.data[0:3])
            self.roll, self.pitch, self.yaw = msg.data[3:6]

    def callback_imagen(self, msg_imagen):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg_imagen, "bgr8")
            frame_bgr = cv2.resize(frame_bgr, (ANCHO_IMAGEN, ALTO_IMAGEN))
            frame_hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        except Exception:
            return

        esquinas_px, frame_debug = self.detectar_esquinas(frame_hsv, frame_bgr.copy())

        if len(esquinas_px) == 4:
            esquinas_ordenadas = self.ordenar_esquinas(esquinas_px)
            
            x_centro_px = int(np.mean([p[0] for p in esquinas_ordenadas]))
            y_centro_px = int(np.mean([p[1] for p in esquinas_ordenadas]))
            
            alto_izq = np.linalg.norm(np.array(esquinas_ordenadas[0]) - np.array(esquinas_ordenadas[1]))
            alto_der = np.linalg.norm(np.array(esquinas_ordenadas[3]) - np.array(esquinas_ordenadas[2]))
            alto_px = (alto_izq + alto_der) / 2.0
            
            if alto_px > 0:
                distancia = (ALTO_REAL_PUERTA_M * FOCAL_PIXELS) / alto_px
                
                punto_cuerpo_centro = self.px_a_cuerpo(x_centro_px, y_centro_px, distancia)
                punto_mundo_centro, R_dron_a_mundo = self.cuerpo_a_mundo(punto_cuerpo_centro)

                angulo_global = self.calcular_angulo_global(esquinas_ordenadas, distancia, R_dron_a_mundo)
                
                self.publicar_punto_y_angulo(punto_mundo_centro, angulo_global)
                self.publicar_punto_mundo(punto_mundo_centro, msg_imagen.header.stamp)

                cv2.polylines(frame_debug, [np.array(esquinas_ordenadas, dtype=np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.circle(frame_debug, (x_centro_px, y_centro_px), 5, (0, 0, 255), -1)
                texto_angulo = f"Angulo Global: {angulo_global:.1f} deg"
                cv2.putText(frame_debug, texto_angulo, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

        try:
            msg_debug = self.bridge.cv2_to_imgmsg(frame_debug, "bgr8")
            msg_debug.header = msg_imagen.header
            self.pub_imagen_debug.publish(msg_debug)
        except Exception:
            pass

    def detectar_esquinas(self, frame_hsv, frame_debug):
        rojo_min1 = np.array([0, 120, 70])
        rojo_max1 = np.array([10, 255, 255])    
        rojo_min2 = np.array([170, 120, 70])
        rojo_max2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(frame_hsv, rojo_min1, rojo_max1)
        mask2 = cv2.inRange(frame_hsv, rojo_min2, rojo_max2)
        mask = mask1 + mask2

        #mask = cv2.inRange(frame_hsv, COLOR_MIN, COLOR_MAX)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        esquinas_detectadas = []
        for cnt in contornos:
            if cv2.contourArea(cnt) > MIN_CORNER_AREA:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    esquinas_detectadas.append((cx, cy))
                    cv2.circle(frame_debug, (cx, cy), 7, (255, 0, 0), -1)
        return esquinas_detectadas, frame_debug

    def ordenar_esquinas(self, puntos):
        # Ordena por la coordenada X
        puntos.sort(key=lambda p: p[0])
        izquierdos = sorted(puntos[:2], key=lambda p: p[1])
        derechos = sorted(puntos[2:], key=lambda p: p[1])
        # [sup_izq, inf_izq, sup_der, inf_der]
        return [izquierdos[0], izquierdos[1], derechos[0], derechos[1]]

    def px_a_cuerpo(self, cx, cy, distancia):
        x_cam = (cx - ANCHO_IMAGEN / 2) * distancia / FOCAL_PIXELS
        y_cam = (cy - ALTO_IMAGEN / 2) * distancia / FOCAL_PIXELS
        return np.array([distancia, x_cam, y_cam]) # [Adelante, Derecha, Abajo]

    def cuerpo_a_mundo(self, punto_cuerpo):
        roll_rad, pitch_rad, yaw_rad = map(math.radians, [self.roll, self.pitch, self.yaw])
        R_x = np.array([[1, 0, 0], [0, math.cos(roll_rad), -math.sin(roll_rad)], [0, math.sin(roll_rad), math.cos(roll_rad)]])
        R_y = np.array([[math.cos(pitch_rad), 0, math.sin(pitch_rad)], [0, 1, 0], [-math.sin(pitch_rad), 0, math.cos(pitch_rad)]])
        R_z = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad), 0], [math.sin(yaw_rad), math.cos(yaw_rad), 0], [0, 0, 1]])
        R_dron_a_mundo = R_z @ R_y @ R_x
        punto_mundo = self.pos_dron_mundo + R_dron_a_mundo @ punto_cuerpo
        return punto_mundo, R_dron_a_mundo

    def calcular_angulo_global(self, esquinas_ordenadas_px, R_dron_a_mundo):
        # 1. Definir el modelo 3D de la puerta en su propio sistema de coordenadas.
        # Asumimos que la puerta es un rectángulo. Si no es un cuadrado, ajusta el ancho.
        alto_real = ALTO_REAL_PUERTA_M
        ancho_real = alto_real # Asumimos que la puerta es cuadrada. Cambia esto si no lo es.
        
        puntos_objeto_3d = np.array([
            [-ancho_real / 2,  alto_real / 2, 0], # Superior Izquierda
            [-ancho_real / 2, -alto_real / 2, 0], # Inferior Izquierda
            [ ancho_real / 2,  alto_real / 2, 0], # Superior Derecha
            [ ancho_real / 2, -alto_real / 2, 0]  # Inferior Derecha
        ], dtype=np.float32)

        # 2. Convertir esquinas a un array de numpy con el formato correcto.
        puntos_imagen_2d = np.array(esquinas_ordenadas_px, dtype=np.float32)

        # 3. Resolver el problema PnP para obtener la rotación (rvec) y traslación (tvec).
        # Esto nos dice cómo está orientada la puerta con respecto a la CÁMARA.
        try:
            success, rvec, tvec = cv2.solvePnP(puntos_objeto_3d, puntos_imagen_2d, self.camera_matrix, self.dist_coeffs)
            if not success:
                return 0.0 # No se pudo calcular
        except:
            return 0.0 # Error en el cálculo

        # 4. Convertir el vector de rotación 'rvec' a una matriz de rotación 3x3.
        R_puerta_a_camara, _ = cv2.Rodrigues(rvec)

        # 5. Aplicar la cadena de transformaciones para obtener la orientación en el MUNDO.
        # (Mundo <- Dron <- Cámara <- Puerta)
        # La transformación de Cámara a Dron (FRD) es una rotación fija.
        R_camara_a_dron = np.array([
            [0, 0, 1],  # El eje X del dron (Adelante) es el Z de la cámara.
            [1, 0, 0],  # El eje Y del dron (Derecha) es el X de la cámara.
            [0, 1, 0]   # El eje Z del dron (Abajo) es el Y de la cámara.
        ])
        R_puerta_a_mundo = R_dron_a_mundo @ R_camara_a_dron @ R_puerta_a_camara
        
        # 6. Extraer el ángulo de Yaw (rotación en Z) de la matriz de rotación final.
        angulo_rad = math.atan2(R_puerta_a_mundo[1, 0], R_puerta_a_mundo[0, 0])
        return math.degrees(angulo_rad)

    def publicar_punto_y_angulo(self, punto, angulo):
        msg = Float32MultiArray(data=[punto[0], punto[1], punto[2], float(angulo)])
        self.pub_punto_angulo.publish(msg)

    def publicar_punto_mundo(self, punto, stamp):
        msg = PointStamped()
        msg.header.stamp, msg.header.frame_id = stamp, 'map'
        msg.point.x, msg.point.y, msg.point.z = punto
        self.pub_punto_mundo.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo_detector = NodoDeteccionLeds()
    try:
        rclpy.spin(nodo_detector)
    except KeyboardInterrupt:
        print("Cerrando nodo.")
    finally:
        nodo_detector.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
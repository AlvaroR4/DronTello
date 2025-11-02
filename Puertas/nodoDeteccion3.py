#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import traceback
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped

ROS_TOPIC_IMAGEN_RAW_INPUT = 'camera/image_raw'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'
ROS_TOPIC_POSE_ANGLES = '/tello/pose_corregida'

ANCHO_IMAGEN = 960
ALTO_IMAGEN = 720

COLOR_MIN = np.array([0, 191, 63])
COLOR_MAX = np.array([10, 255, 142])

MIN_CORNER_AREA = 100

ALTO_REAL = 0.285
ANCHO_REAL = 0.21
FOCAL = 617.0

class NodoDeteccion(Node):
    def __init__(self):
        super().__init__('nodo_deteccion')
        self.get_logger().info("Iniciando nodo de detección de puertas.")
        self.bridge = CvBridge()
        
        self.camera_matrix = np.array([
            [FOCAL, 0, ANCHO_IMAGEN / 2],
            [0, FOCAL, ALTO_IMAGEN / 2],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((1, 5))
        ancho_mitad = ANCHO_REAL / 2.0
        alto_mitad = ALTO_REAL / 2.0
        self.puntos_objeto_3d = np.array([
            [-ancho_mitad,  alto_mitad, 0.0], # Esquina superior izquierda (esq4)
            [ ancho_mitad,  alto_mitad, 0.0], # Esquina superior derecha (esq3)
            [ ancho_mitad, -alto_mitad, 0.0], # Esquina inferior derecha (esq2)
            [-ancho_mitad, -alto_mitad, 0.0]  # Esquina inferior izquierda (esq1)
        ], dtype=np.float32)

        self.timer_log = self.create_timer(3.0, self.log_datos)
        self.distancia_estimada = None
        self.distancia_calculada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None

        self.publisher_ = self.create_publisher(PointStamped, '/punto', 10)
        self.pub_punto_a = self.create_publisher(Float32MultiArray, '/punto_y_angulo', 10)

        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.suscripcion_imagen = self.create_subscription(Image, ROS_TOPIC_IMAGEN_RAW_INPUT, self.callback_procesamiento_imagen, qos_profile_sub)

        qos_profile_pose = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.suscripcion_pose = self.create_subscription(Float32MultiArray, ROS_TOPIC_POSE_ANGLES, self.callback_pose, qos_profile_pose)

        self.pos_dron_mundo = np.array([0.0, 0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        qos_profile_pub_data = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.publicador_puertas_detectadas = self.create_publisher(Float32MultiArray, ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT, qos_profile_pub_data)

        qos_profile_pub_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.publicador_imagen_visualizacion = self.create_publisher(Image, ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT, qos_profile_pub_img)

    def publicar_punto_a(self, x, y , z , angulo):
        msg = Float32MultiArray()
        msg.data = [x, y, z, angulo]
        self.pub_punto_a.publish(msg)

    def callback_pose(self, msg: Float32MultiArray):
        try:
            data = list(msg.data)
            if len(data) >= 6:
                self.pos_dron_mundo = np.array([data[0], data[1], data[2]])
                self.roll = float(data[3])
                self.pitch = float(data[4])
                self.yaw = float(data[5])
        except Exception as e:
            self.get_logger().error(f"Error procesando pose recibida: {e}")
            self.get_logger().error(traceback.format_exc())

    def callback_procesamiento_imagen(self, msg_imagen_ros):
        try:
            frame_bgr_raw = self.bridge.imgmsg_to_cv2(msg_imagen_ros, desired_encoding="bgr8")
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_IMAGEN, ALTO_IMAGEN))
            img_hsv = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2HSV)
            puertas_detectadas, img_con_dibujos = self.algoritmoDetectarPuertas(img_hsv, img_procesamiento.copy())

            ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_con_dibujos, encoding="bgr8")
            self.publicador_imagen_visualizacion.publish(ros_image_msg_out)

        except Exception as e_processing_callback:
            self.get_logger().error(f"Error general en callback_procesamiento_imagen: {e_processing_callback}")
            self.get_logger().error(traceback.format_exc())

    def publicar_punto(self, punto_mundo):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(punto_mundo[0])
        msg.point.y = float(punto_mundo[1])
        msg.point.z = float(punto_mundo[2])
        self.publisher_.publish(msg)

    def estimar_distancia(self, alto_puerta_px):
        distancia_mts = (ALTO_REAL * FOCAL) / alto_puerta_px if alto_puerta_px > 0 else 0
        return distancia_mts

    def punto_imagen_a_punto_cuerpo(self, cx, cy, distancia):
        z_cam = distancia
        x_cam = (cx - ANCHO_IMAGEN / 2) * z_cam / FOCAL
        y_cam = (cy - ALTO_IMAGEN / 2) * z_cam / FOCAL
        
        # X_cuerpo (Adelante) = Z_cam
        # Y_cuerpo (Derecha) = X_cam
        # Z_cuerpo (Abajo) = Y_cam
        return np.array([z_cam, x_cam, y_cam])

    def transformar_punto_cuerpo_a_mundo(self, punto_cuerpo):
        roll_rad, pitch_rad, yaw_rad = map(math.radians, [self.roll, self.pitch, self.yaw])
        cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)
        cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        R_x = np.array([[1, 0, 0], [0, cos_r, -sin_r], [0, sin_r, cos_r]])
        R_y = np.array([[cos_p, 0, sin_p], [0, 1, 0], [-sin_p, 0, cos_p]])
        R_z = np.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])
        R_cuerpo_a_mundo = R_z @ R_y @ R_x
        
        punto_mundo = self.pos_dron_mundo + R_cuerpo_a_mundo @ punto_cuerpo
        return punto_mundo, R_cuerpo_a_mundo

    def algoritmoDetectarPuertas(self, img_hsv, img_visualizacion):
        rojo_min1 = np.array([0, 120, 70]) #cambiar esto arriba
        rojo_max1 = np.array([10, 255, 255])    
        rojo_min2 = np.array([170, 120, 70])
        rojo_max2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(img_hsv, rojo_min1, rojo_max1)
        mask2 = cv2.inRange(img_hsv, rojo_min2, rojo_max2)
        mask = mask1 + mask2
    
        puntos_esquinas_detectados = []
        #mask = cv2.inRange(img_hsv, COLOR_MIN, COLOR_MAX)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > MIN_CORNER_AREA:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    puntos_esquinas_detectados.append((cx, cy))
                    cv2.circle(img_visualizacion, (cx, cy), 7, (255, 0, 0), -1)

        puertas_encontradas = self.tratarPuerta(puntos_esquinas_detectados, img_visualizacion)
        return puertas_encontradas, img_visualizacion

    def tratarPuerta(self, puntos_esquinas, img_visualizacion):
        puertas = []

        if len(puntos_esquinas) >= 4:
            x_sorted = sorted(puntos_esquinas, key=lambda p: p[0])
            x_menor, x_menor2, x_menor3, x_menor4 = x_sorted[0:4]

            esq1, esq4 = (x_menor, x_menor2) if x_menor[1] > x_menor2[1] else (x_menor2, x_menor)
            esq2, esq3 = (x_menor3, x_menor4) if x_menor3[1] > x_menor4[1] else (x_menor4, x_menor3)
            
            puntos_imagen_2d = np.array([esq4, esq3, esq2, esq1], dtype=np.float32)
            success, rvec, _ = cv2.solvePnP(self.puntos_objeto_3d, puntos_imagen_2d, self.camera_matrix, self.dist_coeffs)
            
            if not success:
                return [], img_visualizacion

            _, R_dron_a_mundo = self.transformar_punto_cuerpo_a_mundo(np.array([0,0,0]))
            R_puerta_a_camara, _ = cv2.Rodrigues(rvec)
            R_camara_a_dron = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
            R_puerta_a_mundo = R_dron_a_mundo @ R_camara_a_dron @ R_puerta_a_camara
            vector_normal_mundo = R_puerta_a_mundo[:, 2]
            angulo = math.degrees(math.atan2(vector_normal_mundo[1], vector_normal_mundo[0]))
            
            alto_puerta = math.sqrt((esq1[0] - esq4[0])**2 + (esq1[1] - esq4[1])**2)
            ancho_puerta = math.sqrt((esq2[0] - esq1[0])**2 + (esq2[1] - esq1[1])**2)
            
            x_centro_puerta = int(np.mean([p[0] for p in [esq1, esq2, esq3, esq4]]))
            y_centro_puerta = int(np.mean([p[1] for p in [esq1, esq2, esq3, esq4]]))

            distancia_estimada = self.estimar_distancia(alto_puerta)
            
            punto_cuerpo = self.punto_imagen_a_punto_cuerpo(x_centro_puerta, y_centro_puerta, distancia_estimada)
            punto_mundo, _ = self.transformar_punto_cuerpo_a_mundo(punto_cuerpo)

            self.distancia_estimada = distancia_estimada
            self.coordenada_X, self.coordenada_Y, self.coordenada_Z = punto_cuerpo
            self.distancia_calculada = np.linalg.norm(punto_cuerpo)
            self.punto_mundo = punto_mundo

            puerta_detectada = {'x_centro': x_centro_puerta, 'y_centro': y_centro_puerta, 'ancho': ancho_puerta, 'alto': alto_puerta}
            puertas.append(puerta_detectada)
            
            self.publicar_punto(punto_mundo)
            self.publicar_punto_a(punto_mundo[0], punto_mundo[1], punto_mundo[2], angulo)

            esquinas = [esq1, esq2, esq3, esq4]
            cv2.polylines(img_visualizacion, [np.array(esquinas, np.int32)], True, (0, 255, 0), 2)
            cv2.circle(img_visualizacion, (x_centro_puerta, y_centro_puerta), 5, (0, 0, 255), -1)
            cv2.putText(img_visualizacion, f"Puerta ({x_centro_puerta},{y_centro_puerta},{angulo:.1f})",
                            (x_centro_puerta - 50, y_centro_puerta - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        return puertas

    def log_datos(self):
        if self.distancia_estimada is not None:
            self.get_logger().info("--- DATOS ---")
            self.get_logger().info(f"Distancia a la puerta: {self.distancia_estimada:.2f} m")
            self.get_logger().info(f"Verificación: La distancia calculada es {self.distancia_calculada:.3f} m")
            self.get_logger().info(f"Coordenada puerta (cuerpo): X={self.coordenada_X:.2f}, Y={self.coordenada_Y:.2f}, Z={self.coordenada_Z:.2f} m")
            if self.punto_mundo is not None:
                self.get_logger().info(f"Coordenadas del punto en mundo: ({self.punto_mundo[0]:.2f}, {self.punto_mundo[1]:.2f}, {self.punto_mundo[2]:.2f})")
            self.get_logger().info("-----------------------------")

    def destroy_node(self):
        self.get_logger().info("Fin NodoDeteccion")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo_deteccion = None
    try:
        nodo_deteccion = NodoDeteccion()
        rclpy.spin(nodo_deteccion)
    except KeyboardInterrupt:
        if nodo_deteccion:
            nodo_deteccion.get_logger().info("Ctrl+C")
    finally:
        if nodo_deteccion:
            nodo_deteccion.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
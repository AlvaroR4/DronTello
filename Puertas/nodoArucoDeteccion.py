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
from collections import deque

ROS_TOPIC_IMAGEN_INPUT = '/tello/imagen'
ROS_TOPIC_POSE_DRON_INPUT = '/tello/pose_corregida'
ROS_TOPIC_PUNTO_MUNDO_OUTPUT = '/tello/punto_mundo'
ROS_TOPIC_PUNTO_ANGULO_OUTPUT = '/tello/punto_y_angulo'
ROS_TOPIC_IMAGEN_DEBUG_OUTPUT = '/tello/imagen_puertas'

ANCHO_IMAGEN = 960
ALTO_IMAGEN = 720

FOCAL_PIXELS = 617.0
MARKER_SIZE = 0.167

ARUCO_DICT = cv2.aruco.DICT_5X5_250


class NodoDeteccionAruco(Node):
    def __init__(self):
        super().__init__('nodo_deteccion_aruco')
        self.get_logger().info("Iniciando el nodo de detección con Aruco.")

        self.pos_dron_mundo = np.array([0.0, 0.0, 0.0])  # [x, y, z]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.primera_pose_recibida = False

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.camera_matrix = np.array([
            [FOCAL_PIXELS, 0, ANCHO_IMAGEN / 2],
            [0, FOCAL_PIXELS, ALTO_IMAGEN / 2],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.zeros((1, 5))
        
        qos_best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.suscripcion_imagen = self.create_subscription(Image, ROS_TOPIC_IMAGEN_INPUT, self.callback_procesamiento_imagen, qos_best_effort)
        self.suscripcion_pose = self.create_subscription(Float32MultiArray, ROS_TOPIC_POSE_DRON_INPUT, self.callback_pose, qos_reliable)
        self.pub_punto_mundo = self.create_publisher(PointStamped, ROS_TOPIC_PUNTO_MUNDO_OUTPUT, 10)
        self.pub_punto_angulo = self.create_publisher(Float32MultiArray, ROS_TOPIC_PUNTO_ANGULO_OUTPUT, 10)
        self.pub_imagen_debug = self.create_publisher(Image, ROS_TOPIC_IMAGEN_DEBUG_OUTPUT, 10)

        self.get_logger().info("Nodo listo.")

    def callback_pose(self, msg: Float32MultiArray):
        pose_data = list(msg.data)
        if len(pose_data) >= 6:
            self.pos_dron_mundo = np.array([pose_data[0], pose_data[1], pose_data[2]])
            self.roll, self.pitch, self.yaw = pose_data[3], pose_data[4], pose_data[5]

            if not self.primera_pose_recibida: 
                self.primera_pose_recibida = True
                self.get_logger().info("COMENZANDO DETECCION PUERTAS")

    def callback_procesamiento_imagen(self, msg_imagen):
        if not self.primera_pose_recibida: 
            return
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg_imagen, desired_encoding="bgr8")
            frame_bgr = cv2.resize(frame_bgr, (ANCHO_IMAGEN, ALTO_IMAGEN))
            frame_gris = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")
            return

        corners, ids, _ = cv2.aruco.detectMarkers(frame_gris, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, self.camera_matrix, self.dist_coeffs)
            
            for i, id_marcador in enumerate(ids):
                rvec, tvec = rvecs[i], tvecs[i]
                esquinas = corners[i].reshape((4, 2))
                
                cv2.drawFrameAxes(frame_bgr, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                cx, cy = int(np.mean(esquinas[:, 0])), int(np.mean(esquinas[:, 1]))
                lado_px = np.linalg.norm(esquinas[0] - esquinas[1])
                distancia = (MARKER_SIZE * FOCAL_PIXELS) / lado_px if lado_px > 0 else 0

                if distancia > 0:
                    punto_cuerpo = self.punto_imagen_a_punto_cuerpo(cx, cy, distancia)
                    punto_mundo, R_dron_a_mundo = self.transformar_punto_cuerpo_a_mundo(punto_cuerpo)
                    angulo_global_puerta = self.calcular_angulo_global_puerta(rvec, R_dron_a_mundo)

                    self.publicar_punto_mundo(punto_mundo, msg_imagen.header.stamp)
                    self.publicar_punto_y_angulo(punto_mundo, angulo_global_puerta)
                    self.get_logger().info(f"ID:{id_marcador[0]} -> Mundo: ({punto_mundo[0]:.2f}, {punto_mundo[1]:.2f}, {punto_mundo[2]:.2f})")
                    
                    self.dibujar_info(frame_bgr, esquinas, ids[0], distancia, punto_mundo, angulo_global_puerta)

        msg_debug = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
        msg_debug.header = msg_imagen.header
        self.pub_imagen_debug.publish(msg_debug)


    def punto_imagen_a_punto_cuerpo(self, cx, cy, distancia):
        z_cam = distancia
        x_cam = (cx - ANCHO_IMAGEN / 2) * z_cam / FOCAL_PIXELS
        y_cam = (cy - ALTO_IMAGEN / 2) * z_cam / FOCAL_PIXELS
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

    def calcular_angulo_global_puerta(self, rvec_marcador_camara, R_dron_a_mundo):
        # 1. Matriz de rotación del marcador con respecto a la CÁMARA
        R_marcador_a_camara, _ = cv2.Rodrigues(rvec_marcador_camara)

        # 2. Matriz de corrección fija que transforma de ejes CÁMARA a ejes DRON (FRD)
        R_camara_a_dron = np.array([
            [0, 0, 1],  # El eje X del dron es el Z de la cámara
            [1, 0, 0],  # El eje Y del dron es el X de la cámara
            [0, 1, 0]   # El eje Z del dron es el Y de la cámara
        ])
        
        # 3. Calcular la rotación del marcador con respecto al MUNDO
        R_marcador_a_mundo = R_dron_a_mundo @ R_camara_a_dron @ R_marcador_a_camara
        
        # 4. EXTRAER EL VECTOR DEL EJE Z Y CALCULAR SU YAW
        # La tercera columna (índice 2) de la matriz de rotación es el vector del eje Z
        # del marcador, expresado en coordenadas del mundo. ¡Este es el vector normal!
        vector_normal_mundo = R_marcador_a_mundo[:, 2]
        vector_normal_mundo = -vector_normal_mundo
        
        # Calculamos el ángulo yaw de este vector en el plano XY del mundo
        yaw_rad = math.atan2(vector_normal_mundo[1], vector_normal_mundo[0])
        
        return math.degrees(yaw_rad)
    
    def publicar_punto_mundo(self, punto, stamp):
        msg = PointStamped()
        msg.header.stamp, msg.header.frame_id = stamp, 'map'
        msg.point.x, msg.point.y, msg.point.z = punto
        self.pub_punto_mundo.publish(msg)

    def publicar_punto_y_angulo(self, punto, angulo_yaw):
        msg = Float32MultiArray()
        msg.data = [punto[0], punto[1], punto[2], float(angulo_yaw)]
        self.pub_punto_angulo.publish(msg)

    def dibujar_info(self, imagen, esquinas, id_marcador, dist, punto_mundo, angulo):
        cv2.polylines(imagen, [esquinas.astype(np.int32)], True, (0, 255, 0), 2)
        texto_mundo = f"Mundo: ({punto_mundo[0]:.2f}, {punto_mundo[1]:.2f}, {punto_mundo[2]:.2f})"
        texto_angulo = f"Angulo Global (Z): {angulo:.1f} deg"
        cv2.putText(imagen, texto_mundo, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(imagen, texto_angulo, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    nodo_detector = NodoDeteccionAruco()
    try:
        rclpy.spin(nodo_detector)
    except KeyboardInterrupt:
        print("Cerrando nodo.")
    finally:
        nodo_detector.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
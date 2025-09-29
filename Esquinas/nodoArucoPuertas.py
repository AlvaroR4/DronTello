#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo de Localización adaptado para recibir la pose desde /tello/pose (drone)
y usar detección de ArUco para obtener rápidamente el punto central (para pruebas).
Basado en nodoImagenPuertas6.py.
"""
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

# Tópicos
ROS_TOPIC_IMAGEN_RAW_INPUT = 'camera/image_raw'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'
ROS_TOPIC_POSE_DRON = '/tello/pose'           # ahora recibimos la pose del dron aquí
ROS_TOPIC_PUNTO = '/punto'
ROS_TOPIC_PUNTO_ANG = '/punto_y_angulo'

# Dimensiones de procesamiento
ANCHO_IMAGEN = 640
ALTO_IMAGEN = 480

# ArUco params (ajusta MARKER_SIZE y FOCAL_PIXELS si hace falta)
ARUCO_DICT = cv2.aruco.DICT_5X5_250
MARKER_SIZE = 0.175     # tamaño del marcador en metros (ejemplo 17.5 cm)
FOCAL_PIXELS = 617.0    # focal en píxeles (ajusta a tu cámara)

# Campos de cámara / calibración aproximada (usados para angulos)
FOCAL = FOCAL_PIXELS
FOV_H = 67.2
FOV_V = 52.3

class ModuloLocalizacion(Node):
    def __init__(self):
        super().__init__('modulo_localizacion_aruco')
        self.get_logger().info("Iniciando Módulo de Localización (ArUco).")
        self.bridge = CvBridge()

        # Pose del dron (vendrá del nodo tello)
        self.pos_dron_mundo = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # variables de salida
        self.distancia_estimada = None
        self.distancia_calculada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None

        # publicadores
        self.publisher_ = self.create_publisher(PointStamped, ROS_TOPIC_PUNTO, 10)
        self.pub_punto_a = self.create_publisher(Float32MultiArray, ROS_TOPIC_PUNTO_ANG, 10)

        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.suscripcion_imagen = self.create_subscription(
            Image,
            ROS_TOPIC_IMAGEN_RAW_INPUT,
            self.callback_procesamiento_imagen,
            qos_profile_sub
        )
        self.get_logger().info(f"Suscrito a imagen RAW en: {ROS_TOPIC_IMAGEN_RAW_INPUT}")

        # suscripción a la pose del dron (la publicará el nodo tello que creamos antes)
        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.suscripcion_pose = self.create_subscription(
            Float32MultiArray,
            ROS_TOPIC_POSE_DRON,
            self.callback_pose,
            qos_profile_pose
        )
        self.get_logger().info(f"Suscrito a la pose del dron en: {ROS_TOPIC_POSE_DRON}")

        # publicadores de detección y visualización
        qos_profile_pub_data = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publicador_puertas_detectadas = self.create_publisher(
            Float32MultiArray, ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT, qos_profile_pub_data)
        self.get_logger().info(f"Publicando datos detectados en: {ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT}")

        qos_profile_pub_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.publicador_imagen_visualizacion = self.create_publisher(
            Image, ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT, qos_profile_pub_img)
        self.get_logger().info(f"Publicando imagen de visualización en: {ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT}")

        # preparar diccionario ArUco / parámetros detector
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # timer informativo opcional
        self.timer_log = self.create_timer(3.0, self.log_datos)

    def callback_pose(self, msg: Float32MultiArray):
        """
        Espera: msg.data = [x, y, z, roll_deg, pitch_deg, yaw_deg] (flexible)
        """
        try:
            data = list(msg.data)
            if len(data) < 3:
                return
            x = float(data[0]); y = float(data[1]); z = float(data[2])
            roll = 0.0; pitch = 0.0; yaw = 0.0
            if len(data) >= 6:
                roll = float(data[3]); pitch = float(data[4]); yaw = float(data[5])
            elif len(data) >= 4:
                yaw = float(data[3])
            self.pos_dron_mundo = [x, y, z]
            self.roll = roll; self.pitch = pitch; self.yaw = yaw
        except Exception:
            return

    def callback_procesamiento_imagen(self, msg_imagen_ros):
        try:
            frame_bgr_raw = self.bridge.imgmsg_to_cv2(msg_imagen_ros, desired_encoding="bgr8")
        except CvBridgeError:
            return
        except Exception:
            return

        if frame_bgr_raw is None:
            return

        try:
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_IMAGEN, ALTO_IMAGEN))
            img_gray = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2GRAY)

            # Detectar markers ArUco
            corners, ids, rejected = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

            img_out = img_procesamiento.copy()
            puertas_detectadas = []

            if ids is not None and len(ids) > 0:
                # procesamos cada marcador detectado
                for i, c in enumerate(corners):
                    pts = c.reshape((4,2)).astype(int)
                    # dibujar contorno
                    cv2.polylines(img_out, [pts], True, (0,255,0), 2)

                    # calcular tamaño medio en px (lado promedio)
                    side1 = np.linalg.norm(pts[0] - pts[1])
                    side2 = np.linalg.norm(pts[1] - pts[2])
                    side3 = np.linalg.norm(pts[2] - pts[3])
                    side4 = np.linalg.norm(pts[3] - pts[0])
                    side_px = float(np.mean([side1, side2, side3, side4]))

                    # centro del marcador en imagen (px)
                    cx = int(np.mean(pts[:,0])); cy = int(np.mean(pts[:,1]))

                    # Estimación de distancia (m) usando tamaño conocido del marcador y focal aproximada
                    # dist_m = (MARKER_SIZE * FOCAL_PIXELS) / side_px
                    if side_px > 0.0:
                        dist_m = (MARKER_SIZE * FOCAL) / side_px
                    else:
                        dist_m = None

                    # Dibujos en imagen
                    x_min = int(np.min(pts[:,0])); x_max = int(np.max(pts[:,0]))
                    y_min = int(np.min(pts[:,1])); y_max = int(np.max(pts[:,1]))
                    cv2.rectangle(img_out, (x_min, y_min), (x_max, y_max), (255,0,0), 2)
                    cv2.circle(img_out, (cx, cy), 4, (0,0,255), -1)
                    if dist_m is not None:
                        cv2.putText(img_out, f"ID:{ids[i].item()} Dist:{dist_m:.2f}m",(x_min, max(y_min-8,0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

                    # construir "puerta" detectada (ahora marcador)
                    marcador = {
                        'x_centro': cx,
                        'y_centro': cy,
                        'side_px': side_px,
                        'dist_m': dist_m,
                        'id': int(ids[i].item())
                    }
                    puertas_detectadas.append(marcador)

                    # calcular coordenadas en sistema cuerpo y luego mundo usando la pose del dron
                    if dist_m is not None:
                        # convertimos la detección en coordenadas (X,Y,Z) en cuerpo (FRD)
                        # misma lógica que antes usando FOV y posición en imagen
                        fov_horizontal_rad = math.radians(FOV_H)
                        fov_vertical_rad = math.radians(FOV_V)
                        nx = (cx - ANCHO_IMAGEN / 2) / (ANCHO_IMAGEN / 2)
                        ny = -(cy - ALTO_IMAGEN / 2) / (ALTO_IMAGEN / 2)
                        angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
                        angulo_vertical_rad = ny * (fov_vertical_rad / 2)

                        coordenada_Z = -dist_m * math.sin(angulo_vertical_rad)
                        coordenada_Y = dist_m * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
                        coordenada_X = dist_m * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)

                        punto_cuerpo = [coordenada_X, coordenada_Y, coordenada_Z]
                        punto_mundo = self.punto_cuerpo_a_mundo(self.roll, self.pitch, self.yaw, self.pos_dron_mundo, punto_cuerpo)

                        # publicar punto y punto+angulo (para pruebas usemos yaw como angulo)
                        self.distancia_estimada = dist_m
                        self.coordenada_X = coordenada_X
                        self.coordenada_Y = coordenada_Y
                        self.coordenada_Z = coordenada_Z
                        self.punto_mundo = punto_mundo
                        angulo = float(self.yaw)  # no usamos orientación del marcador por ahora
                        self.publicar_punto(punto_mundo)
                        self.publicar_punto_a(punto_mundo[0], punto_mundo[1], punto_mundo[2], angulo)

            # Publicar datos de detectados: cantidad y (x,y,ancho,alto) por cada uno (similar formato antiguo)
            msg_puertas = Float32MultiArray()
            data_to_publish = [float(len(puertas_detectadas))]
            for marcador in puertas_detectadas:
                # para mantener compatibilidad: usaremos side_px como ancho/alto aproximado
                data_to_publish.extend([float(marcador['x_centro']), float(marcador['y_centro']),
                                        float(marcador['side_px']), float(marcador['side_px'])])
            msg_puertas.data = data_to_publish
            self.publicador_puertas_detectadas.publish(msg_puertas)

            # publicar imagen con dibujos
            try:
                img_publish_rgb = cv2.cvtColor(img_out, cv2.COLOR_BGR2RGB)
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="bgr8")
                ros_image_msg_out.header.stamp = msg_imagen_ros.header.stamp
                ros_image_msg_out.header.frame_id = "tello_camera_processed_localization"
                self.publicador_imagen_visualizacion.publish(ros_image_msg_out)
            except CvBridgeError as e_cv_bridge_pub:
                self.get_logger().error(f"Error CvBridge al publicar imagen: {e_cv_bridge_pub}")
            except Exception:
                pass

        except Exception:
            self.get_logger().error("Error en callback_procesamiento_imagen:\n" + traceback.format_exc())

    def publicar_punto(self, punto_mundo):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(punto_mundo[0])
        msg.point.y = float(punto_mundo[1])
        msg.point.z = float(punto_mundo[2])
        self.publisher_.publish(msg)

    def publicar_punto_a(self, x, y , z , angulo):
        msg = Float32MultiArray()
        msg.data = [x, y, z, angulo]
        self.pub_punto_a.publish(msg)

    def punto_cuerpo_a_mundo(self, roll_deg, pitch_deg, yaw_deg, pos_dron_mundo, punto_cuerpo):
        # Transformación como en tu versión anterior (FRD -> mundo)
        roll = np.deg2rad(roll_deg)
        pitch = np.deg2rad(pitch_deg)
        yaw = np.deg2rad(yaw_deg)

        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]
        ])

        R_y = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1]
        ])

        S = np.diag([1, -1, -1])
        R = R_z @ R_y @ R_x @ S

        punto_cuerpo = np.array(punto_cuerpo).reshape(3,)
        pos_dron_mundo = np.array(pos_dron_mundo).reshape(3,)
        punto_mundo = pos_dron_mundo + R @ punto_cuerpo

        return punto_mundo

    def estimar_distancia(self, alto_puerta_px):
        # helper (no usado directamente en ArUco pero lo dejamos)
        distancia_mts = (0.285 * FOCAL) / alto_puerta_px if alto_puerta_px != 0 else None
        return distancia_mts

    def log_datos(self):
        if self.distancia_estimada is not None:
            self.get_logger().info(f"Distancia estimada (ArUco): {self.distancia_estimada:.3f} m")
            if self.punto_mundo is not None:
                self.get_logger().info(f"Punto mundo: {self.punto_mundo}")

    def destroy_node(self):
        self.get_logger().info("Destruyendo ModuloLocalizacion...")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    modulo_localizacion = None
    try:
        modulo_localizacion = ModuloLocalizacion()
        rclpy.spin(modulo_localizacion)
    except KeyboardInterrupt:
        if modulo_localizacion:
            modulo_localizacion.get_logger().info("Ctrl+C detectado, cerrando ModuloLocalizacion.")
    except Exception as e_main:
        if modulo_localizacion:
            modulo_localizacion.get_logger().fatal(f"Error inesperado en main de ModuloLocalizacion: {e_main}")
            modulo_localizacion.get_logger().fatal(traceback.format_exc())
    finally:
        if modulo_localizacion:
            modulo_localizacion.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa ModuloLocalizacion (ArUco) finalizado.")

if __name__ == '__main__':
    main()

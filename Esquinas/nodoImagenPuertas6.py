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

# Definición de tópicos ROS2
ROS_TOPIC_IMAGEN_RAW_INPUT = 'camera/image_raw'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'
ROS_TOPIC_POSE_ANGLES = '/tello/pose_corregida'

# Dimensiones de procesamiento de la imagen
ANCHO_IMAGEN = 640
ALTO_IMAGEN = 480

# Rangos HSV (configurable)
COLOR_MIN = np.array([0, 191, 63])
COLOR_MAX = np.array([10, 255, 142])

# Área mínima de un contorno para ser considerado una esquina
MIN_CORNER_AREA = 100

ALTO_REAL = 0.285
ANCHO_REAL = 0.21
FOCAL = 617.0
FOV_H = 67.2
FOV_V = 52.3

class ModuloLocalizacion(Node):
    def __init__(self):
        super().__init__('modulo_localizacion')
        self.get_logger().info("Iniciando Módulo de Localización (Detección de Puertas).")
        self.bridge = CvBridge()

        # temporizador que se ejecuta cada 3 segundos
        self.timer_log = self.create_timer(3.0, self.log_datos)
        self.distancia_estimada = None
        self.distancia_calculada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None

        self.publisher_ = self.create_publisher(PointStamped, '/punto', 10)
        self.pub_punto_a = self.create_publisher(Float32MultiArray, '/punto_y_angulo', 10)

        # Suscriptor para la imagen RAW
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

        # Suscriptor al tópico /tello/pose_angles (Float32MultiArray)
        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.suscripcion_pose = self.create_subscription(
            Float32MultiArray,
            ROS_TOPIC_POSE_ANGLES,
            self.callback_pose,
            qos_profile_pose
        )
        self.get_logger().info(f"Suscrito a la pose (ángulos) del dron en: {ROS_TOPIC_POSE_ANGLES}")

        # Inicializamos variables de pose
        self.pos_dron_mundo = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Publicadores
        qos_profile_pub_data = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publicador_puertas_detectadas = self.create_publisher(
            Float32MultiArray, ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT, qos_profile_pub_data)
        self.get_logger().info(f"Publicando datos de puertas detectadas en: {ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT}")

        qos_profile_pub_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.publicador_imagen_visualizacion = self.create_publisher(
            Image, ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT, qos_profile_pub_img)
        self.get_logger().info(f"Publicando imagen de visualización en: {ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT}")


    def publicar_punto_a(self, x, y , z , angulo):
        msg = Float32MultiArray()
        msg.data = [x, y, z, angulo]
        self.pub_punto_a.publish(msg)

    def callback_pose(self, msg: Float32MultiArray):
        """
        Espera: msg.data = [x, y, z, roll_deg, pitch_deg, yaw_deg]
        Si no existen los ángulos, se asumen 0.
        """
        try:
            data = list(msg.data)
            if len(data) < 3:
                self.get_logger().warning("Mensaje de pose con datos insuficientes (<3). Ignorando.")
                return

            # Interpretación flexible: si el nodo intermedio publica [x,y,z,roll,pitch,yaw]
            # o [x,y,z] (o el orden que use: aquí esperamos x,y,z en indices 0..2)
            x = float(data[0])
            y = float(data[1])
            z = float(data[2])

            roll = 0.0
            pitch = 0.0
            yaw = 0.0
            if len(data) >= 6:
                roll = float(data[3])
                pitch = float(data[4])
                yaw = float(data[5])
            elif len(data) >= 4:
                # si solo vino 4 valores asumimos que son [x,y,z,yaw]
                yaw = float(data[3])

            self.pos_dron_mundo = [x, y, z]
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw

            #self.get_logger().info(f"Pose recibida: pos={self.pos_dron_mundo}, roll={self.roll}, pitch={self.pitch}, yaw={self.yaw}")

        except Exception as e:
            self.get_logger().error(f"Error procesando pose recibida: {e}")
            self.get_logger().error(traceback.format_exc())

    def callback_procesamiento_imagen(self, msg_imagen_ros):
        try:
            frame_bgr_raw = self.bridge.imgmsg_to_cv2(msg_imagen_ros, desired_encoding="bgr8")
        except CvBridgeError as e_bridge:
            self.get_logger().error(f"Error CvBridge al convertir imagen RAW: {e_bridge}")
            return
        except Exception as e_conversion:
            self.get_logger().error(f"Error general convirtiendo imagen RAW: {e_conversion}")
            return

        if frame_bgr_raw is None:
            self.get_logger().warn("Frame BGR recibido es None después de conversión.", throttle_duration_sec=5.0)
            return

        try:
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_IMAGEN, ALTO_IMAGEN))
            img_proc = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2RGB)
            img_hsv = cv2.cvtColor(img_proc, cv2.COLOR_BGR2HSV)

            puertas_detectadas, img_con_dibujos = self.algoritmoDetectarPuertas(img_hsv, img_procesamiento.copy())

            # Publicar datos de las puertas detectadas
            msg_puertas = Float32MultiArray()
            data_to_publish = [float(len(puertas_detectadas))]
            for puerta in puertas_detectadas:
                data_to_publish.extend([float(puerta['x_centro']), float(puerta['y_centro']),
                                        float(puerta['ancho']), float(puerta['alto'])])
            msg_puertas.data = data_to_publish
            self.publicador_puertas_detectadas.publish(msg_puertas)

            # Publicar imagen visualizada
            try:
                img_publish_rgb = cv2.cvtColor(img_con_dibujos, cv2.COLOR_BGR2RGB)
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="bgr8")
                ros_image_msg_out.header.stamp = msg_imagen_ros.header.stamp
                ros_image_msg_out.header.frame_id = "tello_camera_processed_localization"
                self.publicador_imagen_visualizacion.publish(ros_image_msg_out)
            except CvBridgeError as e_cv_bridge_pub:
                self.get_logger().error(f"Error CvBridge al convertir/publicar imagen de visualización: {e_cv_bridge_pub}")
            except Exception as e_publish_general:
                self.get_logger().error(f"Error general al publicar visualización de puertas: {e_publish_general}")

        except Exception as e_processing_callback:
            self.get_logger().error(f"Error general en procesamiento del callback_procesamiento_imagen: {e_processing_callback}")
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
        distancia_mts = (ALTO_REAL * FOCAL) / alto_puerta_px
        return distancia_mts

    def punto_cuerpo_a_mundo(self, roll_deg, pitch_deg, yaw_deg, pos_dron_mundo, punto_cuerpo):
        # Igual que antes: transforma punto del cuerpo (FRD) a sistema mundo (coincide con ejes que usabas)
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

    def algoritmoDetectarPuertas(self, img_hsv, img_visualizacion):
        puntos_esquinas_detectados = []
        mask_azul = cv2.inRange(img_hsv, COLOR_MIN, COLOR_MAX)
        kernel = np.ones((5,5), np.uint8)
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, kernel)
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, kernel)
        contornos, _ = cv2.findContours(mask_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > MIN_CORNER_AREA:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    puntos_esquinas_detectados.append((cx, cy))
                    cv2.circle(img_visualizacion, (cx, cy), 7, (255, 0, 0), -1)
                    cv2.putText(img_visualizacion, f"({cx},{cy})", (cx + 10, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        puertas_encontradas = self.tratarPuerta(puntos_esquinas_detectados, img_visualizacion)
        return puertas_encontradas, img_visualizacion

    def tratarPuerta(self, puntos_esquinas, img_visualizacion):
        puertas = []

        # RECIBIR DATOS DE POSE desde self.pos_dron_mundo, self.roll, self.pitch, self.yaw
        roll = self.roll
        pitch = self.pitch
        yaw = self.yaw
        pos_dron_mundo = self.pos_dron_mundo

        if len(puntos_esquinas) >= 4:
            x_sorted = sorted(puntos_esquinas, key=lambda p: p[0])

            x_menor = x_sorted[0]
            x_menor2 = x_sorted[1]
            x_menor3 = x_sorted[2]
            x_menor4 = x_sorted[3]

            if (x_menor[1] > x_menor2[1]):
                esq1 = x_menor
                esq4 = x_menor2
            else:
                esq1 = x_menor2
                esq4 = x_menor

            if (x_menor3[1] > x_menor4[1]):
                esq2 = x_menor3
                esq3 = x_menor4
            else:
                esq2 = x_menor4
                esq3 = x_menor3

            centro_imagen = ANCHO_IMAGEN/2
            ancho_puerta = math.sqrt((esq2[0] - esq1[0])**2 + (esq2[1] - esq1[1])**2)
            alto_puerta = math.sqrt((esq1[0] - esq4[0])**2 + (esq1[1] - esq4[1])**2)
            alto_izq = alto_puerta
            alto_dcha = math.sqrt((esq2[0] - esq3[0])**2 + (esq2[1] - esq3[1])**2)

            if ancho_puerta > alto_puerta:
                x = ancho_puerta
                ancho_puerta = alto_puerta
                alto_puerta = x

            proporcion = ancho_puerta / alto_puerta
            proporcion_real = ANCHO_REAL / ALTO_REAL
            angulo_prop = 90 -(90*proporcion/proporcion_real)

            x_centro_puerta = esq1[0] + ancho_puerta // 2
            y_centro_puerta = esq4[1] + alto_puerta // 2

            distancia_estimada = self.estimar_distancia(alto_puerta)

            if centro_imagen > x_centro_puerta and (alto_izq > alto_dcha):
                angulo = yaw - angulo_prop
                caso = 1
            elif centro_imagen > x_centro_puerta and (alto_izq < alto_dcha):
                angulo = yaw + angulo_prop
                caso = 2
            elif centro_imagen < x_centro_puerta and (alto_izq < alto_dcha):
                angulo = yaw - angulo_prop
                caso = 3
            elif centro_imagen < x_centro_puerta and (alto_izq > alto_dcha):
                angulo = yaw + angulo_prop
                caso = 4
            else:
                angulo = angulo_prop
                caso = 0

            x_centro_puerta = int(x_centro_puerta)
            y_centro_puerta = int(y_centro_puerta)

            fov_horizontal_rad = math.radians(FOV_H)
            fov_vertical_rad = math.radians(FOV_V)
            nx = (x_centro_puerta - ANCHO_IMAGEN / 2) / (ANCHO_IMAGEN / 2)
            ny = -(y_centro_puerta - ALTO_IMAGEN / 2) / (ALTO_IMAGEN / 2)

            angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
            angulo_vertical_rad = ny * (fov_vertical_rad / 2)

            coordenada_Z = -distancia_estimada * math.sin(angulo_vertical_rad)
            coordenada_Y = distancia_estimada * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
            coordenada_X = distancia_estimada * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)

            distancia_calculada = math.sqrt(coordenada_X**2 + coordenada_Y**2 + coordenada_Z**2)

            punto_cuerpo = [coordenada_X, coordenada_Y, coordenada_Z]
            punto_mundo = self.punto_cuerpo_a_mundo(roll, pitch, yaw, pos_dron_mundo, punto_cuerpo)

            self.distancia_estimada = distancia_estimada
            self.distancia_calculada = distancia_calculada
            self.coordenada_X = coordenada_X
            self.coordenada_Y = coordenada_Y
            self.coordenada_Z = coordenada_Z
            self.punto_mundo = punto_mundo

            puerta_detectada = {
                'x_centro': x_centro_puerta,
                'y_centro': y_centro_puerta,
                'ancho': ancho_puerta,
                'alto': alto_puerta
            }
            puertas.append(puerta_detectada)
            self.publicar_punto(punto_mundo)
            self.publicar_punto_a(punto_mundo[0], punto_mundo[1], punto_mundo[2], angulo)

            esquinas = [esq1, esq2, esq3, esq4]
            puntos = np.array(esquinas, np.int32)
            puntos = puntos.reshape((-1, 1, 2))
            cv2.polylines(img_visualizacion, [puntos], True, (0, 255, 0), 2)
            cv2.circle(img_visualizacion, (x_centro_puerta, y_centro_puerta), 5, (0, 0, 255), -1)
            cv2.putText(img_visualizacion, f"Puerta ({x_centro_puerta},{y_centro_puerta},{angulo:.1f})",
                            (x_centro_puerta - 50, y_centro_puerta - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        return puertas

    def log_datos(self):
        if self.distancia_estimada is not None:
            self.get_logger().info("--- DATOS ---")
            self.get_logger().info(f"Distancia a la puerta: {self.distancia_estimada:.2f} m")
            self.get_logger().info(f"Verificación: La distancia calculada es {self.distancia_calculada:.3f} m "
                                    f"(debería ser {self.distancia_estimada:.2f} m)")
            self.get_logger().info(f"Coordenada puerta: {self.coordenada_X:.2f} , "
                                    f"{self.coordenada_Y:.2f} , {self.coordenada_Z:.2f} m")
            self.get_logger().info(f"Coordenadas del punto en mundo: {self.punto_mundo}")
            self.get_logger().info("-----------------------------")

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
        print("Programa ModuloLocalizacion finalizado.")

if __name__ == '__main__':
    main()

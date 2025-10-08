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
from geometry_msgs.msg import PointStamped

# ----------------- CONFIGURACIÓN RÁPIDA -----------------
# Indica si el punto que viene de la detección está en 'FRD' o 'FLD'.
# Si tu detección ya entrega x adelante, y izquierda, z abajo -> usa 'FLD' (valor por defecto).
INPUT_POINT_FRAME = 'FLD'   # 'FLD' o 'FRD'

# Si tu pose (roll,pitch,yaw) viene en la convención world->body en lugar de body->world,
# prueba poner True (usará la transpuesta de R). Por defecto False.
ROTATION_USE_TRANSPOSE = False

# Si la componente Z del punto detectado parece invertida (ej. bajar dron hace subir punto),
# pon True para invertir la Z calculada desde la imagen.
INVERT_POINT_Z = False
# -------------------------------------------------------

# Tópicos
ROS_TOPIC_IMAGEN_RAW_INPUT = 'tello/imagen'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'
ROS_TOPIC_POSE_DRON = '/tello/pose_corregida'
ROS_TOPIC_PUNTO = '/punto'
ROS_TOPIC_PUNTO_ANG = '/punto_y_angulo'

# Dimensiones de procesamiento
ANCHO_IMAGEN = 960
ALTO_IMAGEN = 720

# ArUco params
ARUCO_DICT = cv2.aruco.DICT_5X5_250
MARKER_SIZE = 0.175
FOCAL_PIXELS = 617.0
FOCAL = FOCAL_PIXELS
FOV_H = 67.2
FOV_V = 52.3

class ModuloLocalizacion(Node):
    def __init__(self):
        super().__init__('modulo_localizacion_aruco')
        self.get_logger().info("Iniciando Módulo de Localización (ArUco) - modo simple (puntos en FLD).")
        self.bridge = CvBridge()

        # Pose del dron (mundo: +x adelante, +y izquierda, +z abajo) - la pose que recibes debe estar en FLD
        self.pos_dron_mundo = [0.0, 0.0, 0.0]
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0

        # Salidas / estado
        self.distancia_estimada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None

        # publicadores / subscriptores
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

        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except Exception:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.timer_log = self.create_timer(3.0, self.log_datos)

    def callback_pose(self, msg: Float32MultiArray):
        """
        Espera: msg.data = [x, y, z, roll_deg, pitch_deg, yaw_deg]
        La pose debe venir en FLD y los ángulos en grados.
        """
        try:
            data = list(msg.data)
            if len(data) < 3:
                return
            x = float(data[0]); y = float(data[1]); z = float(data[2])
            roll = float(data[3]) if len(data) >= 4 else 0.0
            pitch = float(data[4]) if len(data) >= 5 else 0.0
            yaw = float(data[5]) if len(data) >= 6 else 0.0
            self.pos_dron_mundo = [x, y, z]
            self.roll = roll; self.pitch = pitch; self.yaw = yaw
        except Exception:
            self.get_logger().warning("Error al procesar mensaje de pose. Ignorando.")

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
                for i, c in enumerate(corners):
                    pts = c.reshape((4,2)).astype(int)
                    cv2.polylines(img_out, [pts], True, (0,255,0), 2)

                    side1 = np.linalg.norm(pts[0] - pts[1])
                    side2 = np.linalg.norm(pts[1] - pts[2])
                    side3 = np.linalg.norm(pts[2] - pts[3])
                    side4 = np.linalg.norm(pts[3] - pts[0])
                    side_px = float(np.mean([side1, side2, side3, side4]))

                    cx = int(np.mean(pts[:,0])); cy = int(np.mean(pts[:,1]))

                    if side_px > 0.0:
                        dist_m = (MARKER_SIZE * FOCAL) / side_px
                    else:
                        dist_m = None

                    x_min = int(np.min(pts[:,0])); x_max = int(np.max(pts[:,0]))
                    y_min = int(np.min(pts[:,1])); y_max = int(np.max(pts[:,1]))
                    cv2.rectangle(img_out, (x_min, y_min), (x_max, y_max), (255,0,0), 2)
                    cv2.circle(img_out, (cx, cy), 4, (0,0,255), -1)
                    if dist_m is not None:
                        cv2.putText(img_out, f"ID:{ids[i].item()} Dist:{dist_m:.2f}m",
                                    (x_min, max(y_min-8,0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

                    marcador = {
                        'x_centro': cx,
                        'y_centro': cy,
                        'side_px': side_px,
                        'dist_m': dist_m,
                        'id': int(ids[i].item())
                    }
                    puertas_detectadas.append(marcador)

                    # cálculo coordenadas en cuerpo (según aproximación FOV -> punto relativo)
                    if dist_m is not None:
                        fov_horizontal_rad = math.radians(FOV_H)
                        fov_vertical_rad = math.radians(FOV_V)
                        nx = (cx - ANCHO_IMAGEN / 2) / (ANCHO_IMAGEN / 2)
                        ny = -(cy - ALTO_IMAGEN / 2) / (ALTO_IMAGEN / 2)
                        angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
                        angulo_vertical_rad = ny * (fov_vertical_rad / 2)

                        # Si tu detección produce valores en FRD cambialo en la constante INPUT_POINT_FRAME.
                        # Aquí calculo según la lógica previa (puedes invertir el signo de Z si lo necesitas).
                        z_sign = -1.0 if not INVERT_POINT_Z else 1.0
                        coordenada_Z = z_sign * dist_m * math.sin(angulo_vertical_rad)
                        coordenada_Y = dist_m * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
                        coordenada_X = dist_m * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)

                        # Forma el punto tal y como sale de la detección:
                        # si INPUT_POINT_FRAME == 'FRD' -> el punto viene (x forward, y right, z down)
                        # si INPUT_POINT_FRAME == 'FLD' -> el punto viene (x forward, y left, z down)
                        if INPUT_POINT_FRAME == 'FRD':
                            punto_detectado = np.array([coordenada_X, coordenada_Y, coordenada_Z], dtype=float)
                        else:  # 'FLD'
                            # convertimos la Y para que represente left: detection already gives left -> use directly
                            # (si tu detector ya da y izquierda, ya estás en FLD).
                            punto_detectado = np.array([coordenada_X, coordenada_Y, coordenada_Z], dtype=float)

                        # TRANSFORMACIÓN simple: punto_detectado está en cuerpo (FLD si INPUT_POINT_FRAME=='FLD')
                        punto_mundo = self.transform_point_body_to_world(punto_detectado,
                                                                         self.pos_dron_mundo,
                                                                         (self.roll, self.pitch, self.yaw))

                        # publicar y guardar
                        self.distancia_estimada = dist_m
                        self.coordenada_X = coordenada_X
                        self.coordenada_Y = coordenada_Y
                        self.coordenada_Z = coordenada_Z
                        self.punto_mundo = punto_mundo
                        angulo = float(self.yaw)
                        self.publicar_punto(punto_mundo)
                        self.publicar_punto_a(float(punto_mundo[0]), float(punto_mundo[1]), float(punto_mundo[2]), angulo)

            # publicar lista de detectados
            msg_puertas = Float32MultiArray()
            data_to_publish = [float(len(puertas_detectadas))]
            for marcador in puertas_detectadas:
                data_to_publish.extend([float(marcador['x_centro']), float(marcador['y_centro']),
                                        float(marcador['side_px']), float(marcador['side_px'])])
            msg_puertas.data = data_to_publish
            self.publicador_puertas_detectadas.publish(msg_puertas)

            # publicar imagen procesada
            try:
                img_publish_rgb = cv2.cvtColor(img_out, cv2.COLOR_BGR2RGB)
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="rgb8")
                ros_image_msg_out.header.stamp = msg_imagen_ros.header.stamp
                ros_image_msg_out.header.frame_id = "tello_camera_processed_localization"
                self.publicador_imagen_visualizacion.publish(ros_image_msg_out)
            except CvBridgeError as e_cv_bridge_pub:
                self.get_logger().error(f"Error CvBridge al publicar imagen: {e_cv_bridge_pub}")
            except Exception:
                self.get_logger().warning("Error no crítico al publicar imagen procesada.")

        except Exception:
            self.get_logger().error("Error en callback_procesamiento_imagen:\n" + traceback.format_exc())

    def transform_point_body_to_world(self, point_body, drone_pos, angles_deg):
        """
        punto_body: numpy array (3,) con coordenadas en el sistema del cuerpo del dron.
                    Se asume que está en FLD si INPUT_POINT_FRAME == 'FLD'.
        drone_pos: [x,y,z] en mundo (FLD)
        angles_deg: (roll, pitch, yaw) en grados, convención ZYX (yaw,pitch,roll)

        Devuelve punto en mundo (numpy array 3).
        """
        p_body = np.asarray(point_body, dtype=float).reshape(3,)

        # Si punto venía en FRD y quieres convertir a FLD, se haría aquí.
        if INPUT_POINT_FRAME == 'FRD':
            S_fr_to_fld = np.diag([1.0, -1.0, 1.0])
            p_body_fld = S_fr_to_fld @ p_body
        else:
            p_body_fld = p_body  # ya está en FLD

        # Construir R_bodyFLD_to_worldFLD usando ZYX (R = Rz * Ry * Rx)
        roll, pitch, yaw = np.deg2rad(angles_deg)
        cr = math.cos(roll); sr = math.sin(roll)
        cp = math.cos(pitch); sp = math.sin(pitch)
        cy = math.cos(yaw); sy = math.sin(yaw)

        R_x = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr, cr]])
        R_y = np.array([[cp, 0, sp],
                        [0, 1, 0],
                        [-sp, 0, cp]])
        R_z = np.array([[cy, -sy, 0],
                        [sy, cy, 0],
                        [0, 0, 1]])

        R_bodyFLD_to_worldFLD = R_z @ R_y @ R_x

        # Si por convención la pose que recibes es la inversa (world->body), usa la transpuesta:
        if ROTATION_USE_TRANSPOSE:
            R_bodyFLD_to_worldFLD = R_bodyFLD_to_worldFLD.T

        t_w = np.asarray(drone_pos, dtype=float).reshape(3,)
        p_w = t_w + R_bodyFLD_to_worldFLD @ p_body_fld
        return p_w

    def publicar_punto(self, punto_mundo):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(punto_mundo[0]); msg.point.y = float(punto_mundo[1]); msg.point.z = float(punto_mundo[2])
        self.publisher_.publish(msg)

    def publicar_punto_a(self, x, y , z , angulo):
        msg = Float32MultiArray(); msg.data = [x, y, z, angulo]; self.pub_punto_a.publish(msg)

    def estimar_distancia(self, alto_puerta_px):
        return (0.175 * FOCAL) / alto_puerta_px if alto_puerta_px != 0 else None

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
        print("Programa finalizado.")

if __name__ == '__main__':
    main()

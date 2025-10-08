# nodoArucoDeteccion.py (versión actualizada con calibración y pose estimate)
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
ROS_TOPIC_IMAGEN_RAW_INPUT = 'tello/imagen'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'
ROS_TOPIC_POSE_DRON = '/tello/pose'
ROS_TOPIC_PUNTO = '/punto'
ROS_TOPIC_PUNTO_ANG = '/punto_y_angulo'

# Dimensiones de procesamiento -> ajustadas a la calibración que has proporcionado
ANCHO_IMAGEN = 960
ALTO_IMAGEN = 720

# ArUco params (ajusta MARKER_SIZE si hace falta)
ARUCO_DICT = cv2.aruco.DICT_5X5_250
MARKER_SIZE = 0.175     # tamaño del marcador en metros (17.5 cm ejemplo)
# (NOTA: ya no usamos FOCAL_PIXELS para la estimación si usamos pose estimate; lo dejamos por compatibilidad)
FOCAL_PIXELS = 617.0
FOCAL = FOCAL_PIXELS
FOV_H = 67.2
FOV_V = 52.3

# --- Parámetros de calibración (tu cámara) ---
CAM_FX = 900.1766
CAM_FY = 894.8176
CAM_CX = 481.5253
CAM_CY = 371.0677

CAM_K1 = 0.089014
CAM_K2 = -1.546625
CAM_P1 = 0.002167
CAM_P2 = 0.004498
CAM_K3 = 6.561574

# Suavizado para reducir saltos (opcional)
SMOOTHING_ENABLED = True
SMOOTH_ALPHA = 0.6  # 0=no suavizado (usar valor cercano a 1 para respuesta rápida)

class ModuloLocalizacion(Node):
    def __init__(self):
        super().__init__('modulo_localizacion_aruco')
        self.get_logger().info("Iniciando Módulo de Localización (ArUco) - versión calibrada.")
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

        # para suavizado (coordenadas en ejes cuerpo)
        self._prev_punto_cuerpo = None

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
        # DetectorParameters (compatibilidad con distintas versiones)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Matriz intrínseca y distorsión (según calibración proporcionada)
        self.camera_matrix = np.array([[CAM_FX, 0.0, CAM_CX],
                                       [0.0, CAM_FY, CAM_CY],
                                       [0.0, 0.0, 1.0]], dtype=np.float64)
        self.dist_coeffs = np.array([CAM_K1, CAM_K2, CAM_P1, CAM_P2, CAM_K3], dtype=np.float64)

        # timer informativo opcional
        self.timer_log = self.create_timer(3.0, self.log_datos)

    def callback_pose(self, msg: Float32MultiArray):
        """
        Espera: msg.data = [x, y, z, roll_deg, pitch_deg, yaw_deg]
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
            # Redimensionamos a la resolución de calibración (si hace falta)
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_IMAGEN, ALTO_IMAGEN))
            img_gray = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2GRAY)

            # Detectar markers ArUco
            corners, ids, rejected = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

            img_out = img_procesamiento.copy()
            puertas_detectadas = []

            # Intentar estimar pose mediante OpenCV si hay detecciones
            pose_rvecs = None
            pose_tvecs = None
            if ids is not None and len(ids) > 0:
                try:
                    # corners tiene forma (N,1,4,2) o lista; estimatePoseSingleMarkers acepta corners
                    pose_rvecs, pose_tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, self.camera_matrix, self.dist_coeffs)
                    # pose_tvecs: (N,1,3) -> vector traslación en coordenadas de cámara (x right, y down, z forward)
                except Exception:
                    pose_rvecs, pose_tvecs = None, None

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

                    # Si tenemos pose estimada con calibración, la usamos
                    if pose_tvecs is not None:
                        tvec_cam = pose_tvecs[i].reshape(3,)  # [x_cam, y_cam, z_cam] en sistema cámara (x right, y down, z forward)
                        # distancia real (norma)
                        dist_m = float(np.linalg.norm(tvec_cam))
                        # Convertir de coordenadas cámara -> cuerpo (tu convención: +X adelante, +Y derecha, +Z abajo)
                        # OpenCV camera: x_right, y_down, z_forward
                        coordenada_X = float(tvec_cam[2])   # z_cam -> X_body (adelante)
                        coordenada_Y = float(tvec_cam[0])   # x_cam -> Y_body (derecha)
                        coordenada_Z = float(tvec_cam[1])   # y_cam -> Z_body (abajo)
                    else:
                        # Fallback: estimación por tamaño y FOV (como en versión original)
                        if side_px > 0.0:
                            dist_m = (MARKER_SIZE * FOCAL) / side_px
                        else:
                            dist_m = None

                        if dist_m is not None:
                            # convertimos la detección en coordenadas (X,Y,Z) en cuerpo (FRD) usando FOV aproximado
                            fov_horizontal_rad = math.radians(FOV_H)
                            fov_vertical_rad = math.radians(FOV_V)
                            nx = (cx - ANCHO_IMAGEN / 2) / (ANCHO_IMAGEN / 2)
                            ny = -(cy - ALTO_IMAGEN / 2) / (ALTO_IMAGEN / 2)
                            angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
                            angulo_vertical_rad = ny * (fov_vertical_rad / 2)

                            coordenada_Z = -dist_m * math.sin(angulo_vertical_rad)
                            coordenada_Y = dist_m * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
                            coordenada_X = dist_m * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)
                        else:
                            coordenada_X = None; coordenada_Y = None; coordenada_Z = None

                    # Dibujos en imagen
                    x_min = int(np.min(pts[:,0])); x_max = int(np.max(pts[:,0]))
                    y_min = int(np.min(pts[:,1])); y_max = int(np.max(pts[:,1]))
                    cv2.rectangle(img_out, (x_min, y_min), (x_max, y_max), (255,0,0), 2)
                    cv2.circle(img_out, (cx, cy), 4, (0,0,255), -1)
                    if dist_m is not None:
                        cv2.putText(img_out, f"ID:{ids[i].item()} Dist:{dist_m:.2f}m",(x_min, max(y_min-8,0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

                    marcador = {
                        'x_centro': cx,
                        'y_centro': cy,
                        'side_px': side_px,
                        'dist_m': dist_m,
                        'id': int(ids[i].item())
                    }
                    puertas_detectadas.append(marcador)

                    # construir punto en ejes cuerpo
                    if coordenada_X is not None and coordenada_Y is not None and coordenada_Z is not None:
                        punto_cuerpo = [coordenada_X, coordenada_Y, coordenada_Z]

                        # Suavizado simple (exponencial) para reducir saltos (opcional)
                        if SMOOTHING_ENABLED:
                            if self._prev_punto_cuerpo is None:
                                smooth_p = np.array(punto_cuerpo, dtype=float)
                            else:
                                smooth_p = SMOOTH_ALPHA * np.array(punto_cuerpo, dtype=float) + (1.0 - SMOOTH_ALPHA) * np.array(self._prev_punto_cuerpo, dtype=float)
                            self._prev_punto_cuerpo = smooth_p
                            punto_cuerpo_pub = smooth_p.tolist()
                        else:
                            punto_cuerpo_pub = punto_cuerpo
                            self._prev_punto_cuerpo = punto_cuerpo

                        # Transformar a mundo usando la pose del dron
                        punto_mundo = self.punto_cuerpo_a_mundo(self.roll, self.pitch, self.yaw, self.pos_dron_mundo, punto_cuerpo_pub)

                        # actualizar variables internas
                        self.distancia_estimada = dist_m
                        self.coordenada_X = float(punto_cuerpo_pub[0])
                        self.coordenada_Y = float(punto_cuerpo_pub[1])
                        self.coordenada_Z = float(punto_cuerpo_pub[2])
                        self.punto_mundo = punto_mundo
                        angulo = float(self.yaw)  # no usamos orientación del marcador por ahora

                        # Publicar: (mantengo la llamada como en tu versión original)
                        # publicar_punto recibe un vector de 3 elementos; el código original la llamaba con punto_cuerpo
                        self.publicar_punto(punto_mundo)
                        self.publicar_punto_a(punto_mundo[0], punto_mundo[1], punto_mundo[2], angulo)

            # Publicar datos de detectados: cantidad y (x,y,ancho,alto) por cada uno
            msg_puertas = Float32MultiArray()
            data_to_publish = [float(len(puertas_detectadas))]
            for marcador in puertas_detectadas:
                data_to_publish.extend([float(marcador['x_centro']), float(marcador['y_centro']),
                                        float(marcador['side_px']), float(marcador['side_px'])])
            msg_puertas.data = data_to_publish
            self.publicador_puertas_detectadas.publish(msg_puertas)

            # publicar imagen con dibujos (observa que la conversión de color y encoding se mantiene como antes)
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
        # Nota: la función mantiene el nombre original. En la versión original se
        # llamaba con punto_cuerpo para depuración; mantengo comportamiento similar.
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
        # Transformación: FRD(body) -> mundo
        # roll,pitch,yaw en grados (asumimos misma convención que antes)
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

        # Ahora S debe convertir de ejes BODY (+X adelante, +Y derecha, +Z abajo)
        # a ejes MUNDO (+X adelante, +Y izquierda, +Z abajo).
        # Por tanto invertimos Y solamente:
        S = np.diag([1, -1, 1])

        # Composición de rotaciones: R = Rz * Ry * Rx * S
        R = R_z @ R_y @ R_x @ S

        punto_cuerpo = np.array(punto_cuerpo).reshape(3,)
        pos_dron_mundo = np.array(pos_dron_mundo).reshape(3,)
        punto_mundo = pos_dron_mundo + R @ punto_cuerpo

        return punto_mundo

    def estimar_distancia(self, alto_puerta_px):
        # helper (no usado directamente en ArUco pero lo dejamos)
        distancia_mts = (0.175 * FOCAL) / alto_puerta_px if alto_puerta_px != 0 else None
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
        print("Programa finalizado.")

if __name__ == '__main__':
    main()

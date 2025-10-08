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
from collections import deque
from geometry_msgs.msg import PointStamped

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

# Parámetros del detector automático de signo
HISTORY_LEN = 12         # muestras en buffer
CORR_THRESHOLD = 0.65    # umbral correlación
MIN_MOV = 0.01           # movimiento mínimo (m) para considerar la muestra

class ModuloLocalizacion(Node):
    def __init__(self):
        super().__init__('modulo_localizacion_aruco')
        self.get_logger().info("Iniciando Módulo de Localización (ArUco).")
        self.bridge = CvBridge()

        # Pose del dron (mundo: +x adelante, +y izquierda, +z abajo según tu convención final)
        self.pos_dron_mundo = [0.0, 0.0, 0.0]
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0

        # flip control (None => auto, True/False => forzar)
        self.force_flip_y = None
        self.force_flip_z = None
        self.auto_flip_enabled = True

        # historial para auto-detección (separado por eje)
        self._hist_y = deque(maxlen=HISTORY_LEN)  # (drone_y, punto_y)
        self._hist_z = deque(maxlen=HISTORY_LEN)  # (drone_z, punto_z)

        # flags decididas por auto-detector
        self._decided_flip_y = False
        self._decided_flip_z = True

        # outputs
        self.distancia_estimada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None

        # publishers/subscribers
        self.publisher_ = self.create_publisher(PointStamped, ROS_TOPIC_PUNTO, 10)
        self.pub_punto_a = self.create_publisher(Float32MultiArray, ROS_TOPIC_PUNTO_ANG, 10)

        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST, depth=1)
        self.suscripcion_imagen = self.create_subscription(Image, ROS_TOPIC_IMAGEN_RAW_INPUT,
                                                          self.callback_procesamiento_imagen, qos_profile_sub)
        self.get_logger().info(f"Suscrito a imagen RAW en: {ROS_TOPIC_IMAGEN_RAW_INPUT}")

        qos_profile_pose = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                      history=HistoryPolicy.KEEP_LAST, depth=10)
        self.suscripcion_pose = self.create_subscription(Float32MultiArray, ROS_TOPIC_POSE_DRON,
                                                         self.callback_pose, qos_profile_pose)
        self.get_logger().info(f"Suscrito a la pose del dron en: {ROS_TOPIC_POSE_DRON}")

        qos_profile_pub_data = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.publicador_puertas_detectadas = self.create_publisher(Float32MultiArray,
                                                                   ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT,
                                                                   qos_profile_pub_data)
        qos_profile_pub_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                        history=HistoryPolicy.KEEP_LAST, depth=5)
        self.publicador_imagen_visualizacion = self.create_publisher(Image,
                                                                     ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT,
                                                                     qos_profile_pub_img)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except Exception:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.timer_log = self.create_timer(3.0, self.log_datos)

    def callback_pose(self, msg: Float32MultiArray):
        """
        msg.data = [x, y, z, roll_deg, pitch_deg, yaw_deg]
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
        except Exception:
            return

        if frame_bgr_raw is None:
            return

        try:
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_IMAGEN, ALTO_IMAGEN))
            img_gray = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

            img_out = img_procesamiento.copy()
            puertas_detectadas = []

            # rotación (usa la convención transpuesta por defecto, como detectamos anteriormente)
            R_trans = self.rotation_matrix_frd_to_world((self.roll, self.pitch, self.yaw), invert=True)

            # t original y candidatos flip
            t_orig = np.asarray(self.pos_dron_mundo, dtype=float).reshape(3,)
            t_flip_y = np.array([t_orig[0], -t_orig[1], t_orig[2]], dtype=float)
            t_flip_z = np.array([t_orig[0], t_orig[1], -t_orig[2]], dtype=float)
            t_flip_yz = np.array([t_orig[0], -t_orig[1], -t_orig[2]], dtype=float)

            # decisión sobre flips: prioridad forzado > auto-decisión
            decided_y = None
            decided_z = None
            if self.force_flip_y is True:
                decided_y = True
            elif self.force_flip_y is False:
                decided_y = False
            else:
                decided_y = getattr(self, '_decided_flip_y', False)

            if self.force_flip_z is True:
                decided_z = True
            elif self.force_flip_z is False:
                decided_z = False
            else:
                decided_z = getattr(self, '_decided_flip_z', False)

            # elegir t_w según decisiones
            if decided_y and decided_z:
                t_w_use = t_flip_yz
            elif decided_y and not decided_z:
                t_w_use = t_flip_y
            elif (not decided_y) and decided_z:
                t_w_use = t_flip_z
            else:
                t_w_use = t_orig

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

                    # cálculo coordenadas en cuerpo FRD
                    if dist_m is not None:
                        fov_horizontal_rad = math.radians(FOV_H)
                        fov_vertical_rad = math.radians(FOV_V)
                        nx = (cx - ANCHO_IMAGEN / 2) / (ANCHO_IMAGEN / 2)
                        ny = -(cy - ALTO_IMAGEN / 2) / (ALTO_IMAGEN / 2)
                        angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
                        angulo_vertical_rad = ny * (fov_vertical_rad / 2)

                        coordenada_Z = -dist_m * math.sin(angulo_vertical_rad)
                        coordenada_Y = dist_m * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
                        coordenada_X = dist_m * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)

                        punto_fr = np.array([coordenada_X, coordenada_Y, coordenada_Z], dtype=float)

                        # candidatos de punto mundo según flips de t_w
                        p_no_flip = t_orig + R_trans.dot(punto_fr)
                        p_flip_y = t_flip_y + R_trans.dot(punto_fr)
                        p_flip_z = t_flip_z + R_trans.dot(punto_fr)
                        p_flip_yz = t_flip_yz + R_trans.dot(punto_fr)
                        

                        # actualizar historiales para auto-detección (si está habilitado y no forzado)
                        if self.auto_flip_enabled:
                            # Y
                            if self.force_flip_y is None:
                                self._hist_y.append((float(t_orig[1]), float(p_no_flip[1])))
                                self._auto_decide_flip('y')
                            # Z
                            if self.force_flip_z is None:
                                self._hist_z.append((float(t_orig[2]), float(p_no_flip[2])))
                                self._auto_decide_flip('z')

                            # recomputar decided flips post-decision
                            decided_y = getattr(self, '_decided_flip_y', False) if self.force_flip_y is None else self.force_flip_y
                            decided_z = getattr(self, '_decided_flip_z', False) if self.force_flip_z is None else self.force_flip_z

                            # recompute chosen candidate t_w
                            if decided_y and decided_z:
                                punto_mundo = p_flip_yz
                            elif decided_y and not decided_z:
                                punto_mundo = p_flip_y
                            elif (not decided_y) and decided_z:
                                punto_mundo = p_flip_z
                            else:
                                punto_mundo = p_no_flip
                        else:
                            # no auto, usar current decisions
                            if decided_y and decided_z:
                                punto_mundo = p_flip_yz
                            elif decided_y and not decided_z:
                                punto_mundo = p_flip_y
                            elif (not decided_y) and decided_z:
                                punto_mundo = p_flip_z
                            else:
                                punto_mundo = p_no_flip

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

            # publicar imagen
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

    def _auto_decide_flip(self, axis: str):
        """
        axis in {'y','z'}
        analiza historial correspondiente y decide flip si corr > threshold
        """
        if axis == 'y':
            hist = self._hist_y
            min_move = MIN_MOV
        else:
            hist = self._hist_z
            min_move = MIN_MOV

        if len(hist) < 4:
            return

        arr_a = np.array([h[0] for h in hist], dtype=float)
        arr_b = np.array([h[1] for h in hist], dtype=float)
        da = np.diff(arr_a)
        db = np.diff(arr_b)
        mask = np.abs(da) >= min_move
        if np.count_nonzero(mask) < 3:
            return
        da_f = da[mask]; db_f = db[mask]
        if np.std(da_f) == 0 or np.std(db_f) == 0:
            return
        corr = np.corrcoef(da_f, db_f)[0,1]
        if math.isnan(corr):
            return
        self.get_logger().debug(f"Auto-flip ({axis}): corr={corr:.3f} samples={len(da_f)}")
        if corr > CORR_THRESHOLD:
            # si corr positiva alta -> el punto se mueve en la misma dirección que el dron => flip necesario
            if axis == 'y':
                self._decided_flip_y = True
                self.get_logger().info("Auto-detección: flip Y = True (invirtiendo signo Y del dron).")
            else:
                self._decided_flip_z = True
                self.get_logger().info("Auto-detección: flip Z = True (invirtiendo signo Z del dron).")
        elif corr < -CORR_THRESHOLD:
            if axis == 'y':
                self._decided_flip_y = False
                self.get_logger().info("Auto-detección: flip Y = False (signo Y correcto).")
            else:
                self._decided_flip_z = False
                self.get_logger().info("Auto-detección: flip Z = False (signo Z correcto).")
        else:
            # ambíguo: no cambiar
            return

    def publicar_punto(self, punto_mundo):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(punto_mundo[0]); msg.point.y = float(punto_mundo[1]); msg.point.z = float(punto_mundo[2])
        self.publisher_.publish(msg)

    def publicar_punto_a(self, x, y , z , angulo):
        msg = Float32MultiArray(); msg.data = [x, y, z, angulo]; self.pub_punto_a.publish(msg)

    def rotation_matrix_frd_to_world(self, angles_deg, invert=False):
        roll, pitch, yaw = np.deg2rad(angles_deg)
        cr = math.cos(roll); sr = math.sin(roll)
        cp = math.cos(pitch); sp = math.sin(pitch)
        cy = math.cos(yaw); sy = math.sin(yaw)

        R_x = np.array([[1,  0,   0],
                        [0, cr, -sr],
                        [0, sr,  cr]])
        R_y = np.array([[ cp, 0, sp],
                        [  0, 1,  0],
                        [-sp, 0, cp]])
        R_z = np.array([[cy, -sy, 0],
                        [sy,  cy, 0],
                        [ 0,   0, 1]])
        R_bodyFLU_to_worldFLU = R_z @ R_y @ R_x
        if invert:
            R_bodyFLU_to_worldFLU = R_bodyFLU_to_worldFLU.T
        S1 = np.diag([1.0, -1.0, -1.0])   # FRD -> FLU
        S2 = np.diag([1.0, 1.0, -1.0])    # FLU -> FLD (world z down)
        R_total = S2 @ R_bodyFLU_to_worldFLU @ S1
        return R_total

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

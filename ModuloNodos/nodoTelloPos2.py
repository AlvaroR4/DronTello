#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import tellopy
import threading
import time
import math
import numpy as np
import cv2

ROS_TOPIC_IMAGEN_RAW = 'camera/image_raw'
ROS_TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
ROS_TOPIC_POSE = '/tello/pose'  # publicador con [altura_altimetro, posx, posy, posz, roll_deg, pitch_deg, yaw_deg]
TIMER_PERIOD_POSE = 0.1  # publicar pose a 10 Hz
TIMER_PERIOD_CAMARA = 1.0 / 30.0  # ~30 FPS para la cámara

class NodoTelloPy(Node):
    def __init__(self):
        super().__init__('nodo_tellopy_tello')
        self.get_logger().info("Iniciando Nodo Tello (tellopy)")

        # --- variables internas en español (preferencia del usuario) ---
        self.bridge = CvBridge()
        self.tello = tellopy.Tello()
        self.ultimo_log = None
        self.ultimo_frame = None
        self.ultimo_flight = None

        # publicadores
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publicador_imagen = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)
        self.get_logger().info(f"Publicando imagen RAW en: {ROS_TOPIC_IMAGEN_RAW}")

        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publicador_pose = self.create_publisher(Float32MultiArray, ROS_TOPIC_POSE, qos_profile_pose)
        self.get_logger().info(f"Publicando pose en: {ROS_TOPIC_POSE}")

        # suscriptor de comandos de velocidad (usa mismo topic que ejemplo)
        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.suscriptor_comandos_velocidad = self.create_subscription(
            Float32MultiArray,
            ROS_TOPIC_COMANDOS_VELOCIDAD,
            self.callback_comandos_velocidad,
            qos_profile_cmd
        )
        self.get_logger().info(f"Suscrito a comandos de velocidad en: {ROS_TOPIC_COMANDOS_VELOCIDAD}")

        # timers
        self.timer_pose = self.create_timer(TIMER_PERIOD_POSE, self._publicar_pose)
        self.timer_camara = self.create_timer(TIMER_PERIOD_CAMARA, self._publicar_frame)

        # conectar al tello y suscribirse a eventos
        self.get_logger().info("Conectando al Tello (tellopy)...")
        # conectar en hilo aparte para no bloquear el init
        hilo_conexion = threading.Thread(target=self._iniciar_conexion, daemon=True)
        hilo_conexion.start()

    def _iniciar_conexion(self):
        try:
            self.tello.set_loglevel(self.tello.LOG_ERROR)
            # Suscripciones a eventos
            self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self._on_flight_data)
            self.tello.subscribe(self.tello.EVENT_LOG_DATA, self._on_log_data)
            self.tello.subscribe(self.tello.EVENT_VIDEO_FRAME, self._on_video_frame)

            self.tello.connect()
            self.tello.wait_for_connection(30.0)

            # iniciar streaming de vídeo
            try:
                self.tello.start_video()
            except Exception:
                try:
                    self.tello.send_command('streamon')
                except Exception:
                    pass

            self.get_logger().info("Tello conectado y stream iniciado.")
        except Exception as ex:
            self.get_logger().info(f"Error conexión: {ex}")

    # ----------------- handlers de tellopy -----------------
    def _on_flight_data(self, event, sender, data, **args):
        self.ultimo_flight = data

    def _on_log_data(self, event, sender, data, **args):
        self.ultimo_log = data

    def _on_video_frame(self, event, sender, data, **args):
        try:
            frame = data
            image_pil = frame.to_image()
            np_img = np.array(image_pil)
            frame_bgr = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
            self.ultimo_frame = frame_bgr
        except Exception:
            pass

    # ----------------- publicación de imagen -----------------
    def _publicar_frame(self):
        if self.ultimo_frame is None:
            return
        try:
            ros_image_msg = self.bridge.cv2_to_imgmsg(self.ultimo_frame, encoding="bgr8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera_link_raw"
            self.publicador_imagen.publish(ros_image_msg)
        except Exception:
            pass

    def _publicar_pose(self):
        msg = Float32MultiArray()
        # orden: [altura_altimetro, posx, posy, posz, roll_deg, pitch_deg, yaw_deg]
        altura_altimetro = 0.0
        posx = 0.0
        posy = 0.0
        posz = 0.0
        roll_deg = 0.0
        pitch_deg = 0.0
        yaw_deg = 0.0

        try:
            if self.ultimo_flight is not None:
                raw_alt = getattr(self.ultimo_flight, 'height', 0.0) or 0.0
                raw_alt = float(raw_alt)
                try:
                    if math.isfinite(raw_alt):
                        # convertimos de centímetros a metros (si la librería devuelve cm)
                        altura_altimetro = raw_alt / 100.0
                    else:
                        # fallback al último valor filtrado si existe
                        altura_altimetro = self.pose_filtrada.get('altura', 0.0) if hasattr(self, 'pose_filtrada') else 0.0
                except Exception:
                    altura_altimetro = self.pose_filtrada.get('altura', 0.0) if hasattr(self, 'pose_filtrada') else 0.0
        except Exception:
            altura_altimetro = self.pose_filtrada.get('altura', 0.0) if hasattr(self, 'pose_filtrada') else 0.0

        # extracción de posición MVO y quaternion IMU
        try:
            if self.ultimo_log is not None:
                log = self.ultimo_log
                if hasattr(log, 'mvo') and log.mvo is not None:
                    try:
                        posx = float(log.mvo.pos_x)
                        posy = float(log.mvo.pos_y)
                        posz = float(log.mvo.pos_z)
                    except Exception:
                        pass
                if hasattr(log, 'imu') and log.imu is not None:
                    try:
                        q0 = float(getattr(log.imu, 'q0', 1.0))
                        q1 = float(getattr(log.imu, 'q1', 0.0))
                        q2 = float(getattr(log.imu, 'q2', 0.0))
                        q3 = float(getattr(log.imu, 'q3', 0.0))
                        roll_rad, pitch_rad, yaw_rad = self._quaternion_to_euler(q0, q1, q2, q3)
                        roll_deg = math.degrees(roll_rad)
                        pitch_deg = math.degrees(pitch_rad)
                        yaw_deg = math.degrees(yaw_rad)
                    except Exception:
                        pass
        except Exception:
            pass

        msg.data = [float(altura_altimetro), float(posx), float(posy), float(posz),
                    float(roll_deg), float(pitch_deg), float(yaw_deg)]
        self.publicador_pose.publish(msg)


    # ----------------- conversión quaternion -> euler (roll,pitch,yaw) -----------------
    def _quaternion_to_euler(self, w, x, y, z):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    # ----------------- callback para recibir velocidades de control (con casos especiales) -----------------
    def callback_comandos_velocidad(self, msg: Float32MultiArray):
        # Intentamos leer cada componente; si falla, se queda en 0.0 (como en tu ejemplo antiguo)
        try:
            lr = float(msg.data[0])
        except Exception:
            lr = 0.0
        try:
            fb = float(msg.data[1])
        except Exception:
            fb = 0.0
        try:
            ud = float(msg.data[2])
        except Exception:
            ud = 0.0
        try:
            yv = float(msg.data[3])
        except Exception:
            yv = 0.0

        # Casos especiales (siguen exactamente la lógica que me has pasado)
        if lr == 2.0 and fb == 0.0 and ud == 0.0:
            self.get_logger().info("Comando: land")
            try:
                self.tello.land()
            except Exception:
                pass
            return
        elif lr == 2.0 and fb == 2.0 and ud == 0.0:
            self.get_logger().info("Comando: emergency (kill)")
            try:
                self.tello.land()
                time.sleep(0.1)
                self.tello.quit()
            except Exception:
                pass
            return
        elif lr == 2.0 and fb == 2.0 and ud == 2.0:
            self.get_logger().info("Comando: takeoff")
            try:
                self.tello.takeoff()
            except Exception:
                pass
            return

        # Si no es un caso especial, se mapean los valores a control de vuelo normal
        def _norm(v):
            try:
                nv = float(v) / 100.0
                if nv > 1.0:
                    nv = 1.0
                if nv < -1.0:
                    nv = -1.0
                return nv
            except:
                return 0.0

        try:
            # Intento usar métodos set_* si existen
            self.tello.set_roll(_norm(lr))
            self.tello.set_pitch(_norm(fb))
            self.tello.set_throttle(_norm(ud))
            self.tello.set_yaw(_norm(yv))
        except Exception:
            # Fallback silencioso a comando 'rc' si set_* no está disponible
            try:
                cmd = f'rc {int(lr)} {int(fb)} {int(ud)} {int(yv)}'
                try:
                    self.tello.send_command(cmd)
                except Exception:
                    try:
                        self.tello.send_packet_data(cmd)
                    except Exception:
                        pass
            except Exception:
                pass

    # ----------------- limpieza y cierre -----------------
    def destroy_node(self):
        try:
            if self.timer_camara:
                self.timer_camara.cancel()
        except Exception:
            pass
        try:
            if self.timer_pose:
                self.timer_pose.cancel()
        except Exception:
            pass

        try:
            self.tello.set_roll(0)
            self.tello.set_pitch(0)
            self.tello.set_throttle(0)
            self.tello.set_yaw(0)
            time.sleep(0.1)
            self.tello.land()
            time.sleep(2.0)
            try:
                self.tello.send_command('streamoff')
            except Exception:
                pass
            self.tello.quit()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoTelloPy()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        try:
            nodo.tello.emergency()
        except Exception:
            pass
        nodo.get_logger().info("Emergencia.")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
        print("Programa NodoTello (tellopy) finalizado.")

if __name__ == '__main__':
    main()

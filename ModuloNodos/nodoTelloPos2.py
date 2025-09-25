#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import logging
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import tellopy
import av
import cv2
import io
import threading
import time
import traceback

TELLO_IP = '192.168.10.1'
ROS_TOPIC_IMAGEN_RAW = 'camera/image_raw'
ROS_TOPIC_COMANDOS_VELOCIDAD = 'tello/comandos_velocidad'
ROS_TOPIC_POSE = '/tello/posicion'   # [altura, x, y, z, roll, pitch, yaw]

TIMER_PERIODO_CAMARA = 0.03


class NodoTelloPos(Node):
    def __init__(self):
        super().__init__('nodo_tello_pos')
        self.get_logger().info('Iniciando nodoTelloPos')

        # Publicador de pose/posición
        self.pub = self.create_publisher(Float32MultiArray, ROS_TOPIC_POSE, 10)
        self.pub_alt = self.create_publisher(Float32MultiArray, '/tello/altura', 2) #para probar el altimetro

        # CvBridge y publicador de imagen
        self.bridge = CvBridge()
        qos_profile_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                     history=HistoryPolicy.KEEP_LAST, depth=1)
        self.publicador_imagen = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)

        # Suscriptor de comandos (igual que antes)
        qos_profile_cmd = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                     history=HistoryPolicy.KEEP_LAST, depth=10)
        self.suscriptor_comandos_velocidad = self.create_subscription(
            Float32MultiArray,
            ROS_TOPIC_COMANDOS_VELOCIDAD,
            self.callback_comandos_velocidad,
            qos_profile_cmd
        )
        self.get_logger().info(f"Suscrito a comandos de velocidad en: {ROS_TOPIC_COMANDOS_VELOCIDAD}")

        # Estado interno de tello / log
        self.latest_flight_data = None
        self.latest_log_data = None
        self.connected = False

        # Inicializar y conectar tello
        self.tello = tellopy.Tello()

        # Establecer nivel de log correcto (usar entero, no string)
        try:
            # usar la constante del módulo logging (entero)
            self.tello.set_loglevel(logging.ERROR)
        except Exception:
            # fallback al valor numérico de ERROR (40) si la API no acepta constantes
            try:
                self.tello.set_loglevel(40)
            except Exception:
                # no fatal — seguir adelante
                pass

        # Intentar conectar con manejo de excepciones para evitar crash total
        try:
            self.tello.connect()
            # espera por la conexión (time out razonable)
            try:
                self.tello.wait_for_connection(30.0)
            except Exception:
                # wait_for_connection puede lanzar si no conecta en tiempo — lo manejamos igualmente
                pass
            self.connected = True
            self.get_logger().info("Conexión a Tello establecida (o intento realizado sin excepción).")
        except Exception as exc:
            self.get_logger().error(f"No se pudo conectar a Tello: {exc}")
            # dejar connected = False y seguir en modo degradado

        # Subscripciones a eventos tello (solo si hay conexión)
        if self.connected:
            try:
                self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self._flight_data_handler)
            except Exception:
                pass
            try:
                self.tello.subscribe(self.tello.EVENT_LOG_DATA, self._log_data_handler)
            except Exception:
                pass

            # arrancar hilo de vídeo solo si la conexión fue aceptada
            try:
                self._start_video_thread()
            except Exception as e:
                self.get_logger().warning(f"No se inició el thread de vídeo: {e}")
        else:
            self.get_logger().warning("Nodo en modo degradado: Tello no conectado. Algunas funciones (vídeo/telemetría) no estarán disponibles.")

        # Timers para publicar pose y altímetro (funcionarán aunque no haya conexión; publicarán ceros hasta que lleguen datos)
        self.create_timer(0.1, self._publicar_comando)
        self.create_timer(0.1, self.publicar_altimetro)


    def _flight_data_handler(self, event, sender, data, **args):
        # dato directo que provee tellopy (flight data)
        self.latest_flight_data = data

    def _log_data_handler(self, event, sender, data, **args):
        # log data (algunos campos como mvo pueden ir aquí)
        self.latest_log_data = data

    def _start_video_thread(self):
        def _video_loop():
            try:
                # get_video_stream() requiere que el tello esté conectado y el stream disponible
                stream = self.tello.get_video_stream()
                container = av.open(stream)
                for frame in container.decode(video=0):
                    img = frame.to_ndarray(format='bgr24')
                    try:
                        ros_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                        self.publicador_imagen.publish(ros_image_msg)
                    except Exception:
                        pass
            except Exception:
                self.get_logger().warning("No se pudo abrir flujo de vídeo (o ya está en uso).")

        t = threading.Thread(target=_video_loop, daemon=True)
        t.start()

    def _safe_get(self, obj, *attrs, default=0.0):
        try:
            cur = obj
            for a in attrs:
                cur = getattr(cur, a, None)
                if cur is None:
                    return default
            # si es convertible a float
            return float(cur)
        except Exception:
            return default

    def _get_rotations_from_flight_or_log(self):
        """
        Intenta extraer roll, pitch, yaw en grados desde latest_flight_data o latest_log_data.
        Devuelve (roll_deg, pitch_deg, yaw_deg)
        """
        # 1) intentamos en latest_flight_data campos directos
        roll = 0.0
        pitch = 0.0
        yaw = 0.0
        fd = self.latest_flight_data
        if fd is not None:
            # campos comunes
            for name in ('roll', 'roll_deg', 'attitude_roll'):
                val = getattr(fd, name, None)
                if val is not None:
                    try:
                        roll = float(val)
                        break
                    except Exception:
                        pass
            for name in ('pitch', 'pitch_deg', 'attitude_pitch'):
                val = getattr(fd, name, None)
                if val is not None:
                    try:
                        pitch = float(val)
                        break
                    except Exception:
                        pass
            for name in ('yaw', 'yaw_deg', 'attitude_yaw'):
                val = getattr(fd, name, None)
                if val is not None:
                    try:
                        yaw = float(val)
                        break
                    except Exception:
                        pass

        # 2) si no hay en flight_data, intentar mirar latest_log_data
        if (roll == 0.0 and pitch == 0.0 and yaw == 0.0) and (self.latest_log_data is not None):
            ld = self.latest_log_data
            # intentos en nested attrs
            roll = roll or self._safe_get(ld, 'attitude', 'roll', default=roll)
            pitch = pitch or self._safe_get(ld, 'attitude', 'pitch', default=pitch)
            yaw = yaw or self._safe_get(ld, 'attitude', 'yaw', default=yaw)

            roll = roll or self._safe_get(ld, 'imu', 'roll', default=roll)
            pitch = pitch or self._safe_get(ld, 'imu', 'pitch', default=pitch)
            yaw = yaw or self._safe_get(ld, 'imu', 'yaw', default=yaw)

        try:
            return float(roll), float(pitch), float(yaw)
        except Exception:
            return 0.0, 0.0, 0.0

    def _publicar_comando(self):
        msg = Float32MultiArray()
        try:
            altura = 0.0
            if self.latest_flight_data and hasattr(self.latest_flight_data, 'height'):
                try:
                    altura = float(self.latest_flight_data.height)
                except Exception:
                    altura = 0.0

            x = 0.0
            y = 0.0
            z = 0.0
            if self.latest_log_data is not None:
                mvo = getattr(self.latest_log_data, 'mvo', None)
                if mvo is not None:
                    try:
                        x = float(getattr(mvo, 'pos_x', 0.0))
                        y = float(getattr(mvo, 'pos_y', 0.0))
                        z = float(getattr(mvo, 'pos_z', 0.0))
                    except Exception:
                        x, y, z = 0.0, 0.0, 0.0

            # obtener rotaciones
            roll_deg, pitch_deg, yaw_deg = self._get_rotations_from_flight_or_log()

            try:
                if (roll_deg == 0.0 and pitch_deg == 0.0 and yaw_deg == 0.0) and self.latest_flight_data is not None:
                    roll_deg = float(getattr(self.latest_flight_data, 'roll', roll_deg))
                    pitch_deg = float(getattr(self.latest_flight_data, 'pitch', pitch_deg))
                    yaw_deg = float(getattr(self.latest_flight_data, 'yaw', yaw_deg))
            except Exception:
                pass

            msg.data = [float(altura), float(x), float(y), float(z),
                        float(roll_deg), float(pitch_deg), float(yaw_deg)]
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publicando pose: {e}")
            msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            try:
                self.pub.publish(msg)
            except Exception:
                pass

    def callback_comandos_velocidad(self, msg):
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

        if lr == 2.0 and fb == 0.0 and ud == 0.0:
            self.get_logger().info("Comando: land")
            try:
                self.tello.land()
            except Exception:
                pass
        elif lr == 2.0 and fb == 2.0 and ud == 0.0:
            self.get_logger().info("Comando: emergency (kill)")
            try:
                self.tello.land()
                time.sleep(0.1)
                self.tello.quit()
            except Exception:
                pass
        elif lr == 2.0 and fb == 2.0 and ud == 2.0:
            self.get_logger().info("Comando: takeoff")
            try:
                self.tello.takeoff()
            except Exception:
                pass

    def publicar_altimetro(self):

        msg = Float32MultiArray()
        try:
            altura_cm = float(getattr(self.latest_flight_data, 'height', 0.0) or 0.0)
            altura_m = altura_cm / 100.0 if altura_cm > 1.0 else altura_cm 
        except Exception:
            altura_m = 0.0
        msg.data = [altura_m]
        try:
            self.pub_alt.publish(msg)
        except Exception:
            pass

    def destroy_node(self):
        try:
            self.tello.quit()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo_tello = None
    try:
        nodo_tello = NodoTelloPos()
        rclpy.spin(nodo_tello)
    except KeyboardInterrupt:
        try:
            nodo_tello.get_logger().info("Ctlr+C")
            nodo_tello.tello.emergency()
        except Exception:
            pass
    finally:
        if nodo_tello:
            nodo_tello.destroy_node()
        rclpy.shutdown()
        print("Programa finalizado.")


if __name__ == '__main__':
    main()

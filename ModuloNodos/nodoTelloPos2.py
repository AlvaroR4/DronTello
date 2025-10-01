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
import subprocess
import sys

ROS_TOPIC_IMAGEN_RAW = 'camera/image_raw'
ROS_TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
ROS_TOPIC_POSE = '/tello/pose'  # publicador con [altura_altimetro, posx, posy, posz, roll_deg, pitch_deg, yaw_deg]
TIMER_PERIOD_POSE = 0.1  # publicar pose a 10 Hz
TIMER_PERIOD_CAMARA = 1.0 / 30.0  # ~30 FPS para la cámara

READ_CHUNK = 4096
FFMPEG_FPS = int(1.0 / TIMER_PERIOD_CAMARA)

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

        # watchdog para evitar stick persistente (si no llegan comandos, enviar 0s)
        self._last_cmd_time = time.time()
        self._cmd_watchdog = self.create_timer(0.2, self._cmd_watchdog_cb)

        # variables para pipeline de vídeo minimal
        self._ffmpeg_proc = None
        self._ffmpeg_thread = None
        self._stdout_buffer = bytearray()
        self._stop_event = threading.Event()

        # conectar al tello y suscribirse a eventos (en hilo para no bloquear init)
        hilo_conexion = threading.Thread(target=self._iniciar_conexion, daemon=True)
        hilo_conexion.start()

    def _iniciar_conexion(self):
        """
        Conecta al Tello, suscribe eventos de flight/log y configura la tubería minimal
        de vídeo: tellopy EVENT_VIDEO_FRAME -> ffmpeg stdin (-f h264) -> ffmpeg stdout MJPEG ->
        ensamblado JPEG -> ultimo_frame (cv2).
        """
        try:
            # Suscripciones de datos (flight/log)
            try:
                self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self._on_flight_data)
                self.tello.subscribe(self.tello.EVENT_LOG_DATA, self._on_log_data)
            except Exception:
                pass

            # Conectar y esperar
            try:
                self.tello.connect()
                self.tello.wait_for_connection(30.0)
            except Exception as e:
                self.get_logger().warning(f"No se ha podido conectar al Tello: {e}")
                return

            # Lanzar ffmpeg: entrada H264 desde stdin, salida MJPEG por stdout
            ff_cmd = [
                'ffmpeg', '-hide_banner', '-loglevel', 'error',
                '-fflags', 'nobuffer', '-flags', 'low_delay',
                '-f', 'h264', '-i', 'pipe:0',
                '-r', str(FFMPEG_FPS),
                '-f', 'image2pipe', '-vcodec', 'mjpeg', 'pipe:1'
            ]
            try:
                self._ffmpeg_proc = subprocess.Popen(
                    ff_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
                )
            except Exception as e:
                self.get_logger().warning(f"ffmpeg no pudo arrancar: {e}")
                self._ffmpeg_proc = None

            # Suscribirse a paquetes H264 provenientes de tellopy
            try:
                self.tello.subscribe(self.tello.EVENT_VIDEO_FRAME, self._video_frame_handler)
                # solicitar inicio del stream
                try:
                    self.tello.start_video()
                except Exception:
                    pass
            except Exception as e:
                self.get_logger().warning(f"No se pudo subscribir a EVENT_VIDEO_FRAME: {e}")

            # arrancar thread que lee stdout de ffmpeg y ensambla JPEGs
            if self._ffmpeg_proc and self._ffmpeg_proc.stdout:
                self._ffmpeg_thread = threading.Thread(target=self._ffmpeg_stdout_reader, daemon=True)
                self._ffmpeg_thread.start()

            # enviar ceros iniciales por seguridad (necesario en algunos casos)
            try:
                self._send_zero_rc()
            except Exception:
                pass

            self.get_logger().info("Tello conectado y pipeline de vídeo minimal iniciado.")
        except Exception as ex:
            self.get_logger().warning(f"Error en iniciar_conexion: {ex}")

    def _video_frame_handler(self, event, sender, data, **args):
        if self._ffmpeg_proc and self._ffmpeg_proc.stdin:
            self._ffmpeg_proc.stdin.write(data)


    def _ffmpeg_stdout_reader(self):
        """
        Lee stdout de ffmpeg en chunks y ensambla JPEGs buscando SOI/EOI.
        Cuando decodifica un JPEG con OpenCV, actualiza self.ultimo_frame.
        """
        try:
            out = self._ffmpeg_proc.stdout
            buf = self._stdout_buffer
            while not self._stop_event.is_set():
                chunk = out.read(READ_CHUNK)
                if not chunk:
                    time.sleep(0.005)
                    continue
                buf.extend(chunk)
                # extraer JPEGs completos
                while True:
                    start = buf.find(b'\xff\xd8')
                    if start < 0:
                        break
                    end = buf.find(b'\xff\xd9', start + 2)
                    if end < 0:
                        break
                    jpg = bytes(buf[start:end+2])
                    del buf[:end+2]
                    # decodificar JPEG a BGR con OpenCV
                    try:
                        arr = np.frombuffer(jpg, dtype=np.uint8)
                        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                        if img is None:
                            continue
                        # opcional: limitar tamaño para no saturar CPU (si lo deseas)
                        # h, w = img.shape[:2]
                        # if max(h, w) > 960:
                        #     scale = 960 / float(max(h, w))
                        #     img = cv2.resize(img, (int(w*scale), int(h*scale)))
                        self.ultimo_frame = img
                    except Exception:
                        # ignorar JPEG corrupto en minimal
                        continue
        except Exception:
            return

    # ----------------- utilidades de seguridad para control -----------------
    def _send_zero_rc(self):
        """Intenta poner los sticks a cero (varias alternativas según API disponible)."""
        try:
            # preferir set_* si existen
            try:
                self.tello.set_roll(0)
                self.tello.set_pitch(0)
                self.tello.set_throttle(0)
                self.tello.set_yaw(0)
                return
            except Exception:
                pass
            # fallback a comando 'rc'
            try:
                self.tello.send_command('rc 0 0 0 0')
                return
            except Exception:
                pass
            # último recurso
            try:
                self.tello.send_packet_data('rc 0 0 0 0')
            except Exception:
                pass
        except Exception:
            pass

    def _cmd_watchdog_cb(self):
        """
        Si no hemos recibido comandos desde >0.5 s, enviar comandos neutros
        para evitar movimiento persistente por orden anterior.
        """
        try:
            if time.time() - getattr(self, '_last_cmd_time', 0.0) > 0.5:
                self._send_zero_rc()
        except Exception:
            pass

    # ----------------- handlers de tellopy -----------------
    def _on_flight_data(self, event, sender, data, **args):
        self.ultimo_flight = data

    def _on_log_data(self, event, sender, data, **args):
        self.ultimo_log = data

    # ----------------- publicación de imagen -----------------
    def _publicar_frame(self):
        """
        Publica self.ultimo_frame en camera/image_raw. Si no hay frame, publica un frame negro
        como fallback. (Mantiene el comportamiento anterior.)
        """
        try:
            if getattr(self, 'ultimo_frame', None) is None:
                h = 480; w = 640
                black = np.zeros((h, w, 3), dtype=np.uint8)
                ros_image_msg = self.bridge.cv2_to_imgmsg(black, encoding="bgr8")
                ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                ros_image_msg.header.frame_id = "tello_camera_link_raw"
                self.publicador_imagen.publish(ros_image_msg)
                return

            try:
                ros_image_msg = self.bridge.cv2_to_imgmsg(self.ultimo_frame, encoding="bgr8")
            except Exception:
                try:
                    frame = self.ultimo_frame.astype(np.uint8)
                    ros_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                except Exception:
                    return

            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera_link_raw"
            self.publicador_imagen.publish(ros_image_msg)
        except Exception:
            return

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
        # actualizar hora del último comando recibido
        self._last_cmd_time = time.time()

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
                # después del takeoff, asegurar sticks a 0 varios ciclos
                def _post_takeoff_zero():
                    time.sleep(0.5)
                    for _ in range(8):
                        self._send_zero_rc()
                        time.sleep(0.1)
                threading.Thread(target=_post_takeoff_zero, daemon=True).start()
            except Exception:
                pass
            return

        # Si no es un caso especial, se mapean los valores a control de vuelo normal
        def _norm(v):
            try:
                nv = float(v) / 100.0
                # deadzone: ignorar ruidos pequeños
                if abs(nv) < 0.05:
                    return 0.0
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

        # cancelar watchdog
        try:
            if getattr(self, '_cmd_watchdog', None):
                try:
                    self._cmd_watchdog.cancel()
                except Exception:
                    pass
        except Exception:
            pass

        # parar threads y ffmpeg
        try:
            self._stop_event.set()
            if self._ffmpeg_proc:
                try:
                    if self._ffmpeg_proc.stdin:
                        self._ffmpeg_proc.stdin.close()
                except Exception:
                    pass
                try:
                    if self._ffmpeg_proc.poll() is None:
                        self._ffmpeg_proc.kill()
                except Exception:
                    pass
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

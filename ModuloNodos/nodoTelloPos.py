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

TELLO_IP = '192.168.10.1'  # no usado por tellopy (usa broadcast / wifi), pero lo dejo para compatibilidad
ROS_TOPIC_IMAGEN_RAW = 'camera/image_raw'
ROS_TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
TIMER_PERIODO_CAMARA = 1.0 / 30.0    # ~30 FPS para la cámara

class NodoTelloTellopy(Node):
    def __init__(self):
        super().__init__('nodo_tello_tellopy')
        self.get_logger().info("Iniciando Nodo Camara Tello (tellopy)")

        # Publisher para altura / yaw (mantengo el nombre original '/tello/yaw')
        self.pub = self.create_publisher(Float32MultiArray, "/tello/yaw", 10)

        # variables para almacenar datos recientes (colócalas en __init__)
        self.latest_flight_data = None
        self.latest_log_data = None

        self.bridge = CvBridge()
        self.tello = tellopy.Tello()
        self.video_thread = None
        self.video_running = False

        # Variables para datos de vuelo
        self.latest_flight_data = None
        self.latest_log_data = None

        # Timer para publicar la altura periódicamente (imitando _publicar_comando)
        periodo = 1.0 / 30.0
        self.timer = self.create_timer(periodo, self._publicar_comando)

        # Conexión al Tello
        try:
            self.get_logger().info("Conectando al Tello...")
            self.tello.set_loglevel(self.tello.LOG_ERROR)
            # Suscribir datos de vuelo (si están disponibles)
            self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self._flight_data_handler)
            self.tello.subscribe(self.tello.EVENT_LOG_DATA, self._log_data_handler)

            # Conectar y esperar
            self.tello.connect()
            self.tello.wait_for_connection(60.0)
            self.get_logger().info("Conectado al Tello.")

            # Preparar video: start_video + get_video_stream en un hilo
            self.tello.start_video()
            time.sleep(0.5)
            self.video_running = True
            self.video_thread = threading.Thread(target=self._video_loop, daemon=True)
            self.video_thread.start()
            self.get_logger().info("Hilo de vídeo arrancado.")

            # No forzamos takeoff aquí; el original tenía comentado takeoff.
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"Error conectando al Tello: {e}")
            traceback.print_exc()

        # Publicador de imagen RAW
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publicador_imagen = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)
        self.get_logger().info(f"Publicando imagen RAW en: {ROS_TOPIC_IMAGEN_RAW}")

        # Suscriptor para comandos de velocidad
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

        # Timer para captura y publicación de imagen (mantengo similar al original)
        self.timer_camara = self.create_timer(TIMER_PERIODO_CAMARA, lambda: None)  # el loop de vídeo publica directamente

    def _flight_data_handler(self, event, sender, data, **args):
        self.latest_flight_data = data

    def _log_data_handler(self, event, sender, data, **args):
        self.latest_log_data = data


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

            msg.data = [float(altura), float(x), float(y), float(z)]
            self.pub.publish(msg)
        except Exception as e:
            # En caso de error: publicar ceros para mantener consistencia
            self.get_logger().error(f"Error publicando pose: {e}")
            msg.data = [0.0, 0.0, 0.0, 0.0]
            self.pub.publish(msg)


    def callback_comandos_velocidad(self, msg):
        # Esperamos msg.data = [lr, fb, ud, yv] en unidades similares a djitellopy (-100..100)
        try:
            lr = float(msg.data[0])
            fb = float(msg.data[1])
            ud = float(msg.data[2])
            yv = float(msg.data[3])
            self.get_logger().info(f"RC recibido: lr={lr}, fb={fb}, ud={ud}, yv={yv}")

            # Comandos especiales (igual que tu original)
            if lr == 2.0 and fb == 0.0 and ud == 0.0:
                self.get_logger().info("Comando: land")
                self.tello.land()
                # no destruimos el nodo automáticamente aquí; limpiamos recursos en destroy_node
            elif lr == 2.0 and fb == 2.0 and ud == 0.0:
                self.get_logger().info("Comando: emergency (kill)")
                try:
                    # tellopy no trae método emergency explícito; usamos quit() después de land.
                    self.tello.land()
                    time.sleep(0.1)
                    self.tello.quit()
                except Exception:
                    pass
            elif lr == 2.0 and fb == 2.0 and ud == 2.0:
                self.get_logger().info("Comando: takeoff")
                self.tello.takeoff()

            # Mapeo de -100..100 (djitellopy) a -1.0..1.0 (set_roll/pitch/throttle/yaw)
            def normalize(v):
                # clamp y normaliza
                v = max(-100.0, min(100.0, v))
                return v / 100.0

            # roll (left/right) => set_roll (positive -> right)
            # pitch (forward/back) => set_pitch (positive -> forward)
            # throttle (up/down) => set_throttle (positive -> up)
            # yaw => set_yaw (positive -> clockwise/right)
            try:
                self.tello.set_roll(normalize(lr))
                self.tello.set_pitch(normalize(fb))
                self.tello.set_throttle(normalize(ud))
                self.tello.set_yaw(normalize(yv))
            except AttributeError:
                # Algunas versiones usan nombres diferentes; como fallback notificar
                self.get_logger().warning("La versión de tellopy instalada puede no soportar set_roll/set_pitch/set_throttle/set_yaw. Ajusta según API.")
        except Exception as e:
            self.get_logger().error(f"Error aplicando RC: {e}")
            # intentar detener movimientos
            try:
                self.tello.set_roll(0.0)
                self.tello.set_pitch(0.0)
                self.tello.set_throttle(0.0)
                self.tello.set_yaw(0.0)
            except Exception:
                pass

    def _video_loop(self):

        try:
            # tellopy.get_video_stream() prepara un file-like object interno que PyAV puede leer.
            stream = self.tello.get_video_stream()
            # Abrimos el contenedor como h264 raw
            container = av.open(stream, format='h264', mode='r')

            # Decodificamos continuamente
            for packet in container.demux():
                if not self.video_running:
                    break
                for frame in packet.decode():
                    # Convertir a numpy BGR utilizando PyAV
                    img = frame.to_ndarray(format='bgr24')
                    # Publicar en ROS
                    try:
                        ros_image_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
                        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
                        ros_image_msg.header.frame_id = "tello_camera_link_raw"
                        self.publicador_imagen.publish(ros_image_msg)
                    except Exception as e:
                        self.get_logger().error(f"Error publicando imagen ROS: {e}")
            # cerrar contenedor
            try:
                container.close()
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f"Error en el bucle de vídeo: {e}")
            traceback.print_exc()
        finally:
            self.get_logger().info("Bucle de vídeo finalizado.")

    def cleanup_recursos(self):
        self.get_logger().info("Iniciando limpieza de recursos del Tello (tellopy)...")
        try:
            # detener cualquier control
            try:
                self.tello.land()
            except Exception:
                pass
            time.sleep(1.0)
            # detener vídeo
            try:
                self.tello.stop_video()
            except Exception:
                # algunas versiones no exponen stop_video
                pass
            # detener hilo de vídeo
            self.video_running = False
            if self.video_thread and self.video_thread.is_alive():
                self.video_thread.join(timeout=2.0)
            # desconectar / cerrar
            try:
                self.tello.quit()
            except Exception:
                pass
            self.get_logger().info("Limpieza de Tello finalizada.")
        except Exception as e:
            self.get_logger().error(f"Error durante cleanup: {e}")

    def destroy_node(self):
        # cancelar timers si existen
        try:
            if self.timer_camara:
                self.timer_camara.cancel()
        except Exception:
            pass

        self.cleanup_recursos()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo_tello = NodoTelloTellopy()
    try:
        rclpy.spin(nodo_tello)
    except KeyboardInterrupt:
        try:
            nodo_tello.get_logger().info("Ctlr+C")
            nodo_tello.tello.emergency()
        except Exception:
            pass
    finally:
        nodo_tello.destroy_node()
        rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()

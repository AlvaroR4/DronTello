import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, Quaternion
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import time
import threading
from pynput import keyboard 
import math

TELLO_IP = '192.168.10.1'
ROS_TOPIC_IMAGEN_RAW = '/tello/imagen'
ROS_TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
TIMER_PERIODO_CAMARA = 1.0 / 30.0    # ~30 FPS para la cámara

class NodoTello(Node):
    def __init__(self):
        super().__init__('nodo_tello_tello')
        self.get_logger().info(f"Iniciando Nodo Camara Tello")

        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)
        self.frame_reader = None 

        self.get_logger().info("Conectando al Tello ")
        self.tello.connect()
        self.tello.streamoff()
        time.sleep(0.5)
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read() 
        time.sleep(1.0) 

        self.get_logger().info("Despegando Tello")
        #self.tello.takeoff()
        self.tello.turn_motor_on()
        #self.tello.initiate_throw_takeoff()
        time.sleep(2) 

        # Publicador para la imagen RAW
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publicador_imagen = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)
        self.get_logger().info(f"Publicando imagen RAW en: {ROS_TOPIC_IMAGEN_RAW}")

        # Suscriptor para los comandos de velocidad
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

        # ------------------------------------------------------------
        # Publicador para la pose simulada (como si fuera ORB-SLAM3)
        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_pose = self.create_publisher(PoseStamped, "/robot_pose_slam", qos_profile_pose)
        self.get_logger().info("Publicando pose en /robot_pose_slam")

        # Timer para publicar pose (10 Hz aprox)
        self.timer_pose = self.create_timer(0.1, self.timer_callback_pose)
        # ------------------------------------------------------------

        # Timer para captura y publicación de imagen
        self.timer_camara = self.create_timer(TIMER_PERIODO_CAMARA, self.timer_callback_camara)
        self.get_logger().info(f"Timer de cámara iniciado ({1.0/TIMER_PERIODO_CAMARA:.1f} FPS).")


    def callback_comandos_velocidad(self, msg):
        # Esperamos msg.data = [lr, fb, ud, yv] en unidades entes de enviar a Tello (cm/s)
        try:
            lr = int(msg.data[0])
            fb = int(msg.data[1])
            ud = int(msg.data[2])
            yv = int(msg.data[3])
            self.get_logger().info(f"RC recibido: lr={lr}, fb={fb}, ud={ud}, yv={yv}")
            self.tello.send_rc_control(lr, fb, ud, yv)
            #self.tello.send_rc_control(0, 0, 0, 0)
        except Exception as e:
            self.tello.send_rc_control(0, 0, 0, 0)


    def timer_callback_camara(self):

        frame_bgr = self.frame_reader.frame # Obtenemos el frame

        # Publicar el frame
        ros_image_msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = "tello_camera_link_raw"
        self.publicador_imagen.publish(ros_image_msg)


    def timer_callback_pose(self):
        """
        Publica un PoseStamped en /robot_pose_slam.
        Estimación de posición:
         - x e y integrando velocidades del dron (cm/s)
         - z = altura del Tello (get_height) en metros
         - orientación: solo yaw (roll=pitch=0) convertidos a cuaternión
        """
        try:
            # Tiempo actual
            current_time = self.get_clock().now().nanoseconds / 1e9  # segundos
            if not hasattr(self, 'last_pose_time'):
                self.last_pose_time = current_time
            dt = current_time - self.last_pose_time
            self.last_pose_time = current_time

            # Integrar velocidades para estimar posición
            try:
                vx = self.tello.get_speed_x()  # cm/s
                vy = self.tello.get_speed_y()  # cm/s
            except Exception:
                vx, vy = 0.0, 0.0

            if not hasattr(self, 'x'):
                self.x = 0.0
            if not hasattr(self, 'y'):
                self.y = 0.0

            self.x += vx / 100.0 * dt  # convertir a metros
            self.y += vy / 100.0 * dt

            # Obtener altura en cm desde djitellopy, convertir a metros
            try:
                height_cm = self.tello.get_height()
            except Exception:
                height_cm = None
            z = float(height_cm)/100.0 if height_cm is not None else 0.0

            # Obtener yaw en grados
            try:
                yaw_deg = self.tello.get_yaw()
            except Exception:
                yaw_deg = 0.0
            yaw_rad = math.radians(float(yaw_deg))

            # Convertir yaw a cuaternión (roll=pitch=0)
            qw = math.cos(yaw_rad / 2.0)
            qz = math.sin(yaw_rad / 2.0)
            qx = 0.0
            qy = 0.0

            # Construir mensaje PoseStamped
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = z

            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw

            # Publicar
            self.pub_pose.publish(msg)

        except Exception as e:
            self.get_logger().warning(f"No se pudo publicar pose en /robot_pose_slam: {e}")


    def cleanup_recursos(self):

        self.get_logger().info("Iniciando limpieza de recursos del Tello...")

        # Detener RC control
        self.tello.send_rc_control(0, 0, 0, 0)
        time.sleep(0.1)

        self.get_logger().info("Aterrizando el Tello...")
        self.tello.land()
        time.sleep(5)

        self.get_logger().info("Deteniendo stream de vídeo del Tello...")
        self.tello.streamoff()

        self.get_logger().info("Desconectando del Tello...")
        self.tello.end()
        self.get_logger().info("Limpieza de Tello finalizada.")

    def destroy_node(self):
        # Cancelamos timers si existen
        if self.timer_camara:
            self.timer_camara.cancel()
        if hasattr(self, "timer_pose") and self.timer_pose:
            self.timer_pose.cancel()

        self.cleanup_recursos()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo_tello = NodoTello() 
    try:
        rclpy.spin(nodo_tello)
    except KeyboardInterrupt:
        nodo_tello.tello.emergency()
        nodo_tello.get_logger().info("Emergencia.")
    finally:
        nodo_tello.destroy_node()
        rclpy.shutdown()
        print("Programa NodoTello finalizado.")

if __name__ == '__main__':
    main()

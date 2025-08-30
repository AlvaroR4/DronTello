import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from transforms3d.euler import quat2euler

class NodoTrayectoria(Node):
    def __init__(self):
        super().__init__('nodo_trayectoria')
        self.get_logger().info("Iniciando Nodo de Trayectoria")

        # Variables de estado
        self.pose_dron = np.array([0.0, 0.0, 0.0])
        self.yaw_dron = 0.0
        self.waypoint = None  # [x,y,z,angulo]

        # Suscriptor a la pose (ORB-SLAM)
        qos_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(PoseStamped, '/robot_pose_slam', self.callback_pose, qos_pose)

        # Suscriptor a los puntos de puerta
        qos_wp = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(Float32MultiArray, '/punto', self.callback_punto, qos_wp)

        # Publicador de comandos de velocidad
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub_cmd_vel = self.create_publisher(Float32MultiArray, '/tello/comandos_velocidad', qos_cmd)

        # Timer de control (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

    def callback_pose(self, msg: PoseStamped):
        self.pose_dron = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        q = msg.pose.orientation
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')
        self.yaw_dron = math.degrees(yaw)

    def callback_punto(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.waypoint = np.array(msg.data)
            self.get_logger().info(f"Waypoint recibido: {self.waypoint}")

    def control_loop(self):
        if self.waypoint is None:
            return

        # Error de posición
        error_xyz = self.waypoint[:3] - self.pose_dron

        # Ganancias de control (simple proporcional)
        Kp = 50.0   # convierte metros -> cm/s (ajustar)
        vel_xyz = Kp * error_xyz  # en cm/s

        # Saturar velocidades a ±100 cm/s
        vel_xyz = np.clip(vel_xyz, -100, 100)

        # Error de orientación (yaw)
        angulo_deseado = self.waypoint[3]
        error_yaw = angulo_deseado - self.yaw_dron
        error_yaw = (error_yaw + 180) % 360 - 180  # normalizar a [-180,180]

        Kp_yaw = 30.0
        vel_yaw = np.clip(Kp_yaw * error_yaw, -100, 100)

        # Construir mensaje para Tello
        # OJO: mapeo al orden de send_rc_control(lr, fb, ud, yv)
        # En mundo: X=adelante, Y=izquierda, Z=arriba
        lr = int(-vel_xyz[1])  # eje Y mundo -> izquierda/derecha
        fb = int(vel_xyz[0])   # eje X mundo -> adelante/atrás
        ud = int(vel_xyz[2])   # eje Z mundo -> arriba/abajo
        yv = int(vel_yaw)      # yaw rate

        msg_out = Float32MultiArray()
        msg_out.data = [lr, fb, ud, yv]
        self.pub_cmd_vel.publish(msg_out)

        self.get_logger().info(f"Control -> lr:{lr} fb:{fb} ud:{ud} yv:{yv}")

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoTrayectoria()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Finalizando NodoTrayectoria...")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

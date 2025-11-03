import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
import time


class PublicadorVelocidad(Node):
    def __init__(self):
        super().__init__('publicador_prueba')

        self.pub_punto_angulo = self.create_publisher(Float32MultiArray, "/tello/punto_y_angulo", 10)

        # Timer cada 1 segundo para enviar el comando
        self.timer = self.create_timer(1.0, self.publicar_punto_y_angulo)
    
    def publicar_punto_y_angulo(self):
        msg = Float32MultiArray()
        msg.data = [1.21, 0.44, -1.12, -163.6]
        self.pub_punto_angulo.publish(msg)
        msg.data = [3.40, 2.50, -3.30, 49.0]
        self.pub_punto_angulo.publish(msg)
        msg.data = [5.40, -1.50, -1.30, -120.0]
        self.pub_punto_angulo.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = PublicadorVelocidad()

    try:
        rclpy.spin(nodo)
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

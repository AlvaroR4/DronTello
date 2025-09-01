import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
import time


class PublicadorVelocidad(Node):
    def __init__(self):
        super().__init__('publicador_prueba')

        # QoS debe coincidir con el que usa NodoTello en el suscriptor
        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publicador en el mismo tópico que escucha NodoTello
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/tello/comandos_velocidad',
            qos_profile_cmd
        )

        self.get_logger().info("Nodo velocidad0 iniciado, publicando en /tello/comandos_velocidad")

        # Timer cada 1 segundo para enviar el comando
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # Mensaje de velocidad con [lr, fb, ud, yv]
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)
        self.get_logger().info(f"Publicado: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    nodo = PublicadorVelocidad()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Nodo velocidad0 interrumpido por teclado.")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
        print("Programa velocidad0 finalizado.")


if __name__ == '__main__':
    main()

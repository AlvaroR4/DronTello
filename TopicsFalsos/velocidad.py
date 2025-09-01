import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray

VELOCIDAD = 10.0

class PublicadorMovimientoY(Node):
    def __init__(self):
        super().__init__('publicador_movimiento')

        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/tello/comandos_velocidad',
            qos_profile_cmd
        )

        self.get_logger().info("Nodo movimiento iniciado")

        self.direccion_positiva = True

        self.ciclos_actuales = 0
        self.ciclos_max = 10  # 10 ciclos * 0.5 s = 5 segundos

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        fb = VELOCIDAD if self.direccion_positiva else -VELOCIDAD
        msg.data = [0.0, fb, 0.0, 0.0]
        self.publisher.publish(msg)

        dir_str = "+Y" if self.direccion_positiva else "-Y"
        self.get_logger().info(f"Publicado movimiento {dir_str}: {msg.data}")

        self.ciclos_actuales += 1
        if self.ciclos_actuales >= self.ciclos_max:
            self.direccion_positiva = not self.direccion_positiva
            self.ciclos_actuales = 0  

def main(args=None):
    rclpy.init(args=args)
    nodo = PublicadorMovimientoY()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Nodo movimiento Y interrumpido por teclado.")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
        print("Programa movimiento Y finalizado.")

if __name__ == '__main__':
    main()

# nodo_publicador_prueba.py
# Publica cada segundo un punto y un ángulo en /punto_y_angulo
# Formato: [x, y, z, angulo_grados]

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PublicadorPrueba(Node):
    def __init__(self):
        super().__init__('publicador_prueba')
        self.pub = self.create_publisher(Float32MultiArray, '/punto_y_angulo', 10)
        # Publicar una vez
        msg = Float32MultiArray()
        msg.data = [2.37, -0.5, 1.0, 30.0]
        self.pub.publish(msg)
        self.get_logger().info(f"Publicado único: {msg.data}")

    def publicar(self):
        msg = Float32MultiArray()
        msg.data = [self.x, self.y, self.z, self.angulo]
        self.pub.publish(msg)
        self.get_logger().info(f"Publicado en /punto_y_angulo: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    nodo = PublicadorPrueba()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Nodo detenido por Ctrl+C")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class Visualizador2D(Node):
    def __init__(self, puerta_real_x, puerta_real_y):
        super().__init__('visualizador_2d')

        self.puerta_real = (puerta_real_x, puerta_real_y)

        self.historial_dron_x = []
        self.historial_dron_y = []
        
        self.primera_puerta_calculada = None
        self.puertas_recalculadas_x = []
        self.puertas_recalculadas_y = []

        self.sub_pose = self.create_subscription(
            Float32MultiArray,
            '/tello/pose_corregida',
            self.pose_callback,
            10
        )
        self.sub_puerta = self.create_subscription(
            Float32MultiArray,
            '/tello/punto_y_angulo',
            self.puerta_callback,
            10
        )

        plt.ion()
        self.fig, self.ax = plt.subplots()
        
        self.linea_dron, = self.ax.plot([], [], 'b-', label='Trayectoria Dron')
        self.ax.plot(self.puerta_real[0], self.puerta_real[1], 'go', markersize=15, label='Puerta Real')
        
        self.punto_primera_puerta, = self.ax.plot([], [], 'm*', markersize=15, label='Primera Detección') # Estrella Magenta
        
        self.puntos_recalculados, = self.ax.plot([], [], 'rx', markersize=7, label='Nuevas detecciones') # Cruces Rojas

        self.ax.set_xlabel("Eje X (Mundo)")
        self.ax.set_ylabel("Eje Y (Mundo)")
        self.ax.set_title("Visualización de Trayectoria 2D (XY)")
        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')

        self.timer = self.create_timer(0.5, self.actualizar_grafico)
        self.get_logger().info("Visualizador 2D iniciado.")

    def pose_callback(self, msg):
        if len(msg.data) >= 2:
            self.historial_dron_x.append(msg.data[0])
            self.historial_dron_y.append(msg.data[1])

    def puerta_callback(self, msg):
        if len(msg.data) >= 2:
            if self.primera_puerta_calculada is None:
                self.primera_puerta_calculada = (msg.data[0], msg.data[1])
                self.get_logger().info(f"Primera detección de puerta registrada en: {self.primera_puerta_calculada}")
            else:
                self.puertas_recalculadas_x.append(msg.data[0])
                self.puertas_recalculadas_y.append(msg.data[1])

    def actualizar_grafico(self):
        self.linea_dron.set_data(self.historial_dron_x, self.historial_dron_y)
        
        if self.primera_puerta_calculada:
            self.punto_primera_puerta.set_data([self.primera_puerta_calculada[0]], [self.primera_puerta_calculada[1]])
        
        self.puntos_recalculados.set_data(self.puertas_recalculadas_x, self.puertas_recalculadas_y)

        self.ax.relim()
        self.ax.autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    puerta_x = float(input("Introduce la coordenada X del punto P real de la puerta: "))
    puerta_y = float(input("Introduce la coordenada Y del punto P real de la puerta: "))
    rclpy.init(args=args)
    visualizador_node = Visualizador2D(puerta_x, puerta_y)
    try:
        rclpy.spin(visualizador_node)
    except KeyboardInterrupt:
        pass
    finally:
        visualizador_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class Visualizador2D(Node):
    def __init__(self, puerta_real_x, puerta_real_y):
        super().__init__('visualizador_2d')

        # Coordenadas de la puerta real
        self.puerta_real = (puerta_real_x, puerta_real_y)

        # Listas para almacenar el historial de la trayectoria
        self.historial_dron_x = []
        self.historial_dron_y = []
        self.puertas_calculadas_x = []
        self.puertas_calculadas_y = []

        # Suscriptores
        self.sub_pose = self.create_subscription(
            Float32MultiArray,
            '/tello/pose_corregida',
            self.pose_callback,
            10
        )
        self.sub_puerta = self.create_subscription(
            Float32MultiArray,
            '/tello/punto_y_angulo', # Usamos este para el punto P calculado
            self.puerta_callback,
            10
        )

        # Configuración del gráfico
        plt.ion() # Activar modo interactivo
        self.fig, self.ax = plt.subplots()
        self.linea_dron, = self.ax.plot([], [], 'b-', label='Trayectoria Dron') # Línea azul para el dron
        self.puntos_puertas_calc, = self.ax.plot([], [], 'rx', markersize=10, label='Puertas Calculadas (P)') # Cruces rojas
        
        self.ax.plot(self.puerta_real[0], self.puerta_real[1], 'go', markersize=15, label='Puerta Real') # Círculo verde grande
        
        self.ax.set_xlabel("Eje X (Mundo)")
        self.ax.set_ylabel("Eje Y (Mundo)")
        self.ax.set_title("Visualización de Trayectoria 2D (Vista Cenital)")
        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')

        # Temporizador para actualizar el gráfico
        self.timer = self.create_timer(0.5, self.actualizar_grafico)
        self.get_logger().info("Visualizador 2D iniciado.")

    def pose_callback(self, msg):
        if len(msg.data) >= 2:
            self.historial_dron_x.append(msg.data[0])
            self.historial_dron_y.append(msg.data[1])

    def puerta_callback(self, msg):
        if len(msg.data) >= 2:
            self.puertas_calculadas_x.append(msg.data[0])
            self.puertas_calculadas_y.append(msg.data[1])

    def actualizar_grafico(self):
        # Actualizar los datos de las líneas
        self.linea_dron.set_data(self.historial_dron_x, self.historial_dron_y)
        self.puntos_puertas_calc.set_data(self.puertas_calculadas_x, self.puertas_calculadas_y)

        # Reajustar los límites del gráfico para que todo sea visible
        self.ax.relim()
        self.ax.autoscale_view()

        # Forzar el redibujado
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    # Pedir al usuario las coordenadas de la puerta real
    try:
        puerta_x = float(input("Introduce la coordenada X del punto P real de la puerta: "))
        puerta_y = float(input("Introduce la coordenada Y del punto P real de la puerta: "))
    except ValueError:
        print("Entrada no válida. Usando (0,0) por defecto.")
        puerta_x, puerta_y = 0.0, 0.0

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
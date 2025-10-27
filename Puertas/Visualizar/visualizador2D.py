import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import math

class Visualizador2D(Node):
    def __init__(self, puerta_real_x, puerta_real_y):
        super().__init__('visualizador_2d')

        self.puerta_real = (puerta_real_x, puerta_real_y)

        self.historial_dron_x = []
        self.historial_dron_y = []
        self.primera_puerta_calculada = None
        self.primer_angulo_calculado_rad = None 
        self.puertas_recalculadas_x = []
        self.puertas_recalculadas_y = []
        
        self.posicion_inicial_dron = None
        self.se_ha_alejado = False
        self.UMBRAL_ALEJADO_M = 1.0 
        self.UMBRAL_RESET_M = 0.3

        self.sub_pose = self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.pose_callback, 10)
        self.sub_puerta = self.create_subscription(Float32MultiArray, '/tello/punto_y_angulo', self.puerta_callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.linea_dron, = self.ax.plot([], [], 'b-', label='Trayectoria dron', zorder=20)
        self.ax.plot(self.puerta_real[0], self.puerta_real[1], 'go', markersize=15, label='Puerta real')
        self.punto_primera_puerta, = self.ax.plot([], [], 'y*', markersize=15, label='Primera detección', zorder=10)         
        self.puntos_recalculados, = self.ax.plot([], [], 'rx', markersize=7, label='Nuevas detecciones')
        self.linea_angulo, = self.ax.plot([], [], 'c-', linewidth=2, label='Vector normal puerta')

        self.ax.set_xlabel("Eje X (Mundo) - Adelante/Atrás")
        self.ax.set_ylabel("Eje Y (Mundo) - Derecha/Izquierda")
        self.ax.set_title("Visualización de Trayectoria 2D (XY)")
        self.ax.legend()
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')

        self.timer = self.create_timer(0.5, self.actualizar_grafico)
        self.get_logger().info("Visualizador 2D iniciado.")

    def reset_plot(self):
        self.get_logger().info("Dron ha vuelto al inicio. Reiniciando gráfico")
        self.historial_dron_x.clear()
        self.historial_dron_y.clear()
        self.primera_puerta_calculada = None
        self.primer_angulo_calculado_rad = None
        self.puertas_recalculadas_x.clear()
        self.puertas_recalculadas_y.clear()
        self.posicion_inicial_dron = None
        self.se_ha_alejado = False

    def pose_callback(self, msg):
        if len(msg.data) >= 2:
            pos_actual = np.array([msg.data[0], msg.data[1]])
            
            if self.posicion_inicial_dron is None:
                self.posicion_inicial_dron = pos_actual

            dist_al_inicio = np.linalg.norm(pos_actual - self.posicion_inicial_dron)
            if not self.se_ha_alejado and dist_al_inicio > self.UMBRAL_ALEJADO_M:
                self.se_ha_alejado = True
            
            if self.se_ha_alejado and dist_al_inicio < self.UMBRAL_RESET_M:
                self.reset_plot()
                return

            self.historial_dron_x.append(pos_actual[0])
            self.historial_dron_y.append(pos_actual[1])

    def puerta_callback(self, msg):
        if len(msg.data) >= 4:
            if self.primera_puerta_calculada is None:
                self.primera_puerta_calculada = (msg.data[0], msg.data[1])
                self.primer_angulo_calculado_rad = math.radians(msg.data[3])
                self.get_logger().info(f"Primera detección: {self.primera_puerta_calculada}")
            else:
                self.puertas_recalculadas_x.append(msg.data[0])
                self.puertas_recalculadas_y.append(msg.data[1])

    def actualizar_grafico(self):
        self.linea_dron.set_data(self.historial_dron_x, self.historial_dron_y)
        
        if self.primera_puerta_calculada:
            px, py = self.primera_puerta_calculada
            self.punto_primera_puerta.set_data([px], [py])

            if self.primer_angulo_calculado_rad is not None:
                angulo = self.primer_angulo_calculado_rad
                
                longitud_media_vector = 0.5 
                dx = math.cos(angulo) * longitud_media_vector
                dy = math.sin(angulo) * longitud_media_vector
                
                linea_x = [px - dx, px + dx]
                linea_y = [py - dy, py + dy]
                self.linea_angulo.set_data(linea_x, linea_y)

        self.puntos_recalculados.set_data(self.puertas_recalculadas_x, self.puertas_recalculadas_y)

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    #puerta_x = float(input("Introduce la coordenada X del punto P real de la puerta: "))
    #puerta_y = float(input("Introduce la coordenada Y del punto P real de la puerta: "))
    puerta_x = 4.0
    puerta_y = 0.5

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
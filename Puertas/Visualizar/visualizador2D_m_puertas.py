import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import math

DISTANCIA_PROMEDIO_PUERTA = 1.0 

class Visualizador2D(Node):
    def __init__(self):
        super().__init__('visualizador_2d')

        self.historial_dron_x = []
        self.historial_dron_y = []
        self.puertas_detectadas = [] # Tuplas [(x, y), angulo]
        self.posicion_dron_actual = None
        self.yaw_dron_rad = 0.0
        self.altura_dron_z = 0.0

        self.ruta_planificada_x = []
        self.ruta_planificada_y = []
        self.puerta_objetivo_actual = None # (x, y, angulo)

        self.posicion_inicial_dron = None
        self.se_ha_alejado = False
        self.UMBRAL_ALEJADO_M = 1.0 
        self.UMBRAL_RESET_M = 0.3

        self.sub_pose = self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.pose_callback, 10)
        self.sub_deteccion = self.create_subscription(Float32MultiArray, '/tello/punto_y_angulo', self.deteccion_callback, 10)
        
        self.sub_ruta = self.create_subscription(Float32MultiArray, '/tello/lista_puertas', self.ruta_callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.subplots_adjust(right=0.75)

        # 1. Dron y Trayectoria Real
        self.linea_dron, = self.ax.plot([], [], 'b-', label='Vuelo Real', linewidth=1.5, zorder=20)
        self.linea_yaw_dron, = self.ax.plot([], [], 'r-', linewidth=2, zorder=25)

        # 2. Detecciones Crudas (Amarillo)
        self.puntos_puertas, = self.ax.plot([], [], 'y*', markersize=10, label='Detecciones', zorder=10)
        
        # 3. Ruta Planificada (Verde Discontinuo) - LA CURVA BÉZIER
        self.linea_ruta, = self.ax.plot([], [], 'g--', label='Ruta Bézier', linewidth=2, zorder=15)
        
        # 4. Puerta Objetivo Activa (Cuadrado Verde)
        self.plot_puerta_activa, = self.ax.plot([], [], 'gs', markersize=12, fillstyle='none', markeredgewidth=2, label='Meta Activa', zorder=16)

        self.altura_texto = self.ax.text(1.05, 0.95, '', transform=self.ax.transAxes, fontsize=12,
                                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        self.ax.set_xlabel("X (Mundo)")
        self.ax.set_ylabel("Y (Mundo)")
        self.ax.set_title("Navegación Bézier 2D")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.legend(loc='upper left', bbox_to_anchor=(1.05, 0.8))

        self.timer = self.create_timer(0.1, self.actualizar_grafico)
        self.get_logger().info("Visualizador 2D Bézier listo.")

    def reset_plot(self):
        self.get_logger().info("Reset gráfico.")
        self.historial_dron_x.clear()
        self.historial_dron_y.clear()
        self.puertas_detectadas.clear()
        self.ruta_planificada_x.clear()
        self.ruta_planificada_y.clear()
        self.posicion_inicial_dron = None
        self.se_ha_alejado = False
        self.posicion_dron_actual = None

    def pose_callback(self, msg):
        if len(msg.data) >= 6:
            pos_actual = np.array([msg.data[0], msg.data[1]])
            self.altura_dron_z = msg.data[2]
            self.posicion_dron_actual = pos_actual
            self.yaw_dron_rad = math.radians(msg.data[5])
            
            if self.posicion_inicial_dron is None:
                self.posicion_inicial_dron = pos_actual

            dist = np.linalg.norm(pos_actual - self.posicion_inicial_dron)
            if not self.se_ha_alejado and dist > self.UMBRAL_ALEJADO_M:
                self.se_ha_alejado = True
            
            if self.se_ha_alejado and dist < self.UMBRAL_RESET_M:
                self.reset_plot()
                return

            self.historial_dron_x.append(pos_actual[0])
            self.historial_dron_y.append(pos_actual[1])

    def deteccion_callback(self, msg):
        if len(msg.data) < 4: return
        punto = (msg.data[0], msg.data[1])
        angulo = math.radians(msg.data[3])
        
        actualizada = False
        for i, (p_ex, a_ex) in enumerate(self.puertas_detectadas):
            if math.dist(punto, p_ex) < DISTANCIA_PROMEDIO_PUERTA:                
                px = (p_ex[0] + punto[0]) / 2.0
                py = (p_ex[1] + punto[1]) / 2.0
                self.puertas_detectadas[i] = ((px, py), angulo) 
                actualizada = True
                break
        
        if not actualizada:
            self.puertas_detectadas.append((punto, angulo))

    def ruta_callback(self, msg):
        data = list(msg.data)
        if not data: return
        
        try:
            num_puntos = int(data[0])
            idx_fin = 1 + (num_puntos * 3)
            
            raw_puntos = data[1 : idx_fin]
            self.ruta_planificada_x = [raw_puntos[i] for i in range(0, len(raw_puntos), 3)]
            self.ruta_planificada_y = [raw_puntos[i+1] for i in range(0, len(raw_puntos), 3)]

            if len(data) >= idx_fin + 4:
                cx = data[idx_fin]
                cy = data[idx_fin+1]
                yaw = math.radians(data[idx_fin+3])
                self.puerta_objetivo_actual = (cx, cy, yaw)
            else:
                self.puerta_objetivo_actual = None

        except Exception as e:
            self.get_logger().warn(f"Error parseando ruta 2D: {e}")

    def actualizar_grafico(self):
        # 1. Dibujar Dron
        self.linea_dron.set_data(self.historial_dron_x, self.historial_dron_y)
        
        # 2. Dibujar Ruta Planificada (Bézier)
        if self.ruta_planificada_x:
            self.linea_ruta.set_data(self.ruta_planificada_x, self.ruta_planificada_y)
        
        # 3. Dibujar Puerta Activa (Meta)
        if self.puerta_objetivo_actual:
            cx, cy, yaw = self.puerta_objetivo_actual
            self.plot_puerta_activa.set_data([cx], [cy])

        # 5. Dibujar Orientación Dron
        if self.posicion_dron_actual is not None:
            px, py = self.posicion_dron_actual
            l = 0.4
            dx = math.cos(self.yaw_dron_rad) * l
            dy = math.sin(self.yaw_dron_rad) * l
            self.linea_yaw_dron.set_data([px, px + dx], [py, py + dy])

        self.altura_texto.set_text(f'Z: {self.altura_dron_z:.2f} m')

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    nodo = Visualizador2D()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
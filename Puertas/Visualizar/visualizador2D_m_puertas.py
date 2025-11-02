import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np
import math

# --- PARÁMETROS ---
# Distancia mínima para considerar una detección como una puerta existente y promediar
# Debería ser similar a DISTANCIA_NUEVA_PUERTA_M en nodoNavegacion.py
DISTANCIA_PROMEDIO_PUERTA = 1.0 # en metros

class Visualizador2D(Node):
    def __init__(self):
        super().__init__('visualizador_2d')

        # --- Datos para el gráfico ---
        self.historial_dron_x = []
        self.historial_dron_y = []
        self.puertas_detectadas = [] # Almacena tuplas: [(x, y), angulo_rad]
        self.posicion_dron_actual = None
        self.yaw_dron_rad = 0.0
        self.altura_dron_z = 0.0
        
        # Lógica para reiniciar el gráfico
        self.posicion_inicial_dron = None
        self.se_ha_alejado = False
        self.UMBRAL_ALEJADO_M = 1.0 
        self.UMBRAL_RESET_M = 0.3

        # --- Suscriptores ---
        self.sub_pose = self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.pose_callback, 10)
        self.sub_puerta = self.create_subscription(Float32MultiArray, '/tello/punto_y_angulo', self.puerta_callback, 10)

        # --- Configuración del Gráfico (Matplotlib) ---
        plt.ion()
        self.fig, self.ax = plt.subplots()
        
        self.fig.subplots_adjust(right=0.75)

        # --- Elementos del Gráfico ---
        self.linea_dron, = self.ax.plot([], [], 'b-', label='Trayectoria dron', zorder=20)
        self.puntos_puertas, = self.ax.plot([], [], 'y*', markersize=15, label='Puertas', zorder=10)
        self.lineas_angulos_puertas, = self.ax.plot([], [], 'c-', linewidth=2, label='Vector normal puerta')
        self.linea_yaw_dron, = self.ax.plot([], [], 'r-', linewidth=2, label='Yaw Dron', zorder=15)

        self.altura_texto = self.ax.text(1.05, 0.95, '', transform=self.ax.transAxes, fontsize=12,
                                        verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # --- Estilo del Gráfico ---
        self.ax.set_xlabel("Eje X (Mundo) - Adelante/Atrás")
        self.ax.set_ylabel("Eje Y (Mundo) - Derecha/Izquierda")
        self.ax.set_title("Visualización de Trayectoria 2D (XY)")
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')
        
        self.ax.legend(loc='upper left', bbox_to_anchor=(1.05, 0.8))

        self.timer = self.create_timer(0.1, self.actualizar_grafico)
        self.get_logger().info("Visualizador 2D iniciado. Esperando datos...")

    def reset_plot(self):
        self.get_logger().info("Dron ha vuelto al inicio. Reiniciando gráfico")
        self.historial_dron_x.clear()
        self.historial_dron_y.clear()
        self.puertas_detectadas.clear()
        self.posicion_inicial_dron = None
        self.se_ha_alejado = False
        self.posicion_dron_actual = None
        self.altura_dron_z = 0.0

    def pose_callback(self, msg):
        if len(msg.data) >= 6:
            pos_actual = np.array([msg.data[0], msg.data[1]])
            self.altura_dron_z = msg.data[2]
            self.posicion_dron_actual = pos_actual
            self.yaw_dron_rad = math.radians(msg.data[5])
            
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
        if len(msg.data) < 4:
            return

        punto_nuevo_xy = (msg.data[0], msg.data[1])
        angulo_nuevo_rad = math.radians(msg.data[3])
        
        puerta_actualizada = False
        # Buscamos si la puerta detectada ya está en nuestra lista
        for i, (punto_existente_xy, angulo_existente_rad) in enumerate(self.puertas_detectadas):
            distancia = math.dist(punto_nuevo_xy, punto_existente_xy)
            
            if distancia < DISTANCIA_PROMEDIO_PUERTA:                
                # 1. Promediar la posición
                px_promedio = (punto_existente_xy[0] + punto_nuevo_xy[0]) / 2.0
                py_promedio = (punto_existente_xy[1] + punto_nuevo_xy[1]) / 2.0
                punto_promedio = (px_promedio, py_promedio)

                # 2. Promediar el ángulo de forma robusta (usando vectores)
                avg_cos = (math.cos(angulo_existente_rad) + math.cos(angulo_nuevo_rad)) / 2.0
                avg_sin = (math.sin(angulo_existente_rad) + math.sin(angulo_nuevo_rad)) / 2.0
                angulo_promedio_rad = math.atan2(avg_sin, avg_cos)

                # 3. Actualizar la entrada en la lista
                self.puertas_detectadas[i] = (punto_promedio, angulo_promedio_rad)
                self.get_logger().info(f"Puerta en {punto_existente_xy} actualizada a {punto_promedio}")

                puerta_actualizada = True
                break # Salimos del bucle una vez que encontramos y actualizamos la puerta
        
        # Si después de recorrer toda la lista no encontramos una puerta cercana, es nueva
        if not puerta_actualizada:
            self.get_logger().info(f"Nueva puerta detectada y añadida: {punto_nuevo_xy}")
            self.puertas_detectadas.append((punto_nuevo_xy, angulo_nuevo_rad))

    def actualizar_grafico(self):
        self.linea_dron.set_data(self.historial_dron_x, self.historial_dron_y)
        
        if self.puertas_detectadas:
            # Desempaquetamos los datos de la lista de tuplas
            puertas_puntos = [p[0] for p in self.puertas_detectadas]
            puertas_angulos = [p[1] for p in self.puertas_detectadas]

            puertas_x = [p[0] for p in puertas_puntos]
            puertas_y = [p[1] for p in puertas_puntos]
            self.puntos_puertas.set_data(puertas_x, puertas_y)

            lineas_x_todas = []
            lineas_y_todas = []
            for punto, angulo in zip(puertas_puntos, puertas_angulos):
                px, py = punto
                longitud_vector = 0.5
                dx = math.cos(angulo) * longitud_vector
                dy = math.sin(angulo) * longitud_vector
                
                # Se dibuja una línea centrada en el punto de la puerta
                lineas_x_todas.extend([px - dx/2, px + dx/2, None])
                lineas_y_todas.extend([py - dy/2, py + dy/2, None])
            
            self.lineas_angulos_puertas.set_data(lineas_x_todas, lineas_y_todas)

        if self.posicion_dron_actual is not None:
            px, py = self.posicion_dron_actual
            longitud_vector = 0.4
            dx = math.cos(self.yaw_dron_rad) * longitud_vector
            dy = math.sin(self.yaw_dron_rad) * longitud_vector
            self.linea_yaw_dron.set_data([px, px + dx], [py, py + dy])

        self.altura_texto.set_text(f'Altura Dron (Z): {self.altura_dron_z:.2f} m')

        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    visualizador_node = Visualizador2D()
    try:
        rclpy.spin(visualizador_node)
    except KeyboardInterrupt:
        pass
    finally:
        visualizador_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
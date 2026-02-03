import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np

class VisualizadorPuertas(Node):
    def __init__(self):
        super().__init__('visualizador_puertas')

        self.sub_pose = self.create_subscription(
            Float32MultiArray, 
            '/tello/pose_corregida', 
            self.callback_pose, 
            10
        )
        self.sub_puertas = self.create_subscription(
            Float32MultiArray, 
            '/tello/lista_puertas', 
            self.callback_puertas, 
            10
        )

        self.pub_markers = self.create_publisher(MarkerArray, '/visualizador/marcadores', 10)

        self.pose_dron = None # [x, y, z, yaw]
        self.datos_puertas = [] 
        
        self.ancho_puerta = 0.7
        self.alto_puerta = 0.5

        self.timer = self.create_timer(0.1, self.bucle_visualizacion)
        self.get_logger().info("Visualizador de Puertas iniciado.")

    def callback_pose(self, msg):
        if len(msg.data) >= 6: 
             self.pose_dron = [msg.data[0], msg.data[1], msg.data[2], msg.data[5]]

    def callback_puertas(self, msg):
        self.datos_puertas = list(msg.data)

    def euler_a_quaternion(self, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

    def bucle_visualizacion(self):
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        id_counter = 0

        # --- 1. DIBUJAR EL DRON ---
        if self.pose_dron:
            m_dron = Marker()
            m_dron.header.frame_id = "map"
            m_dron.header.stamp = timestamp
            m_dron.ns = "dron"
            m_dron.id = id_counter
            id_counter += 1
            m_dron.type = Marker.ARROW
            m_dron.action = Marker.ADD
            
            m_dron.pose.position.x = float(self.pose_dron[0])
            m_dron.pose.position.y = float(self.pose_dron[1])
            m_dron.pose.position.z = float(self.pose_dron[2])
            
            q = self.euler_a_quaternion(math.radians(self.pose_dron[3]))
            m_dron.pose.orientation.x = q[0]
            m_dron.pose.orientation.y = q[1]
            m_dron.pose.orientation.z = q[2]
            m_dron.pose.orientation.w = q[3]
            
            m_dron.scale.x = 0.3; m_dron.scale.y = 0.1; m_dron.scale.z = 0.1
            m_dron.color.r = 1.0; m_dron.color.g = 1.0; m_dron.color.b = 0.0; m_dron.color.a = 1.0
            marker_array.markers.append(m_dron)

        # --- 2. DIBUJAR LAS PUERTAS ---
        # Estructura del bloque (19 floats):
        # [P0(3), P1(3), P2(3), P3(3), P4(3), Centro(3), Yaw(1)]
        TAMANO_BLOQUE = 19
        
        # Validación de seguridad para evitar errores si la lista está incompleta
        if len(self.datos_puertas) > 0:
            num_puertas = len(self.datos_puertas) // TAMANO_BLOQUE

            for i in range(num_puertas):
                base = i * TAMANO_BLOQUE
                
                # Extracción de datos
                p0 = self.datos_puertas[base:base+3]
                p1 = self.datos_puertas[base+3:base+6]
                p2 = self.datos_puertas[base+6:base+9]
                p3 = self.datos_puertas[base+9:base+12]
                p4 = self.datos_puertas[base+12:base+15]
                centro = self.datos_puertas[base+15:base+18]
                
                # --- CORRECCIÓN AQUÍ: Índice 18 es el elemento 19 ---
                yaw_deg = self.datos_puertas[base+18] 

                # A. Marco de la Puerta
                m_puerta = Marker()
                m_puerta.header.frame_id = "map"
                m_puerta.header.stamp = timestamp
                m_puerta.ns = "marcos_puerta"
                m_puerta.id = id_counter
                id_counter += 1
                m_puerta.type = Marker.CUBE
                m_puerta.action = Marker.ADD

                m_puerta.pose.position.x = float(centro[0])
                m_puerta.pose.position.y = float(centro[1])
                m_puerta.pose.position.z = float(centro[2])
                
                q_puerta = self.euler_a_quaternion(math.radians(yaw_deg))
                m_puerta.pose.orientation.x = q_puerta[0]
                m_puerta.pose.orientation.y = q_puerta[1]
                m_puerta.pose.orientation.z = q_puerta[2]
                m_puerta.pose.orientation.w = q_puerta[3]

                m_puerta.scale.x = 0.05  
                m_puerta.scale.y = self.ancho_puerta
                m_puerta.scale.z = self.alto_puerta

                m_puerta.color.r = 0.0; m_puerta.color.g = 0.5; m_puerta.color.b = 1.0; m_puerta.color.a = 0.5 
                marker_array.markers.append(m_puerta)

                # B. Puntos de Navegación (Ahora incluimos P0)
                puntos_nav = [p0, p1, p2, p3, p4]
                colores_puntos = [
                    (0.5, 0.0, 0.5), # P0 - Violeta (Inicio lejano)
                    (1.0, 0.0, 0.0), # P1 - Rojo
                    (1.0, 0.6, 0.0), # P2 - Naranja
                    (0.5, 1.0, 0.0), # P3 - Verde claro
                    (0.0, 1.0, 0.0)  # P4 - Verde fuerte
                ]

                for j, punto in enumerate(puntos_nav):
                    m_pt = Marker()
                    m_pt.header.frame_id = "map"
                    m_pt.header.stamp = timestamp
                    m_pt.ns = "puntos_nav"
                    m_pt.id = id_counter
                    id_counter += 1
                    m_pt.type = Marker.SPHERE
                    m_pt.action = Marker.ADD
                    
                    m_pt.pose.position.x = float(punto[0])
                    m_pt.pose.position.y = float(punto[1])
                    m_pt.pose.position.z = float(punto[2])
                    
                    m_pt.scale.x = 0.15; m_pt.scale.y = 0.15; m_pt.scale.z = 0.15
                    
                    c = colores_puntos[j]
                    m_pt.color.r = c[0]; m_pt.color.g = c[1]; m_pt.color.b = c[2]; m_pt.color.a = 1.0
                    marker_array.markers.append(m_pt)

        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    nodo = VisualizadorPuertas()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
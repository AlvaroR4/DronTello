import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math
import numpy as np

ALTURA_DESPEGUE = 0.72
ANCHO_PUERTA = 0.5
ALTO_PUERTA = 0.7
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
        
        self.estado_progreso = {} 

        self.timer = self.create_timer(0.1, self.bucle_visualizacion)
        self.get_logger().info("Visualizador RViz (Con Vector Inicio) iniciado.")

    def callback_pose(self, msg):
        if len(msg.data) >= 6: 
             self.pose_dron = [msg.data[0], msg.data[1], msg.data[2], msg.data[5]]

    def callback_puertas(self, msg):
        self.datos_puertas = list(msg.data)

    def euler_a_quaternion(self, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

    def calcular_distancia(self, p_dron, p_objetivo):
        return math.sqrt(
            (p_dron[0] - p_objetivo[0])**2 + 
            (p_dron[1] - p_objetivo[1])**2 + 
            (p_dron[2] - p_objetivo[2])**2
        )

    def bucle_visualizacion(self):
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        id_counter = 0

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
            m_dron.pose.orientation.x = q[0]; m_dron.pose.orientation.y = q[1]
            m_dron.pose.orientation.z = q[2]; m_dron.pose.orientation.w = q[3]
            
            m_dron.scale.x = 0.3; m_dron.scale.y = 0.1; m_dron.scale.z = 0.1
            m_dron.color.r = 1.0; m_dron.color.g = 1.0; m_dron.color.b = 0.0; m_dron.color.a = 1.0
            marker_array.markers.append(m_dron)

        TAMANO_BLOQUE = 19
        
        if len(self.datos_puertas) > 0:
            num_puertas = len(self.datos_puertas) // TAMANO_BLOQUE

            for i in range(num_puertas):
                base = i * TAMANO_BLOQUE
                
                if i not in self.estado_progreso:
                    self.estado_progreso[i] = [False, False, False, False, False, False]

                coords_puntos = [self.datos_puertas[base+k:base+k+3] for k in range(0, 15, 3)] 
                centro = self.datos_puertas[base+15:base+18]
                yaw_deg = self.datos_puertas[base+18]

                if self.pose_dron:
                    umbrales = [0.24, 0.144, 0.12, 0.12, 0.12] 
                    
                    for idx_pt, coord in enumerate(coords_puntos):
                        dist = self.calcular_distancia(self.pose_dron, coord)
                        if dist < umbrales[idx_pt]:
                            self.estado_progreso[i][idx_pt] = True 
                    
                    dist_centro = self.calcular_distancia(self.pose_dron, centro)
                    if dist_centro < 0.25:
                        self.estado_progreso[i][5] = True 

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
                m_puerta.pose.orientation.x = q_puerta[0]; m_puerta.pose.orientation.y = q_puerta[1]
                m_puerta.pose.orientation.z = q_puerta[2]; m_puerta.pose.orientation.w = q_puerta[3]
                
                m_puerta.scale.x = 0.05  
                m_puerta.scale.y = ANCHO_PUERTA 
                m_puerta.scale.z = ALTO_PUERTA 

                if self.estado_progreso[i][5]: 
                    m_puerta.color.r = 0.0; m_puerta.color.g = 0.8; m_puerta.color.b = 0.0; m_puerta.color.a = 0.8
                else:
                    m_puerta.color.r = 0.0; m_puerta.color.g = 0.5; m_puerta.color.b = 1.0; m_puerta.color.a = 0.4
                
                marker_array.markers.append(m_puerta)

                m_linea = Marker()
                m_linea.header.frame_id = "map"
                m_linea.header.stamp = timestamp
                m_linea.ns = "lineas_trayectoria"
                m_linea.id = id_counter
                id_counter += 1
                m_linea.type = Marker.LINE_STRIP
                m_linea.action = Marker.ADD
                m_linea.scale.x = 0.02
                m_linea.color.r = 1.0; m_linea.color.g = 1.0; m_linea.color.b = 1.0; m_linea.color.a = 0.5
                
                if i == 0:
                    p_inicio = Point()
                    p_inicio.x = 0.0
                    p_inicio.y = 0.0
                    p_inicio.z = float(ALTURA_DESPEGUE) 
                    m_linea.points.append(p_inicio)

                for coord in coords_puntos:
                    p = Point()
                    p.x, p.y, p.z = float(coord[0]), float(coord[1]), float(coord[2])
                    m_linea.points.append(p)
                
                marker_array.markers.append(m_linea)

                colores_pendientes = [
                    (0.5, 0.0, 0.5), 
                    (1.0, 0.0, 0.0), 
                    (1.0, 0.6, 0.0), 
                    (0.4, 0.4, 0.4), 
                    (0.6, 0.3, 0.0)
                ]
                color_verde_claro = (0.3, 1.0, 0.3) 

                for j, punto in enumerate(coords_puntos):
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
                    
                    if self.estado_progreso[i][j]: 
                        c = color_verde_claro
                        m_pt.color.a = 1.0
                    else:
                        c = colores_pendientes[j]
                        m_pt.color.a = 1.0 
                    
                    m_pt.color.r = c[0]; m_pt.color.g = c[1]; m_pt.color.b = c[2]
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
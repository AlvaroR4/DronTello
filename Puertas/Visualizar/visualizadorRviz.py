import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math

ALTO_PUERTA = 0.7
ANCHO_PUERTA = 0.5

class VisualizadorPuertas(Node):
    def __init__(self):
        super().__init__('visualizador_puertas')

        self.sub_pose = self.create_subscription(
            Float32MultiArray, '/tello/pose_corregida', self.callback_pose, 10
        )
        self.sub_puertas = self.create_subscription(
            Float32MultiArray, '/tello/lista_puertas', self.callback_ruta, 10
        )

        self.pub_markers = self.create_publisher(MarkerArray, '/visualizador/marcadores', 10)
        self.pose_dron = None 
        
        self.rutas_guardadas = [] 
        
        self.timer = self.create_timer(0.1, self.bucle_visualizacion)

    def callback_pose(self, msg):
        if len(msg.data) >= 6: 
             self.pose_dron = [msg.data[0], msg.data[1], msg.data[2], msg.data[5]]

    def callback_ruta(self, msg):
        datos = list(msg.data)
        if len(datos) < 1: return

        try:
            num_puntos = int(datos[0])
            idx_fin_puntos = 1 + (num_puntos * 3)
            
            raw_puntos = datos[1 : idx_fin_puntos]
            ruta_points = []
            for k in range(0, len(raw_puntos), 3):
                ruta_points.append([raw_puntos[k], raw_puntos[k+1], raw_puntos[k+2]])

            if len(datos) >= idx_fin_puntos + 4:
                centro = datos[idx_fin_puntos : idx_fin_puntos+3]
                yaw_deg = datos[idx_fin_puntos+3]

                if abs(centro[0]) < 0.001 and abs(centro[1]) < 0.001 and abs(centro[2]) < 0.001:
                    return

                nueva_entrada = {
                    'centro': centro,
                    'yaw': yaw_deg,
                    'puntos': ruta_points
                }

                if not self.rutas_guardadas:
                    self.rutas_guardadas.append(nueva_entrada)
                else:
                    ultimo = self.rutas_guardadas[-1]
                    dist = math.sqrt(
                        (ultimo['centro'][0] - centro[0])**2 +
                        (ultimo['centro'][1] - centro[1])**2 +
                        (ultimo['centro'][2] - centro[2])**2
                    )

                    if dist > 0.5:
                        self.rutas_guardadas.append(nueva_entrada)
                    else:
                        self.rutas_guardadas[-1] = nueva_entrada

        except Exception as e:
            self.get_logger().error(f"Error parseando ruta: {e}")

    def euler_a_quaternion(self, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

    def bucle_visualizacion(self):
        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()
        id_counter = 0

        if self.pose_dron:
            m_dron = Marker()
            m_dron.header.frame_id = "map"; m_dron.header.stamp = timestamp
            m_dron.ns = "dron"; m_dron.id = id_counter; id_counter += 1
            m_dron.type = Marker.ARROW; m_dron.action = Marker.ADD
            m_dron.pose.position.x = float(self.pose_dron[0])
            m_dron.pose.position.y = float(self.pose_dron[1])
            m_dron.pose.position.z = float(self.pose_dron[2])
            q = self.euler_a_quaternion(math.radians(self.pose_dron[3]))
            m_dron.pose.orientation.x = q[0]; m_dron.pose.orientation.y = q[1]
            m_dron.pose.orientation.z = q[2]; m_dron.pose.orientation.w = q[3]
            m_dron.scale.x = 0.3; m_dron.scale.y = 0.05; m_dron.scale.z = 0.05
            m_dron.color.r = 1.0; m_dron.color.g = 1.0; m_dron.color.b = 0.0; m_dron.color.a = 1.0
            marker_array.markers.append(m_dron)
        
        for i, ruta in enumerate(self.rutas_guardadas):
            centro = ruta['centro']
            yaw_deg = ruta['yaw']
            puntos = ruta['puntos']

            m_puerta = Marker()
            m_puerta.header.frame_id = "map"; m_puerta.header.stamp = timestamp
            m_puerta.ns = "marcos_puerta"
            m_puerta.id = id_counter; id_counter += 1 # ID Único
            m_puerta.type = Marker.CUBE; m_puerta.action = Marker.ADD
            m_puerta.pose.position.x = float(centro[0])
            m_puerta.pose.position.y = float(centro[1])
            m_puerta.pose.position.z = float(centro[2])
            
            q_puerta = self.euler_a_quaternion(math.radians(yaw_deg))
            m_puerta.pose.orientation.x = q_puerta[0]; m_puerta.pose.orientation.y = q_puerta[1]
            m_puerta.pose.orientation.z = q_puerta[2]; m_puerta.pose.orientation.w = q_puerta[3]
            m_puerta.scale.x = 0.05; m_puerta.scale.y = ANCHO_PUERTA; m_puerta.scale.z = ALTO_PUERTA
            
            es_ultima = (i == len(self.rutas_guardadas) - 1)
            if es_ultima:
                m_puerta.color.r = 0.0; m_puerta.color.g = 1.0; m_puerta.color.b = 0.0; m_puerta.color.a = 0.6
            else:
                m_puerta.color.r = 0.5; m_puerta.color.g = 0.5; m_puerta.color.b = 0.5; m_puerta.color.a = 0.3 # Grisáceo

            marker_array.markers.append(m_puerta)

            m_linea = Marker()
            m_linea.header.frame_id = "map"; m_linea.header.stamp = timestamp
            m_linea.ns = "trayectoria_bezier"
            m_linea.id = id_counter; id_counter += 1
            m_linea.type = Marker.LINE_STRIP; m_linea.action = Marker.ADD
            m_linea.scale.x = 0.03 
            
            if es_ultima:
                m_linea.color.r = 0.0; m_linea.color.g = 1.0; m_linea.color.b = 1.0; m_linea.color.a = 0.8 # Cyan vivo
            else:
                m_linea.color.r = 0.0; m_linea.color.g = 0.5; m_linea.color.b = 0.5; m_linea.color.a = 0.4 # Cyan apagado

            for p in puntos:
                pt = Point()
                pt.x = float(p[0]); pt.y = float(p[1]); pt.z = float(p[2])
                m_linea.points.append(pt)
            marker_array.markers.append(m_linea)

            for p in puntos:
                m_pt = Marker()
                m_pt.header.frame_id = "map"; m_pt.header.stamp = timestamp
                m_pt.ns = "puntos_nav"
                m_pt.id = id_counter; id_counter += 1
                m_pt.type = Marker.SPHERE; m_pt.action = Marker.ADD
                m_pt.pose.position.x = float(p[0]); m_pt.pose.position.y = float(p[1]); m_pt.pose.position.z = float(p[2])
                m_pt.scale.x = 0.05; m_pt.scale.y = 0.05; m_pt.scale.z = 0.05
                
                if es_ultima:
                    m_pt.color.r = 1.0; m_pt.color.g = 0.5; m_pt.color.b = 0.0; m_pt.color.a = 0.8
                else:
                    m_pt.color.r = 1.0; m_pt.color.g = 0.5; m_pt.color.b = 0.0; m_pt.color.a = 0.2

                marker_array.markers.append(m_pt)

        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    nodo = VisualizadorPuertas()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally: nodo.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()
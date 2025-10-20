import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

DISTANCIA_APROXIMACION_M = 0.50
DISTANCIA_SALIDA_M = 0.50
MARGEN_ALTURA_M = 0.50
AUMENTAR_VELOCIDAD = 0.8
AUMENTAR_YAW = 1.0
VELOCIDAD_MAXIMA = 20
DISTANCIA_NUEVA_PUERTA_M = 1.0
LLEGADA_POSICION_M = 0.10
LLEGADA_YAW_DEG = 5.0

def rango_velocidad(valor, maximo):
    return np.clip(valor, -maximo, maximo)

def normalizar_angulo_deg(angulo):
    return (angulo + 180.0) % 360.0 - 180.0

class NodoNavegacion(Node):
    def __init__(self):
        super().__init__('nodo_navegacion')

        self.pose_recibida = False
        self.posicion_dron_mundo = np.zeros(3)
        self.yaw_dron_deg = 0.0
        self.estado_mision = 'ESPERANDO_PUERTA'
        self.mision_en_curso = False
        self.puertas_visitadas = []
        self.puntos_trayectoria_actual = []
        self.angulo_objetivo_deg = 0.0

        self.sub_pose = self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.callback_pose_dron, 10)
        self.sub_puerta = self.create_subscription(Float32MultiArray, '/tello/punto_y_angulo', self.callback_nueva_puerta, 10)
        self.pub_velocidad = self.create_publisher(Float32MultiArray, '/tello/comandos_velocidad', 10)

        qos_marcadores = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_marcadores = self.create_publisher(MarkerArray, '/trayectoria_visual', qos_marcadores)

        self.timer_control = self.create_timer(1.0 / 20.0, self.bucle_de_control)
        self.get_logger().info("Nodo de Navegación iniciado. Esperando detección de puertas.")

    def callback_pose_dron(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.posicion_dron_mundo = np.array(msg.data[0:3])
            self.yaw_dron_deg = msg.data[5]
            self.pose_recibida = True

    def callback_nueva_puerta(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        punto_puerta_recibido = np.array(msg.data[0:3])
        
        es_puerta_nueva = True
        for puerta_visitada in self.puertas_visitadas:
            #distancia euclidiana
            if np.linalg.norm(punto_puerta_recibido - puerta_visitada) < DISTANCIA_NUEVA_PUERTA_M:
                es_puerta_nueva = False
                break
        
        if not es_puerta_nueva:
            return

        self.get_logger().info(f"Nueva puerta detectada en {punto_puerta_recibido}")
        self.puertas_visitadas.append(punto_puerta_recibido)
        
        punto_central_puerta = punto_puerta_recibido
        self.angulo_objetivo_deg = float(msg.data[3])
        angulo_objetivo_rad = math.radians(self.angulo_objetivo_deg)

        punto_central_puerta[2] -= MARGEN_ALTURA_M
        normal_plano_puerta = np.array([math.cos(angulo_objetivo_rad), math.sin(angulo_objetivo_rad), 0.0])
        
        punto_aproximacion = punto_central_puerta - DISTANCIA_APROXIMACION_M * normal_plano_puerta
        punto_salida = punto_central_puerta + DISTANCIA_SALIDA_M * normal_plano_puerta
        self.puntos_trayectoria_actual = [punto_aproximacion, punto_central_puerta, punto_salida]
        
        self.publicar_marcadores_rviz()
        
        self.estado_mision = 'IR_A_PUNTO_APROXIMACION'
        self.mision_en_curso = True
        self.get_logger().info(f"Iniciando trayectoria. Próximo objetivo: {punto_aproximacion}")

    def bucle_de_control(self):
        if not self.pose_recibida or not self.mision_en_curso:
            return

        if self.estado_mision == 'IR_A_PUNTO_APROXIMACION':
            objetivo = self.puntos_trayectoria_actual[0]
            if self.ir_a_posicion(objetivo):
                self.get_logger().info("Punto de aproximación alcanzado -> Rotando")
                self.estado_mision = 'ROTAR_HACIA_PUERTA'

        elif self.estado_mision == 'ROTAR_HACIA_PUERTA':
            if self.rotar_a_yaw(self.angulo_objetivo_deg):
                self.get_logger().info("Rotación completada -> Cruzando puerta")
                self.estado_mision = 'IR_A_PUNTO_SALIDA'

        elif self.estado_mision == 'IR_A_PUNTO_SALIDA':
            objetivo = self.puntos_trayectoria_actual[2]
            if self.ir_a_posicion(objetivo, yaw_objetivo_deg=self.angulo_objetivo_deg):
                self.get_logger().info("Puerta cruzada")
                self.estado_mision = 'ESPERANDO_PUERTA'
                self.mision_en_curso = False
                self.detener_dron()
                self.publicar_marcadores_rviz(borrar=True)

    def ir_a_posicion(self, punto_objetivo, yaw_objetivo_deg=None):
        error_posicion = punto_objetivo - self.posicion_dron_mundo
        distancia_al_objetivo = np.linalg.norm(error_posicion)
        
        velocidad_deseada = AUMENTAR_VELOCIDAD * error_posicion

        #Ejes Mundo: +X (Adelante), +Y (Derecha), +Z (Abajo)
        #Comandos Dron: avance (Adelante), lateral (Derecha), vertical (Arriba)
        avance = rango_velocidad(velocidad_deseada[0] * 100, VELOCIDAD_MAXIMA)
        lateral      = rango_velocidad(velocidad_deseada[1] * 100, VELOCIDAD_MAXIMA)
        vertical     = rango_velocidad(-velocidad_deseada[2] * 100, VELOCIDAD_MAXIMA)#comprobar signo
        
        cmd_rotacion = 0
        if yaw_objetivo_deg is not None:
            error_yaw = normalizar_angulo_deg(yaw_objetivo_deg - self.yaw_dron_deg)
            cmd_rotacion = rango_velocidad(AUMENTAR_YAW * error_yaw, VELOCIDAD_MAXIMA)

        self.enviar_comando_velocidad(lateral, avance, vertical, cmd_rotacion)
        return distancia_al_objetivo < LLEGADA_POSICION_M

    def rotar_a_yaw(self, angulo_objetivo_deg):
        error_yaw = normalizar_angulo_deg(angulo_objetivo_deg - self.yaw_dron_deg)
        
        if abs(error_yaw) < LLEGADA_YAW_DEG:
            self.detener_dron()
            return True
        
        cmd_rotacion = rango_velocidad(AUMENTAR_YAW * error_yaw, VELOCIDAD_MAXIMA)
        self.enviar_comando_velocidad(0, 0, 0, cmd_rotacion)
        return False

    def enviar_comando_velocidad(self, lr, fb, ud, yv):
        msg = Float32MultiArray(data=[float(lr), float(fb), float(ud), float(yv)])
        self.pub_velocidad.publish(msg)

    def detener_dron(self):
        self.enviar_comando_velocidad(0, 0, 0, 0)

    def publicar_marcadores_rviz(self, borrar=False):
        marker_array = MarkerArray()
        colores = [[1.0,1.0,0.0,0.8], [1.0,0.0,0.0,0.8], [0.0,1.0,0.0,0.8]]

        for i, punto in enumerate(self.puntos_trayectoria_actual):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "puntos_trayectoria"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.DELETE if borrar else Marker.ADD
            
            marker.pose.position.x = float(punto[0])
            marker.pose.position.y = float(punto[1])
            marker.pose.position.z = float(punto[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
            
            if not borrar:
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = colores[i]

            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
            marker_array.markers.append(marker)
        
        self.pub_marcadores.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    nodo_navegacion = NodoNavegacion()
    try:
        rclpy.spin(nodo_navegacion)
    except KeyboardInterrupt:
        nodo_navegacion.get_logger().info("Ctrl+C")
    finally:
        nodo_navegacion.detener_dron()
        nodo_navegacion.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

DISTANCIA_APROXIMACION_M = 0.40
DISTANCIA_SALIDA_M = 1.5
MARGEN_ALTURA_M = 0.0
AUMENTAR_VELOCIDAD = 100.0
AUMENTAR_YAW = 4.0
VELOCIDAD_MAXIMA = 10
VELOCIDAD_MAXIMA_YAW = 7.0
DISTANCIA_NUEVA_PUERTA_M = 1.0
ERROR_POSICION_M = 0.15
ERROR_YAW_DEG = 5.0

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
        self.puntos_trayectoria_actual = []
        self.angulo_objetivo_deg = 0.0
        self.puerta_para_promediar = None
        self.primera_puerta = True

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
        self.enviar_comando_despegue()
        self.get_logger().info("Nodo de Navegación iniciado. Esperando detección de puertas.")
    
    def enviar_comando_despegue(self):
        self.get_logger().info("Enviando comando de despegue")
        comando_despegue = [2.0, 2.0, 2.0, 0.0]
        msg = Float32MultiArray(data=[float(val) for val in comando_despegue])
        self.pub_velocidad.publish(msg)

    def callback_pose_dron(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.posicion_dron_mundo = np.array(msg.data[0:3])
            self.yaw_dron_deg = msg.data[5]
            self.pose_recibida = True
    
    def promediar_angulos_deg(self, angulo_actual, nuevo_angulo, n_actual, n_nuevo):
        rad_actual = math.radians(angulo_actual)
        rad_nuevo = math.radians(nuevo_angulo)
        
        x_total = (math.cos(rad_actual) * n_actual) + (math.cos(rad_nuevo) * n_nuevo)
        y_total = (math.sin(rad_actual) * n_actual) + (math.sin(rad_nuevo) * n_nuevo)
        
        angulo_promedio_rad = math.atan2(y_total, x_total)
        return math.degrees(angulo_promedio_rad)

    def callback_nueva_puerta(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        punto_puerta_recibido = np.array(msg.data[0:3])
        punto_puerta_recibido[2] -= MARGEN_ALTURA_M
        angulo_recibido_deg = float(msg.data[3])
        
        if self.puerta_para_promediar is None:
            self.puerta_para_promediar = {
                'punto': punto_puerta_recibido,
                'angulo': angulo_recibido_deg,
                'n_muestras': 1
            }
            self.get_logger().info("Nueva puerta detectada. Empezando a promediar.")
        else:
            dist = np.linalg.norm(self.puerta_para_promediar['punto'] - punto_puerta_recibido)
            if dist < DISTANCIA_NUEVA_PUERTA_M and self.puerta_para_promediar['n_muestras'] < 40:
                n = self.puerta_para_promediar['n_muestras']
                punto_actual = self.puerta_para_promediar['punto']
                angulo_actual = self.puerta_para_promediar['angulo']

                nuevo_punto = (punto_actual * n + punto_puerta_recibido) / (n + 1)
                nuevo_angulo = self.promediar_angulos_deg(angulo_actual, angulo_recibido_deg, n, 1)

                self.puerta_para_promediar['punto'] = nuevo_punto
                self.puerta_para_promediar['angulo'] = nuevo_angulo
                self.puerta_para_promediar['n_muestras'] += 1
                self.get_logger().info(f"Puerta modificada, muestra {n+1}")
        
        punto_central_puerta = self.puerta_para_promediar['punto']
        self.angulo_objetivo_deg = self.puerta_para_promediar['angulo']
        angulo_objetivo_rad = math.radians(self.angulo_objetivo_deg)

        normal_plano_puerta = np.array([math.cos(angulo_objetivo_rad), math.sin(angulo_objetivo_rad), 0.0])
        
        punto_aproximacion = punto_central_puerta - DISTANCIA_APROXIMACION_M * normal_plano_puerta
        punto_salida = punto_central_puerta + DISTANCIA_SALIDA_M * normal_plano_puerta
        self.puntos_trayectoria_actual = [punto_aproximacion, punto_central_puerta, punto_salida]
        
        self.publicar_marcadores_rviz()
        if(self.primera_puerta):
            self.estado_mision = 'IR_A_PUNTO_APROXIMACION'
            self.mision_en_curso = True
            self.primera_puerta = False
            self.get_logger().info(f"Iniciando trayectoria. Próximo objetivo: {punto_aproximacion}")

    def bucle_de_control(self):
        if not self.pose_recibida or not self.mision_en_curso:
            self.detener_dron()
            return

        if self.estado_mision == 'IR_A_PUNTO_APROXIMACION':
            objetivo = self.puntos_trayectoria_actual[0]
            if self.ir_a_posicion(objetivo):
                self.get_logger().info("Punto de aproximación alcanzado -> Punto salida")
                self.estado_mision = 'IR_A_PUNTO_SALIDA'

        elif self.estado_mision == 'ROTAR_HACIA_PUERTA':
            if self.rotar_a_yaw(self.angulo_objetivo_deg):
                self.get_logger().info("Rotación completada -> Cruzando puerta")
                self.estado_mision = 'IR_A_PUNTO_SALIDA'

        if self.estado_mision == 'IR_A_PUNTO_SALIDA':
            objetivo = self.puntos_trayectoria_actual[2]
            if self.ir_a_posicion(objetivo):
                self.get_logger().info("Punto de salida -> Punto inicio")
                self.get_logger().info("Puerta cruzada")
                self.estado_mision = 'IR_P1'

        if self.estado_mision == 'IR_P1':
            objetivo = self.puntos_trayectoria_actual[0]
            if self.ir_a_posicion(objetivo):
                self.get_logger().info("Punto de inicio -> 0,0,0")
                self.estado_mision = 'IR_P0'

        elif self.estado_mision == 'IR_P0':
            objetivo = (0.0,0.0,0.0)
            if self.ir_a_posicion(objetivo):
                self.get_logger().info("FIN")
                self.enviar_comando_velocidad(2,0,0,0)#land
                self.estado_mision = 'ESPERANDO_PUERTA'
                self.mision_en_curso = False
                self.detener_dron()
                self.publicar_marcadores_rviz(borrar=True)


    def ir_a_posicion(self, punto_objetivo, yaw_objetivo_deg=None):
        rotacion = 0
        velocidad_deseada = punto_objetivo - self.posicion_dron_mundo
        distancia_al_objetivo = np.linalg.norm(velocidad_deseada)

        avance = velocidad_deseada[0]
        lateral = velocidad_deseada[1]
        vertical = velocidad_deseada[2]

        distancia_maxima = max(abs(avance),abs(lateral),abs(vertical))
        if distancia_maxima > 0:
            velocidad_maxima_eje = distancia_maxima * AUMENTAR_VELOCIDAD
            if velocidad_maxima_eje > VELOCIDAD_MAXIMA:
                factor_aumento = VELOCIDAD_MAXIMA / distancia_maxima
            else: 
                factor_aumento = AUMENTAR_VELOCIDAD
            avance *= factor_aumento
            lateral *= factor_aumento
            vertical *= -factor_aumento

            if yaw_objetivo_deg is not None:
                rotacion = normalizar_angulo_deg(yaw_objetivo_deg - self.yaw_dron_deg)
                if abs(rotacion) > VELOCIDAD_MAXIMA_YAW:
                    if rotacion < 0:
                        rotacion = -VELOCIDAD_MAXIMA_YAW
                    else: 
                        rotacion = VELOCIDAD_MAXIMA_YAW
        #self.detener_dron()                
        self.enviar_comando_velocidad(lateral, avance, vertical, rotacion)
        return distancia_al_objetivo < ERROR_POSICION_M

    def rotar_a_yaw(self, angulo_objetivo_deg):
        error_yaw = normalizar_angulo_deg(angulo_objetivo_deg - self.yaw_dron_deg)
        
        if abs(error_yaw) < ERROR_YAW_DEG:
            self.detener_dron()
            return True
        
        rotacion = rango_velocidad(AUMENTAR_YAW * error_yaw, VELOCIDAD_MAXIMA_YAW)
        self.enviar_comando_velocidad(0, 0, 0, rotacion)
        return False

    def enviar_comando_velocidad(self, lr, fb, ud, yv):
        msg = Float32MultiArray(data=[float(lr), float(fb), float(ud), float(yv)])
        #msg = Float32MultiArray(data=[0,0,0,0])
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

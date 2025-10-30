import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

DISTANCIA_APROXIMACION_M = 0.40 #distancia P'
DISTANCIA_SALIDA_M = 0.4 #distancia P''
MARGEN_ALTURA_M = 0.50 #aumentar altura del P
AUMENTAR_VELOCIDAD = 30.0
AUMENTAR_YAW = 1.0
VELOCIDAD_MAXIMA_YAW = 5.0 
VELOCIDAD_MAXIMA = 20
DISTANCIA_NUEVA_PUERTA_M = 1.0 #para que un P sea admitido como nueva puerta
ERROR_POSICION_M = 0.10 #margen error para considerar que estas un un punto
ERROR_YAW_DEG = 5.0 #margen error yaw

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
        self.puertas_visitadas = []#lista de puertas detectadas
        self.cola_puertas = []#lista de puertas a navegar, primero punto, siguiente valor su angulo
        self.puntos_trayectoria_actual = []
        self.minimo_una_puerta = False #para que minimo atraviese una puerta 

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
        msg = Float32MultiArray(data=[2.0, 2.0, 2.0, 0.0])
        self.pub_velocidad.publish(msg)

    def callback_pose_dron(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.posicion_dron_mundo = np.array(msg.data[0:3])
            self.yaw_dron_deg = msg.data[5]
            self.pose_recibida = True

    def callback_nueva_puerta(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        punto_puerta_recibido = np.array(msg.data[0:3])
        angulo_recibido_deg = float(msg.data[3])

        for puerta_visitada in self.puertas_visitadas:
            if np.linalg.norm(punto_puerta_recibido - puerta_visitada) < DISTANCIA_NUEVA_PUERTA_M:
                self.get_logger().info(f"Puerta en {punto_puerta_recibido} ya ha sido visitada; ignorada")
                return
            
        if self.mision_en_curso:
            punto_central_actual = self.puntos_trayectoria_actual[1]
            if np.linalg.norm(punto_puerta_recibido - punto_central_actual) < DISTANCIA_NUEVA_PUERTA_M:
                self.get_logger().info("Calculando nueva trayectoria")

                angulo_actual_deg = self.puntos_trayectoria_actual[3]
                
                punto_central_nuevo = (punto_central_actual + punto_puerta_recibido) / 2.0

                for i, puerta_visitada in enumerate(self.puertas_visitadas):
                    if np.array_equal(puerta_visitada, punto_central_actual):
                        self.puertas_visitadas[i] = punto_central_nuevo
                        break

                angulo_nuevo_deg = (angulo_actual_deg + angulo_recibido_deg) / 2.0
                angulo_nuevo_rad = math.radians(angulo_nuevo_deg)

                normal_plano_puerta = np.array([math.cos(angulo_nuevo_rad), math.sin(angulo_nuevo_rad), 0.0])
                punto_aproximacion_nuevo = punto_central_nuevo - DISTANCIA_APROXIMACION_M * normal_plano_puerta
                punto_salida_nuevo = punto_central_nuevo + DISTANCIA_SALIDA_M * normal_plano_puerta

                self.puntos_trayectoria_actual = [
                    punto_aproximacion_nuevo,
                    punto_central_nuevo,
                    punto_salida_nuevo,
                    angulo_nuevo_deg
                ]

                self.publicar_marcadores_rviz()
                return

        for i in range(0, len(self.cola_puertas), 2):
            punto_en_cola = self.cola_puertas[i]
            if np.linalg.norm(punto_puerta_recibido - punto_en_cola) < DISTANCIA_NUEVA_PUERTA_M:
                self.get_logger().info(f"Actualizando una puerta en la cola.")
                
                nuevo_punto_promedio = (punto_en_cola + punto_puerta_recibido) / 2.0
                self.cola_puertas[i] = nuevo_punto_promedio

                angulo_en_cola_deg = self.cola_puertas[i+1]
                nuevo_angulo_promedio = (angulo_en_cola_deg + angulo_recibido_deg) / 2.0
                self.cola_puertas[i+1] = nuevo_angulo_promedio
                
                return 

        self.get_logger().info(f"Nueva puerta añadida a la cola en {punto_puerta_recibido}")
        self.cola_puertas.append(punto_puerta_recibido)
        self.cola_puertas.append(angulo_recibido_deg)

    def bucle_de_control(self):
        if not self.pose_recibida:
            return

        if not self.mision_en_curso and len(self.cola_puertas) > 0:
            self.get_logger().info("Tarea finalizada. Buscando siguiente puerta en la cola")
            punto_central_puerta = self.cola_puertas.pop(0)
            angulo_objetivo_deg = self.cola_puertas.pop(0)

            self.puertas_visitadas.append(punto_central_puerta)
            
            angulo_objetivo_rad = math.radians(angulo_objetivo_deg)

            punto_central_puerta[2] -= MARGEN_ALTURA_M
            normal_plano_puerta = np.array([math.cos(angulo_objetivo_rad), math.sin(angulo_objetivo_rad), 0.0])
            
            punto_aproximacion = punto_central_puerta - DISTANCIA_APROXIMACION_M * normal_plano_puerta
            punto_salida = punto_central_puerta + DISTANCIA_SALIDA_M * normal_plano_puerta
            self.puntos_trayectoria_actual = [punto_aproximacion, punto_central_puerta, punto_salida, angulo_objetivo_deg]
            
            self.publicar_marcadores_rviz()
            
            self.estado_mision = 'IR_A_PUNTO_APROXIMACION'
            self.mision_en_curso = True
            self.get_logger().info(f"Iniciando trayectoria hacia {punto_central_puerta}. Próximo objetivo: {punto_aproximacion}")
            return # Salimos para que el siguiente ciclo ya procese el estado

        # Si no hay misión en curso y la cola está vacía, no hacemos nada más
        if not self.mision_en_curso:
            if self.minimo_una_puerta:
                self.get_logger().info("Cola de puertas vacía. Aterrizando.")
                self.enviar_comando_velocidad(2, 0, 0, 0) # land
                rclpy.shutdown()
            return

        if self.estado_mision == 'IR_A_PUNTO_APROXIMACION':
            objetivo = self.puntos_trayectoria_actual[0]
            if self.ir_a_posicion(objetivo):
                self.get_logger().info("Punto de aproximación alcanzado -> Rotando")
                self.estado_mision = 'ROTAR_HACIA_PUERTA'

        elif self.estado_mision == 'ROTAR_HACIA_PUERTA':
            if self.rotar_a_yaw(self.puntos_trayectoria_actual[3]):
                self.get_logger().info("Rotación completada -> Cruzando puerta")
                self.estado_mision = 'IR_A_PUNTO_SALIDA'

        elif self.estado_mision == 'IR_A_PUNTO_SALIDA':
            objetivo = self.puntos_trayectoria_actual[2]
            if self.ir_a_posicion(objetivo):
                self.get_logger().info("Puerta cruzada.")
                self.mision_en_curso = False 
                self.minimo_una_puerta = True
                self.detener_dron()
                self.publicar_marcadores_rviz(borrar=True)

    def ir_a_posicion(self, punto_objetivo, yaw_objetivo_deg=None):
        #Ejes Mundo: +X (Adelante), +Y (Derecha), +Z (Abajo)
        #Comandos Dron: avance (Adelante), lateral (Derecha), vertical (Arriba)
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

            #Preparado para moverse a un punto y rotar a la vez en un futuro
            if yaw_objetivo_deg is not None:
                rotacion = normalizar_angulo_deg(yaw_objetivo_deg - self.yaw_dron_deg)
                if abs(rotacion) > VELOCIDAD_MAXIMA_YAW:
                    if rotacion < 0:
                        rotacion = -VELOCIDAD_MAXIMA_YAW
                    else: 
                        rotacion = VELOCIDAD_MAXIMA_YAW
                        
        self.enviar_comando_velocidad(lateral, avance, vertical, rotacion)
        return distancia_al_objetivo < ERROR_POSICION_M

    def rotar_a_yaw(self, angulo_objetivo_deg):
        error_yaw = normalizar_angulo_deg(angulo_objetivo_deg - self.yaw_dron_deg)
        
        if abs(error_yaw) < ERROR_YAW_DEG:
            self.detener_dron()
            return True
        
        rotacion = rango_velocidad(AUMENTAR_YAW * error_yaw, VELOCIDAD_MAXIMA)
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

        for i, punto in enumerate(self.puntos_trayectoria_actual[:3]):
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

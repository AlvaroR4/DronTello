import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

DISTANCIA_APROXIMACION_M = 0.8
DISTANCIA_SALIDA_M = 0.8
MARGEN_ALTURA_M = 0.0
AUMENTAR_YAW = 4.0
VELOCIDAD_MAXIMA = 40
VELOCIDAD_MINIMA = 20
VELOCIDAD_MAXIMA_YAW = 10.0
ERROR_POS_MIN = 0.12
ERROR_POS_MAX = 1.3 
ERROR_YAW_DEG = 2.0
DISTANCIA_NUEVA_PUERTA = 1.25 

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
        self.puertas_pendientes = []  # {'punto': np.array, 'angulo': float, 'n_muestras': int}
        self.puertas_visitadas = []   # Lista de puntos (np.array) de puertas ya cruzadas
        self.todas_las_puertas = [] #para visualzar
        self.minimo_una_cruzada = False 
        self.indice_objetivo_actual = -1

        self.sub_pose = self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.callback_pose_dron, 10)
        self.sub_puerta = self.create_subscription(Float32MultiArray, '/tello/punto_y_angulo', self.callback_nueva_puerta, 10)
        self.pub_velocidad = self.create_publisher(Float32MultiArray, '/tello/comandos_velocidad', 10)
        self.pub_lista_puertas= self.create_publisher(Float32MultiArray, '/tello/lista_puertas', 10)

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
        data = list(msg.data)
        if len(data) >= 6:
            self.posicion_dron_mundo = np.array([data[0], data[1], data[2]])
            self.yaw_dron_deg = float(data[5])
            self.pose_recibida = True
    
    def promediar_angulos_deg(self, angulo_actual, nuevo_angulo, n_actual, n_nuevo):
        rad_actual = math.radians(angulo_actual)
        rad_nuevo = math.radians(nuevo_angulo)
        
        x_total = (math.cos(rad_actual) * n_actual) + (math.cos(rad_nuevo) * n_nuevo)
        y_total = (math.sin(rad_actual) * n_actual) + (math.sin(rad_nuevo) * n_nuevo)
        
        angulo_promedio_rad = math.atan2(y_total, x_total)
        return math.degrees(angulo_promedio_rad)

    def iniciar_siguiente_mision(self):
        if not self.puertas_pendientes:
            if self.minimo_una_cruzada:
                self.get_logger().info("Lista vacía y misión cumplida. Finalizando.")
                self.estado_mision = 'FIN_MISION' 
            else:
                self.get_logger().info(" Esperando primera detección.")
            return

        # Ordenar la lista de pendientes: de la más cercana a la más lejana respecto al dron
        self.puertas_pendientes.sort(key=lambda p: np.linalg.norm(p['punto'] - self.posicion_dron_mundo))
        indice_seleccionado = -1
        
        for i, puerta in enumerate(self.puertas_pendientes):
            if puerta['n_muestras'] > 5: 
                indice_seleccionado = i
                break
        
        if indice_seleccionado != -1:
            target = self.puertas_pendientes[indice_seleccionado]
            self.indice_objetivo_actual = indice_seleccionado
            
            punto_central = target['punto']
            angulo_obj = target['angulo']
            self.angulo_objetivo_deg = angulo_obj
            
            rad = math.radians(angulo_obj)
            normal = np.array([math.cos(rad), math.sin(rad), 0.0])
            
            p1 = punto_central - DISTANCIA_APROXIMACION_M * normal
            p2 = punto_central - DISTANCIA_APROXIMACION_M/2 * normal
            p3 = punto_central + DISTANCIA_SALIDA_M/2 * normal
            p4 = punto_central + DISTANCIA_SALIDA_M * normal
            
            self.puntos_trayectoria_actual = [p1, p2, p3, p4]
            
            self.estado_mision = 'IR_A_P1'
            self.mision_en_curso = True
            dist = np.linalg.norm(target['punto'] - self.posicion_dron_mundo)
            self.get_logger().info(f"Navegando a puerta a {dist:.2f}m. Ángulo entrada: {angulo_obj:.1f}º")

    def callback_nueva_puerta(self, msg: Float32MultiArray):
        if not self.pose_recibida or len(msg.data) < 4:
            return

        data = list(msg.data)
        punto_recibido = np.array([data[0], data[1], data[2]])
        punto_recibido[2] += MARGEN_ALTURA_M
        angulo_recibido = float(msg.data[3]) 

        for p_vis in self.puertas_visitadas:
            if np.linalg.norm(p_vis - punto_recibido) < DISTANCIA_NUEVA_PUERTA :
                return

        puerta_existente = False
        for puerta in self.puertas_pendientes:
            if np.linalg.norm(puerta['punto'] - punto_recibido) < DISTANCIA_NUEVA_PUERTA :
                n = puerta['n_muestras']
                peso_historico = (n * (n + 1)) / 2
                peso_nuevo = n + 1
                numerador = (puerta['punto'] * peso_historico) + (punto_recibido * peso_nuevo)
                denominador = peso_historico + peso_nuevo
                puerta['punto'] = numerador / denominador
                puerta['angulo'] = self.promediar_angulos_deg(puerta['angulo'], angulo_recibido, peso_historico, peso_nuevo)
                puerta['n_muestras'] += 1
                puerta_existente = True
                break

        if not puerta_existente:
            nueva_puerta = {
                'punto': punto_recibido,
                'angulo': angulo_recibido,
                'n_muestras': 1
            }
            self.puertas_pendientes.append(nueva_puerta)
            self.todas_las_puertas.append(nueva_puerta)
            self.get_logger().info(f"Nueva puerta añadida a cola. Total pendientes: {len(self.puertas_pendientes)}")

    def bucle_de_control(self):
        self.publicar_datos_puertas()

        if not self.pose_recibida:
            self.detener_dron()
            return

        if not self.mision_en_curso and self.estado_mision == 'ESPERANDO_PUERTA':
            self.iniciar_siguiente_mision()
            if not self.mision_en_curso:
                self.detener_dron()
            return

        if self.estado_mision == 'IR_A_P1':
            objetivo = self.puntos_trayectoria_actual[0]
            if self.ir_a_posicion(objetivo, despacio = False, yaw = True):
                self.get_logger().info(f"P1 alcanzado")
                self.estado_mision = 'IR_A_P2'
        
        if self.estado_mision == 'IR_A_P2':
            objetivo = self.puntos_trayectoria_actual[1]
            if self.ir_a_posicion(objetivo, despacio = True, yaw = False):
                self.get_logger().info(f"P2 alcanzado")
                self.estado_mision = 'IR_A_P3'

        if self.estado_mision == 'IR_A_P3':
            objetivo = self.puntos_trayectoria_actual[2]
            if self.ir_a_posicion(objetivo, despacio = True, yaw = False):
                self.get_logger().info(f"P3 alcanzado")
                self.estado_mision = 'IR_A_P4'

        elif self.estado_mision == 'ROTAR_HACIA_PUERTA':
            if self.rotar_a_yaw():
                self.get_logger().info("Alineado. Cruzando hacia punto de salida...")
                self.estado_mision = 'IR_A_P2'

        elif self.estado_mision == 'IR_A_P4':
            objetivo = self.puntos_trayectoria_actual[3]
            
            if self.ir_a_posicion(objetivo, despacio = True, yaw = False):
                self.get_logger().info("--- PUERTA CRUZADA ---")
                
                if 0 <= self.indice_objetivo_actual < len(self.puertas_pendientes):
                    puerta_completada = self.puertas_pendientes.pop(self.indice_objetivo_actual)
                    self.puertas_visitadas.append(puerta_completada['punto'])
                    self.minimo_una_cruzada = True
                    self.get_logger().info(f"Puerta archivada. Total visitadas: {len(self.puertas_visitadas)}")

                if len(self.puertas_pendientes) > 0:
                    self.get_logger().info(f"Quedan {len(self.puertas_pendientes)} puertas pendientes")
                    self.indice_objetivo_actual = -1
                    self.mision_en_curso = False
                    self.estado_mision = 'ESPERANDO_PUERTA'
                    
                else:
                    self.get_logger().info("Lista de pendientes vacía. Misión Finalizada.")
                    self.estado_mision = 'FIN_MISION'

        elif self.estado_mision == 'FIN_MISION':
            self.detener_dron()
            self.enviar_comando_velocidad(2,0,0,0)
            self.mision_en_curso = False

    def mapear_valor(self, valor, in_min, in_max, out_min, out_max):
        if valor <= in_min:
            return out_min
        if valor >= in_max:
            return out_max
        
        pendiente = (out_max - out_min) / (in_max - in_min)
        return out_min + (valor - in_min) * pendiente
    
    def ir_a_posicion(self, punto_objetivo, despacio, yaw):
        error_posicion = punto_objetivo - self.posicion_dron_mundo
        magnitud_error = np.linalg.norm(error_posicion)
        error_min = ERROR_POS_MIN
        if not despacio:
            error_min = error_min *1.5

        if magnitud_error < error_min:
            return True

        avance_norm = error_posicion[0] / magnitud_error
        lateral_norm = error_posicion[1] / magnitud_error
        vertical_norm = error_posicion[2] / magnitud_error
        if despacio:
            velocidad_magnitud = self.mapear_valor(magnitud_error, error_min, ERROR_POS_MAX, VELOCIDAD_MINIMA/2, VELOCIDAD_MAXIMA/2)
        else: 
            velocidad_magnitud = self.mapear_valor(magnitud_error, error_min, ERROR_POS_MAX, VELOCIDAD_MINIMA/2, VELOCIDAD_MAXIMA)

        vx = avance_norm * velocidad_magnitud
        vy = lateral_norm * velocidad_magnitud 
        vz = vertical_norm * velocidad_magnitud
        if vx < abs(4.0):
            vx = 0.0
            vy = vy * 2
        rotacion = 0
        if yaw:
            rotacion = normalizar_angulo_deg(self.angulo_objetivo_deg - self.yaw_dron_deg)
            if abs(rotacion) < ERROR_YAW_DEG:
                rotacion = 0
            else:
                rotacion = rango_velocidad(AUMENTAR_YAW * rotacion, VELOCIDAD_MAXIMA_YAW)

        # Fórmula de rotación de vectores 2D:
        # V_fb = V_x_mundo * cos(yaw) + V_y_mundo * sin(yaw)
        # V_lr = -V_x_mundo * sin(yaw) + V_y_mundo * cos(yaw)

        yaw_rad = math.radians(self.yaw_dron_deg)
        avance_cuerpo = vx * math.cos(yaw_rad) + vy * math.sin(yaw_rad)
        lateral_cuerpo = -vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)
        
        self.enviar_comando_velocidad(lateral_cuerpo, avance_cuerpo, vz, rotacion)
        
        return False
    
    def rotar_a_yaw(self):
        error_yaw = normalizar_angulo_deg(self.angulo_objetivo_deg - self.yaw_dron_deg)
        
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
    
    def publicar_datos_puertas(self):
        msg = Float32MultiArray()
        lista_datos = []
        
        for puerta in self.todas_las_puertas:
            centro = puerta['punto']
            angulo = puerta['angulo']
            
            rad = math.radians(angulo)
            normal = np.array([math.cos(rad), math.sin(rad), 0.0])
            
            p1 = centro - DISTANCIA_APROXIMACION_M * normal
            p2 = centro - (DISTANCIA_APROXIMACION_M / 2.0) * normal
            p3 = centro + (DISTANCIA_SALIDA_M / 2.0) * normal
            p4 = centro + DISTANCIA_SALIDA_M * normal
            
            puntos_interes = [p1, p2, p3, p4, centro]
            for p in puntos_interes:
                lista_datos.extend([float(p[0]), float(p[1]), float(p[2])])
            
            lista_datos.append(float(angulo))

        msg.data = lista_datos
        self.pub_lista_puertas.publish(msg)

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

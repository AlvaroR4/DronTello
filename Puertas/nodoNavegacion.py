import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
"""
DISTANCIA_APROXIMACION_M = 0.6
DISTANCIA_SALIDA_M = 0.6
DISTANCIA_P_CONTROL_M = 0.8 #respecto P_aprox
DISTANCIA_BUSQUEDA_M = 0.5 
DISTANCIA_ANTICIPACION_M = 0.4
PUNTOS_RECTA = 25
PUNTOS_CURVA = 30
MARGEN_ALTURA_M = 0.0
AUMENTAR_YAW = 2.0
VELOCIDAD_AVANCE = 20
VELOCIDAD_MAXIMA = 40
VELOCIDAD_MAXIMA_YAW = 10.0
ERROR_YAW_DEG = 2.0
DISTANCIA_NUEVA_PUERTA = 1.25 
KP = 2.4
KI = 1.0
KD = 1.0
KL_Y = 5.0
KL_Z = 1.0
KML = 0.6 #en funcion de esta se calcula Ka
MAX_INTEGRAL = 15.0
"""
#oreo
""" 
DISTANCIA_APROXIMACION_M = 0.6
DISTANCIA_SALIDA_M = 0.6
DISTANCIA_P_CONTROL_M = 0.8 #respecto P_aprox
DISTANCIA_BUSQUEDA_M = 0.5 
DISTANCIA_ANTICIPACION_M = 0.4
PUNTOS_RECTA = 25
PUNTOS_CURVA = 20
MARGEN_ALTURA_M = 0.0
AUMENTAR_YAW = 2.0
VELOCIDAD_AVANCE = 20
VELOCIDAD_MAXIMA = 40
VELOCIDAD_MAXIMA_YAW = 10.0
ERROR_YAW_DEG = 2.0
DISTANCIA_NUEVA_PUERTA = 1.25 
KP = 2.2
KI = 1.2
KD = 0.8
KL_Y = 5.0
KL_Z = 1.0
KML = 0.5 #en funcion de esta se calcula Ka
MAX_INTEGRAL = 15.0
"""
#webots
DISTANCIA_APROXIMACION_M = 0.8
DISTANCIA_SALIDA_M = 0.8
DISTANCIA_P_CONTROL_M = 1.0 #respecto P_aprox
DISTANCIA_BUSQUEDA_M = 0.8
DISTANCIA_ANTICIPACION_M = 0.4
PUNTOS_RECTA = 80
PUNTOS_CURVA = 80
MARGEN_ALTURA_M = 0.0
AUMENTAR_YAW = 6.0
VELOCIDAD_AVANCE = 25
VELOCIDAD_MAXIMA = 50
VELOCIDAD_MAXIMA_YAW = 20.0
ERROR_YAW_DEG = 2.0
DISTANCIA_NUEVA_PUERTA = 1.25 
KP = 1.4
KI = 0.6
KD = 0.2
KL_Y = 2.5
KL_Z = 2.5
KML = 0.6 #en funcion de esta se calcula Ka
MAX_INTEGRAL = 20.0


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

        self.error_integral = np.zeros(3)
        self.error_previo = np.zeros(3)
        self.punto_inicio_mision = np.zeros(3)
        self.punto_inicio_segmento = np.zeros(3)
        self.tiempo_dt = 1.0 / 20.0
        self.ruta_global = []      
        self.idx_actual = 0

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
    
    def generar_curva_bezier(self, p0, p1, p2, pasos):
        ruta = []
        for t in np.linspace(0, 1, pasos):
            punto = (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
            ruta.append(punto)
        return ruta
    
    def iniciar_siguiente_mision(self):
        if not self.puertas_pendientes:
            if self.minimo_una_cruzada:
                self.get_logger().info("Misión cumplida.")
                self.estado_mision = 'FIN_MISION' 
            return

        self.puertas_pendientes.sort(key=lambda p: np.linalg.norm(p['punto'] - self.posicion_dron_mundo))
        indice_seleccionado = -1
        for i, puerta in enumerate(self.puertas_pendientes):
            if puerta['n_muestras'] > 3: 
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
            
            p_inicio_dron = self.posicion_dron_mundo.copy()
            p_aprox = punto_central - DISTANCIA_APROXIMACION_M * normal
            p_salida = punto_central + DISTANCIA_SALIDA_M * normal
            p_control = p_aprox - DISTANCIA_P_CONTROL_M * normal

            self.ruta_global = []
            
            tramo_curvo = self.generar_curva_bezier(p_inicio_dron, p_control, p_aprox, PUNTOS_CURVA)
            self.ruta_global.extend(tramo_curvo)
            
            vector_recta = p_salida - p_aprox
            for i in range(1, PUNTOS_RECTA + 1): 
                t = i / PUNTOS_RECTA
                p = p_aprox + (vector_recta * t)
                self.ruta_global.append(p)
            
            self.idx_actual = 0 
            self.punto_inicio_segmento = p_inicio_dron.copy() 
            
            self.estado_mision = 'SEGUIR_TRAYECTORIA'
            self.mision_en_curso = True
            
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
            return
        if self.estado_mision == 'SEGUIR_TRAYECTORIA':
            if not self.ruta_global: return

            idx_mas_cercano = self.idx_actual
            dist_minima = float('inf')
            dist_recorrida_acumulada = 0.0 

            for i in range(self.idx_actual, len(self.ruta_global)):
                if i > self.idx_actual:
                    dist_recorrida_acumulada += np.linalg.norm(self.ruta_global[i] - self.ruta_global[i-1])
                
                if dist_recorrida_acumulada > DISTANCIA_BUSQUEDA_M:
                    break

                d_euclidea = np.linalg.norm(self.ruta_global[i] - self.posicion_dron_mundo)
                if d_euclidea < dist_minima:
                    dist_minima = d_euclidea
                    idx_mas_cercano = i

            self.idx_actual = idx_mas_cercano

            idx_objetivo = self.idx_actual
            dist_acumulada = 0.0
            for j in range(self.idx_actual + 1, len(self.ruta_global)):
                dist_acumulada += np.linalg.norm(self.ruta_global[j] - self.ruta_global[j-1])
                idx_objetivo = j
                if dist_acumulada >= DISTANCIA_ANTICIPACION_M:
                    break
            
            es_fase_final = (idx_objetivo == len(self.ruta_global) - 1)

            if es_fase_final:
                ultimo_punto = self.ruta_global[-1]
                dist_final = np.linalg.norm(ultimo_punto - self.posicion_dron_mundo)
                
                if dist_final < 0.4: 
                    self.get_logger().info("--- TRAYECTORIA COMPLETADA ---")
                    self.finalizar_puerta()
                else:
                    punto_A = self.ruta_global[self.idx_actual]
                    self.punto_inicio_segmento = punto_A
                    self.ir_a_posicion(ultimo_punto, yaw=False)

            else:
                punto_A = self.ruta_global[self.idx_actual]     
                punto_B = self.ruta_global[idx_objetivo]        
                
                self.punto_inicio_segmento = punto_A

                self.ir_a_posicion(punto_B, yaw=True)

        elif self.estado_mision == 'FIN_MISION':
            self.detener_dron()
            self.enviar_comando_velocidad(2,0,0,0)
            self.mision_en_curso = False
   
    def finalizar_puerta(self):
        if 0 <= self.indice_objetivo_actual < len(self.puertas_pendientes):
            puerta_completada = self.puertas_pendientes.pop(self.indice_objetivo_actual)
            self.puertas_visitadas.append(puerta_completada['punto'])
            self.minimo_una_cruzada = True
        
        self.mision_en_curso = False
        self.resetear_pid()
        if len(self.puertas_pendientes) > 0:
            self.indice_objetivo_actual = -1
            self.estado_mision = 'ESPERANDO_PUERTA'
        else:
            self.estado_mision = 'FIN_MISION'

    def mapear_valor(self, valor, in_min, in_max, out_min, out_max):
        if valor <= in_min:
            return out_min
        if valor >= in_max:
            return out_max
        
        pendiente = (out_max - out_min) / (in_max - in_min)
        return out_min + (valor - in_min) * pendiente
    
    def ir_a_posicion(self, punto_objetivo, yaw):
        vector_total, vector_correccion = self.calcular_error_trayectoria(punto_objetivo)
        
        P_vector = vector_total * KP

        self.error_integral += vector_correccion * self.tiempo_dt
        
        self.error_integral = np.clip(self.error_integral, -MAX_INTEGRAL, MAX_INTEGRAL)
        
        I_vector = KI * self.error_integral
        
        derivada = (vector_total - self.error_previo) / self.tiempo_dt
        D_vector = KD * derivada
        
        self.error_previo = vector_total
        
        velocidad_mundo = P_vector + I_vector + D_vector

        norm_vel = np.linalg.norm(velocidad_mundo)
        if norm_vel > VELOCIDAD_MAXIMA:
            velocidad_mundo = (velocidad_mundo / norm_vel) * VELOCIDAD_MAXIMA

        vx_mundo = velocidad_mundo[0]
        vy_mundo = velocidad_mundo[1]
        vz_mundo = velocidad_mundo[2]
        
        rotacion = 0
        if yaw:
            v_carril = punto_objetivo - self.punto_inicio_segmento
            
            if np.linalg.norm(v_carril) < 0.01:
                 dx, dy = velocidad_mundo[0], velocidad_mundo[1]
            else:
                 dx, dy = v_carril[0], v_carril[1]

            angulo_carril_rad = math.atan2(dy, dx)
            angulo_carril_deg = math.degrees(angulo_carril_rad)
            
            diff_yaw = normalizar_angulo_deg(angulo_carril_deg - self.yaw_dron_deg)

            if abs(diff_yaw) < ERROR_YAW_DEG:
                rotacion = 0
            else:
                rotacion = rango_velocidad(AUMENTAR_YAW * diff_yaw, VELOCIDAD_MAXIMA_YAW)

        yaw_rad = math.radians(self.yaw_dron_deg)
        avance_cuerpo = vx_mundo * math.cos(yaw_rad) + vy_mundo * math.sin(yaw_rad)
        lateral_cuerpo = -vx_mundo * math.sin(yaw_rad) + vy_mundo * math.cos(yaw_rad)
        
        self.enviar_comando_velocidad(lateral_cuerpo, avance_cuerpo, vz_mundo, rotacion)
        
        return False
    
    def calcular_error_trayectoria(self, punto_objetivo):
        p_inicio = self.punto_inicio_segmento
        
        if np.linalg.norm(p_inicio) == 0 and np.linalg.norm(self.posicion_dron_mundo) == 0:
             diff = punto_objetivo - self.posicion_dron_mundo
             return diff, diff 

        v_ruta = punto_objetivo - p_inicio
        len_ruta = np.linalg.norm(v_ruta)
        
        if len_ruta < 0.01:
            diff = punto_objetivo - self.posicion_dron_mundo
            return diff, diff

        u_ruta = v_ruta / len_ruta
        v_dron = self.posicion_dron_mundo - p_inicio
        
        proyeccion = np.dot(v_dron, u_ruta)
        proyeccion_clamped = np.clip(proyeccion, 0, len_ruta)
        p_ideal = p_inicio + (proyeccion_clamped * u_ruta)
        
        e_lateral = p_ideal - self.posicion_dron_mundo 
        
        e_lat_z = e_lateral[2]
        e_lat_xy = np.array([e_lateral[0], e_lateral[1], 0.0])

        v_correccion_xy = e_lat_xy * KL_Y
        v_correccion_z = np.array([0.0, 0.0, e_lat_z]) * KL_Z

        vector_solo_correccion = v_correccion_xy + v_correccion_z

        dist_lat = np.linalg.norm(vector_solo_correccion)
        factor_penalizacion = dist_lat / KML
        k_avance = 1.0 - factor_penalizacion
        k_avance = np.clip(k_avance, 0.1, 1.0) 
        
        e_avance = u_ruta * VELOCIDAD_AVANCE
        
        vector_total = vector_solo_correccion + (e_avance * k_avance)
        
        return vector_total, vector_solo_correccion
    
    def enviar_comando_velocidad(self, lr, fb, ud, yv):
        msg = Float32MultiArray(data=[float(lr), float(fb), float(ud), float(yv)])
        #msg = Float32MultiArray(data=[0,0,0,0])
        self.pub_velocidad.publish(msg)

    def resetear_pid(self):
        self.error_integral = np.zeros(3)
        self.error_previo = np.zeros(3)

    def detener_dron(self):
        self.enviar_comando_velocidad(0, 0, 0, 0)
    
    def publicar_datos_puertas(self):
        msg = Float32MultiArray()
        lista_datos = []

        if self.ruta_global:
            lista_datos.append(float(len(self.ruta_global))) 
            for p in self.ruta_global:
                lista_datos.extend([float(p[0]), float(p[1]), float(p[2])])
            
            if 0 <= self.indice_objetivo_actual < len(self.puertas_pendientes):
                puerta = self.puertas_pendientes[self.indice_objetivo_actual]
                c = puerta['punto']
                lista_datos.extend([float(c[0]), float(c[1]), float(c[2]), float(puerta['angulo'])])
            else:
                lista_datos.extend([0.0, 0.0, 0.0, 0.0])

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

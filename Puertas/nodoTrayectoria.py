import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

DIST_ANTES_M = 0.50
DIST_DESPUES_M = 0.50
ALTURA_SOBRE_PUNTO_M = 0.50
TOLERANCIA_NUEVA_PUERTA_M = 1.0

K_LINEAL = 0.8
K_YAW = 1.0
VEL_MAX = 20
TOL_POS_M = 0.10
TOL_YAW_DEG = 5.0
FRECUENCIA_HZ = 20.0

def saturar(valor, maximo):
    return np.clip(valor, -maximo, maximo)

def wrap_ang_deg(ang):
    return (ang + 180.0) % 360.0 - 180.0

class NodoTrayectoria(Node):
    def __init__(self):
        super().__init__('nodo_trayectoria')

        self.pose_recibida = False
        self.pos_dron_mundo = np.zeros(3)
        self.yaw_deg = 0.0
        self.estado = 'ESPERANDO_PUERTA'
        self.puertas_navegadas = []
        self.mision_activa = False
        self.puntos_mision_actual = [] # Para guardar [P', P, P'']
        self.angulo_puerta_deg = 0.0

        # --- Suscripciones y Publicaciones ---
        self.sub_pose = self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.callback_pose, 10)
        self.sub_datos = self.create_subscription(Float32MultiArray, '/tello/punto_y_angulo', self.callback_datos, 10)
        self.pub_vel = self.create_publisher(Float32MultiArray, '/tello/comandos_velocidad', 10)
        qos_profile_marcadores = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 
        )
        self.pub_marcadores = self.create_publisher(MarkerArray, '/trayectoria_visual', qos_profile_marcadores)

        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.bucle_control)
        self.get_logger().info("Nodo trayectoria iniciado. Esperando la primera puerta...")

    def callback_pose(self, msg: Float32MultiArray):
        if len(msg.data) >= 6:
            self.pos_dron_mundo = np.array(msg.data[0:3])
            self.yaw_deg = msg.data[5]
            self.pose_recibida = True

    def callback_datos(self, msg: Float32MultiArray):
        if len(msg.data) < 4 or self.mision_activa: return

        punto_recibido = np.array(msg.data[0:3])
        es_puerta_nueva = all(np.linalg.norm(punto_recibido - p) > TOLERANCIA_NUEVA_PUERTA_M for p in self.puertas_navegadas)

        if not es_puerta_nueva: return

        self.get_logger().info(f"¡Nueva puerta detectada en {punto_recibido}!")
        self.puertas_navegadas.append(punto_recibido)
        
        punto_p = punto_recibido
        self.angulo_puerta_deg = float(msg.data[3])
        angulo_rad = math.radians(self.angulo_puerta_deg)

        punto_p[2] -= ALTURA_SOBRE_PUNTO_M
        normal_plano = np.array([math.cos(angulo_rad), math.sin(angulo_rad), 0.0])
        punto_p_prima = punto_p - DIST_ANTES_M * normal_plano
        punto_p_doble_prima = punto_p + DIST_DESPUES_M * normal_plano

        self.puntos_mision_actual = [punto_p_prima, punto_p, punto_p_doble_prima]
        
        self.publicar_marcadores_debug()
        
        self.estado = 'IR_A_P_PRIMA'
        self.mision_activa = True
        self.get_logger().info(f"Iniciando trayectoria hacia P'={punto_p_prima}")

    def publicar_marcadores_debug(self, borrar=False):
        marker_array = MarkerArray()
        colores = [
            [1.0, 1.0, 0.0, 0.8],  # Amarillo para P'
            [1.0, 0.0, 0.0, 0.8],  # Rojo para P
            [0.0, 1.0, 0.0, 0.8]   # Verde para P''
        ]

        for i, punto in enumerate(self.puntos_mision_actual):
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

            marker.lifetime = rclpy.duration.Duration(seconds=30).to_msg()
            marker_array.markers.append(marker)
        
        self.pub_marcadores.publish(marker_array)

    def publicar_velocidad(self, lr, fb, ud, yv):
        msg = Float32MultiArray(data=[float(lr), float(fb), float(ud), float(yv)])
        self.pub_vel.publish(msg)

    def detener_dron(self):
        self.publicar_velocidad(0, 0, 0, 0)

    def controlar_posicion(self, objetivo, yaw_objetivo_deg=None):
        error_pos = objetivo - self.pos_dron_mundo
        distancia = np.linalg.norm(error_pos)
        
        vel_mundo = K_LINEAL * error_pos
        yaw_rad = math.radians(self.yaw_deg)
        c, s = math.cos(-yaw_rad), math.sin(-yaw_rad)
        R_mundo_a_cuerpo = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        vel_cuerpo = R_mundo_a_cuerpo @ vel_mundo

        fb_cmd = saturar(vel_cuerpo[0] * 100, VEL_MAX)
        lr_cmd = saturar(-vel_cuerpo[1] * 100, VEL_MAX)
        ud_cmd = saturar(-vel_cuerpo[2] * 100, VEL_MAX)
        
        yv_cmd = 0
        if yaw_objetivo_deg is not None:
            error_yaw = wrap_ang_deg(yaw_objetivo_deg - self.yaw_deg)
            yv_cmd = saturar(K_YAW * error_yaw, VEL_MAX)

        self.publicar_velocidad(lr_cmd, fb_cmd, ud_cmd, yv_cmd)
        return distancia < TOL_POS_M

    def controlar_rotacion(self, yaw_objetivo_deg):
        error_yaw = wrap_ang_deg(yaw_objetivo_deg - self.yaw_deg)
        if abs(error_yaw) < TOL_YAW_DEG:
            self.detener_dron()
            return True
        
        yv_cmd = saturar(K_YAW * error_yaw, VEL_MAX)
        self.publicar_velocidad(0, 0, 0, yv_cmd)
        return False

    def bucle_control(self):
        if not self.pose_recibida or not self.mision_activa:
            return

        estado_actual = self.estado
        
        if estado_actual == 'IR_A_P_PRIMA':
            objetivo_actual = self.puntos_mision_actual[0]
            if self.controlar_posicion(objetivo_actual):
                self.get_logger().info("Alcanzado P'. Rotando para alinear con la puerta...")
                self.estado = 'ROTAR_PARA_CRUZAR'

        elif estado_actual == 'ROTAR_PARA_CRUZAR':
            if self.controlar_rotacion(self.angulo_puerta_deg):
                self.get_logger().info("Rotación completada. Cruzando la puerta...")
                self.estado = 'IR_A_P_DOBLE_PRIMA'

        elif estado_actual == 'IR_A_P_DOBLE_PRIMA':
            objetivo_actual = self.puntos_mision_actual[2]
            if self.controlar_posicion(objetivo_actual, yaw_objetivo_deg=self.angulo_puerta_deg):
                self.get_logger().info("¡Puerta cruzada! Misión completada.")
                self.estado = 'ESPERANDO_PUERTA'
                self.mision_activa = False
                self.detener_dron()
                self.publicar_marcadores_debug(borrar=True)

# --- AÑADIDO: Bloque principal para ejecutar el nodo ---
def main(args=None):
    rclpy.init(args=args)
    nodo_trayectoria = NodoTrayectoria()
    try:
        rclpy.spin(nodo_trayectoria)
    except KeyboardInterrupt:
        nodo_trayectoria.get_logger().info("Nodo detenido por el usuario (Ctrl+C)")
    finally:
        # Asegurarse de que el dron se detiene al cerrar
        nodo_trayectoria.detener_dron()
        nodo_trayectoria.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
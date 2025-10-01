import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

# ==============================
# Constantes de configuración
# ==============================
DIST_ANTES_M = 0.20      # Distancia a P' (antes del plano)
DIST_DESPUES_M = 0.20    # Distancia a P'' (después del plano)
K_LINEAL = 1.0           # Ganancia proporcional posición
K_YAW = 1.0              # Ganancia proporcional yaw
VEL_RC_MAX = 10          # Límite de comandos [-100,100]
TOL_POS_M = 0.07         # Tolerancia de posición
TOL_YAW_DEG = 3.0        # Tolerancia de yaw
FRECUENCIA_HZ = 20.0     # Frecuencia de control (Hz)

def saturar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo)

def wrap_ang_deg(ang):
    """ Normaliza ángulos a [-180, 180] """
    return (ang + 180.0) % 360.0 - 180.0

def rotz_matrix(theta_rad):
    """ Matriz de rotación en Z """
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)

class NodoTrayectoria(Node):
    def __init__(self):
        super().__init__('nodo_trayectoria')

        # ==============================
        # Variables internas
        # ==============================
        self.pose_recibida = False
        self.pos_dron_mundo = np.zeros(3)
        self.yaw_deg = 0.0
        self.estado = 'ESPERANDO_DATOS'

        # ==============================
        # Suscripciones y publicaciones
        # ==============================
        self.sub_pose = self.create_subscription(
            PoseStamped, '/robot_pose_slam', self.callback_pose, 10)

        self.sub_datos = self.create_subscription(
            Float32MultiArray, '/punto_y_angulo', self.callback_datos, 10)

        self.pub_vel = self.create_publisher(Float32MultiArray, '/tello/comandos_velocidad', 10)

        # Bucle de control
        self.timer = self.create_timer(1.0/FRECUENCIA_HZ, self.control_loop)

        self.get_logger().info("Nodo trayectoria iniciado. Esperando datos en /punto_y_angulo...")

    # =========================================================
    # Paso 4. Recibir la pose del dron (posición y orientación)
    # =========================================================
    def callback_pose(self, msg: PoseStamped):
        # Posición del dron
        self.pos_dron_mundo = np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        msg.pose.position.z], dtype=float)

        # Cuaternión → yaw (solo nos importa yaw en grados)
        qx, qy, qz, qw = msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)

        self.yaw_deg = math.degrees(yaw_rad)
        self.pose_recibida = True

    # =========================================================
    # Paso 1. Recibir P y ángulo desde el topic /punto_y_angulo
    # =========================================================
    def callback_datos(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            self.get_logger().error("Mensaje inválido, se esperaban 4 valores [x,y,z,angulo]")
            return

        self.punto_p = np.array(msg.data[0:3], dtype=float)
        self.angulo_grados = float(msg.data[3])
        self.angulo_rad = math.radians(self.angulo_grados)

        # Paso 2. Calcular vector normal del plano
        self.normal_plano = np.array([math.cos(self.angulo_rad), 0.0, math.sin(self.angulo_rad)], dtype=float)
        self.normal_plano /= (np.linalg.norm(self.normal_plano) + 1e-9)

        # Paso 3. Calcular P' y P''
        self.punto_p_prima = self.punto_p - DIST_ANTES_M * self.normal_plano
        self.punto_p_doble_prima = self.punto_p + DIST_DESPUES_M * self.normal_plano

        # Arrancar misión
        self.estado = 'IR_P_PRIMA'
        self.get_logger().info(f"Recibido P={self.punto_p}, ángulo={self.angulo_grados}°")

    # =========================================================
    # Funciones auxiliares para control
    # =========================================================
    def publicar_vel(self, lr, fb, ud, yv):
        msg = Float32MultiArray()
        msg.data = [float(lr), float(fb), float(ud), float(yv)]
        self.pub_vel.publish(msg)

    def detener(self):
        self.publicar_vel(0,0,0,0)

    def vector_error(self, objetivo):
        return (objetivo - self.pos_dron_mundo)

    def mundo_a_cuerpo_rc(self, v_mundo):
        """ Convierte vector en marco mundo → marco dron (rc control) """
        yaw_rad = math.radians(self.yaw_deg)
        v_cuerpo_enu = rotz_matrix(-yaw_rad) @ v_mundo
        fb = v_cuerpo_enu[0]
        lr = -v_cuerpo_enu[1]  # derecha positiva
        ud = v_cuerpo_enu[2]
        return lr, fb, ud

    def controlar_posicion(self, objetivo, forzar_yaw=None):
        error = self.vector_error(objetivo)
        dist = np.linalg.norm(error)

        # Velocidad proporcional
        v_mundo = K_LINEAL * error
        lr_f, fb_f, ud_f = self.mundo_a_cuerpo_rc(v_mundo)

        # Escalar y limitar
        lr_cmd = int(saturar(lr_f*100, -VEL_RC_MAX, VEL_RC_MAX))
        fb_cmd = int(saturar(fb_f*100, -VEL_RC_MAX, VEL_RC_MAX))
        ud_cmd = int(saturar(ud_f*100, -VEL_RC_MAX, VEL_RC_MAX))

        # Yaw (si aplica)
        if forzar_yaw is None:
            yv_cmd = 0
        else:
            err_yaw = wrap_ang_deg(forzar_yaw - self.yaw_deg)
            yv_cmd = int(saturar(K_YAW*err_yaw, -VEL_RC_MAX, VEL_RC_MAX))

        self.publicar_vel(lr_cmd, fb_cmd, ud_cmd, yv_cmd)
        return (dist <= TOL_POS_M)

    def controlar_rotacion_unica(self, yaw_obj_deg):
        err_yaw = wrap_ang_deg(yaw_obj_deg - self.yaw_deg)
        yv_cmd = int(saturar(K_YAW*err_yaw, -VEL_RC_MAX, VEL_RC_MAX))
        self.publicar_vel(0,0,0,yv_cmd)
        return abs(err_yaw) <= TOL_YAW_DEG

    # =========================================================
    # Paso 5. Máquina de estados: ir a P' → rotar → P → P''
    # =========================================================
    def control_loop(self):
        if not self.pose_recibida or self.estado == 'ESPERANDO_DATOS':
            self.detener()
            return

        if self.estado == 'IR_P_PRIMA':
            if self.controlar_posicion(self.punto_p_prima):
                self.get_logger().info("Alcanzado P'. Rotando...")
                self.estado = 'ROTAR_ANGULO'

        elif self.estado == 'ROTAR_ANGULO':
            if self.controlar_rotacion_unica(self.angulo_grados):
                self.get_logger().info("Yaw alineado. Yendo a P...")
                self.estado = 'IR_P'

        elif self.estado == 'IR_P':
            if self.controlar_posicion(self.punto_p, forzar_yaw=self.angulo_grados):
                self.get_logger().info("Alcanzado P. Yendo a P''...")
                self.estado = 'IR_P_DOBLE_PRIMA'

        elif self.estado == 'IR_P_DOBLE_PRIMA':
            if self.controlar_posicion(self.punto_p_doble_prima):
                self.get_logger().info("Alcanzado P''. Misión completa.")
                self.estado = 'HECHO'
                self.detener()

        elif self.estado == 'HECHO':
            self.detener()

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoTrayectoria()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Nodo detenido por Ctrl+C")
    finally:
        nodo.detener()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

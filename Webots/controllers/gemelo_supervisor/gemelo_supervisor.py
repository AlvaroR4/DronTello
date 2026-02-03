import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from controller import Supervisor

class PuenteGemelo(Node):
    def __init__(self, supervisor):
        super().__init__('nodo_gemelo_webots')
        self.supervisor = supervisor
        
        self.nodo_dron = self.supervisor.getFromDef('Mavic_2_PRO')
        self.nodo_puerta1 = self.supervisor.getFromDef('PuertaA1')
        self.nodo_puerta2 = self.supervisor.getFromDef('PuertaA2')
        
        pose_inicial_p = [-20.0, -20.0, -20.0]
        if self.nodo_puerta1: self.nodo_puerta1.getField("translation").setSFVec3f(pose_inicial_p)
        if self.nodo_puerta2: self.nodo_puerta2.getField("translation").setSFVec3f(pose_inicial_p)

        self.pose_dron = None
        self.rot_dron = None
        self.pose_puerta1 = None
        self.rot_puerta1 = None
        self.pose_puerta2 = None
        self.rot_puerta2 = None

        qos_perfil = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1 
        )
        
        self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.callback_dron, qos_perfil)
        self.create_subscription(Float32MultiArray, '/tello/lista_puertas', self.callback_puertas, qos_perfil)

    def callback_dron(self, msg):
        datos = msg.data
        if len(datos) < 3: return
        
        x, y, z = datos[0], datos[1], datos[2]
        
        yaw_grados = 0.0
        if len(datos) == 4: yaw_grados = datos[3]
        elif len(datos) >= 6: yaw_grados = datos[5]

        self.pose_dron = [float(x), float(y), float(z)]
        self.rot_dron = [0.0, 0.0, 1.0, math.radians(yaw_grados - 0,785398)]

    def callback_puertas(self, msg):
        datos = msg.data
        
        if len(datos) >= 16:
            p1_x, p1_y, p1_z = datos[12], datos[13], datos[14]
            p1_yaw = datos[15]
            
            z_corregida = float(p1_z) - 0.58
            
            self.pose_puerta1 = [float(p1_x), float(p1_y), z_corregida]
            self.rot_puerta1 = [0.0, 0.0, 1.0, math.radians(p1_yaw)]
        
        if len(datos) >= 16 * 2:
            offset = 16
            p2_x, p2_y, p2_z = datos[offset + 12], datos[offset + 13], datos[offset + 14]
            p2_yaw = datos[offset + 15]
            
            z_corregida_2 = float(p2_z) - 1.0

            self.pose_puerta2 = [float(p2_x), float(p2_y), z_corregida_2]
            self.rot_puerta2 = [0.0, 0.0, 1.0, math.radians(p2_yaw)]

    def actualizar_escena(self):
        if self.nodo_dron and self.pose_dron:
            self.nodo_dron.getField("translation").setSFVec3f(self.pose_dron)
            self.nodo_dron.getField("rotation").setSFRotation(self.rot_dron)
            
        if self.nodo_puerta1 and self.pose_puerta1:
            self.nodo_puerta1.getField("translation").setSFVec3f(self.pose_puerta1)
            self.nodo_puerta1.getField("rotation").setSFRotation(self.rot_puerta1)
            
        if self.nodo_puerta2 and self.pose_puerta2:
            self.nodo_puerta2.getField("translation").setSFVec3f(self.pose_puerta2)
            self.nodo_puerta2.getField("rotation").setSFRotation(self.rot_puerta2)

def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep())
    rclpy.init(args=None)
    nodo = PuenteGemelo(supervisor)

    while supervisor.step(timestep) != -1:
        rclpy.spin_once(nodo, timeout_sec=0)
        nodo.actualizar_escena()

    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
from controller import Supervisor
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PointStamped

ESFERA_PROTO = """
Solid {
  translation %f %f %f
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 1 0
        metalness 0
        roughness 0.5
      }
      geometry Sphere { radius 0.05 }
    }
  ]
}
"""

class WebotsVisualizador(Node):
    def __init__(self, supervisor):
        super().__init__('webots_visualizador')
        self.supervisor = supervisor
        
        self.dron_node = self.supervisor.getFromDef("DRON_TELLO")
        if self.dron_node is None:
            self.get_logger().error("error")
        else:
            self.trans_field = self.dron_node.getField("translation")
            self.rot_field = self.dron_node.getField("rotation")

        self.root_children = self.supervisor.getRoot().getField("children")

        self.create_subscription(Float32MultiArray, '/tello/pose_corregida', self.callback_pose, 10)
        self.create_subscription(Float32MultiArray, '/tello/puertas_detectadas', self.callback_puerta, 10)
        self.create_subscription(Float32MultiArray, '/tello/punto_y_angulo', self.callback_punto_trayectoria, 10)

    def callback_pose(self, msg):
        if self.dron_node:
            x, y, z = msg.data[0], msg.data[1], msg.data[2]
            yaw = msg.data[5] 
            
            self.trans_field.setSFVec3f([x, y, z])
            
            self.rot_field.setSFRotation([0, 0, 1, yaw * 3.14159 / 180.0])

    def callback_punto_trayectoria(self, msg):
        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        
        nueva_esfera = ESFERA_PROTO % (x, y, z)
        
        self.root_children.importMFNodeFromString(-1, nueva_esfera)
        self.get_logger().info(f"Esfera dibujada en Webots: {x:.2f}, {y:.2f}, {z:.2f}")

    def callback_puerta(self, msg):
        id_puerta = int(msg.data[4]) 
        nodo_puerta = self.supervisor.getFromDef(f"PUERTA_{id_puerta}")
        
        if nodo_puerta:
            mat = nodo_puerta.getField("children").getMFNode(0).getField("appearance").getSFNode()
            mat.getField("transparency").setSFFloat(0.0)
            nodo_puerta.getField("translation").setSFVec3f([msg.data[0], msg.data[1], msg.data[2]])

def main(args=None):
    rclpy.init(args=args)
    supervisor = Supervisor()
    nodo_vis = WebotsVisualizador(supervisor)
    
    timestep = int(supervisor.getBasicTimeStep())
    
    while supervisor.step(timestep) != -1:
        rclpy.spin_once(nodo_vis, timeout_sec=0)

    nodo_vis.destroy_node()
    rclpy.shutdown()
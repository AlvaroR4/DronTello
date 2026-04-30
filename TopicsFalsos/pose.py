import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import math

FRECUENCIA_ACTUALIZACION = 20.0
FACTOR_ESCALA_VELOCIDAD = 0.01
FACTOR_ESCALA_YAW = 0.5

class SimuladorPoseDron(Node):
    def __init__(self):
        super().__init__('simulador_pose_dron')
        self.get_logger().info('Iniciando simulador FLU (Front-Left-Up).')

        self.pose_actual = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.velocidades_cmd = np.array([0.0, 0.0, 0.0, 0.0])

        self.sub_comandos = self.create_subscription(
            Float32MultiArray,
            '/tello/comandos_velocidad',
            self.callback_comandos_velocidad,
            10)
        
        self.pub_pose = self.create_publisher(
            Float32MultiArray,
            '/tello/pose_corregida',
            10)

        periodo_actualizacion = 1.0 / FRECUENCIA_ACTUALIZACION
        self.timer = self.create_timer(periodo_actualizacion, self.actualizar_y_publicar_pose)

    def callback_comandos_velocidad(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        
        lr, fb, ud = msg.data[0], msg.data[1], msg.data[2]

        if lr == 2.0 and fb == 0.0 and ud == 0.0:
            self.get_logger().info("Comando: land")
            self.velocidades_cmd = np.zeros(4)
            return
        
        if lr == 2.0 and fb == 2.0 and ud == 2.0:
            self.get_logger().info("Comando: takeoff")
            self.velocidades_cmd = np.zeros(4)
            
            if self.pose_actual[2] < 0.1:
                self.pose_actual[2] = 1.4
            return

        self.velocidades_cmd = np.array(msg.data)

    def actualizar_y_publicar_pose(self):
        dt = 1.0 / FRECUENCIA_ACTUALIZACION

        vel_avance = self.velocidades_cmd[1] * FACTOR_ESCALA_VELOCIDAD
        vel_lateral = self.velocidades_cmd[0] * FACTOR_ESCALA_VELOCIDAD
        vel_vertical = self.velocidades_cmd[2] * FACTOR_ESCALA_VELOCIDAD
        vel_yaw = self.velocidades_cmd[3] * FACTOR_ESCALA_YAW

        nuevo_yaw_grados = self.pose_actual[5] - (vel_yaw * dt)
        nuevo_yaw_grados = (nuevo_yaw_grados + 180.0) % 360.0 - 180.0
        self.pose_actual[5] = nuevo_yaw_grados
        
        
        delta_x_mundo = vel_avance * dt
        
        delta_y_mundo = -vel_lateral * dt 
        
        delta_z_mundo = vel_vertical * dt

        self.pose_actual[0] += delta_x_mundo
        self.pose_actual[1] += delta_y_mundo
        self.pose_actual[2] += delta_z_mundo
        
        if self.pose_actual[2] < 0.0:
            self.pose_actual[2] = 0.0

        msg = Float32MultiArray()
        msg.data = [float(v) for v in self.pose_actual]
        self.pub_pose.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo_simulador = SimuladorPoseDron()

    try:
        rclpy.spin(nodo_simulador)
    except KeyboardInterrupt:
        print("Cerrando nodo simulador.")
    finally:
        nodo_simulador.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
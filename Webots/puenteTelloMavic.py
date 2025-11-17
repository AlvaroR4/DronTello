#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import tf_transformations

from std_msgs.msg import Float32MultiArray

from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu  
from sensor_msgs.msg import NavSatFix 

class TelloMavicPuente(Node):
    def __init__(self):
        super().__init__('tello_a_mavic_puente')

        self.sub_cam_mavic = self.create_subscription(
            Image,
            '/Mavic_2_PRO/camera/image_color',  
            self.callback_camara,
            10
        )
        self.pub_cam_tello = self.create_publisher(
            Image,
            '/tello/imagen',  
            10
        )

        self.sub_vel_tello = self.create_subscription(
            Float32MultiArray,
            '/tello/comandos_velocidad',        
            self.callback_velocidad,
            10
        )
        self.pub_vel_mavic = self.create_publisher(
            Twist,
            '/cmd_vel',                        
            10
        )

        self.sub_imu_mavic = self.create_subscription(
            Imu,
            '/imu',
            self.callback_imu,
            10
        )
        self.sub_gps_mavic = self.create_subscription(
            NavSatFix,
            '/Mavic_2_PRO/gps',
            self.callback_gps,
            10
        )

        self.pub_pose_tello = self.create_publisher(
            Float32MultiArray,
            '/tello/pose_corregida',            
            10
        )

        self.posicion_actual = [0.0, 0.0, 0.0]  # [x, y, z]
        self.orientacion_actual = [0.0, 0.0, 0.0] # [roll, pitch, yaw]
        self.gps_origen = None
        self.R_TIERRA = 6371000  # Radio de la Tierra en metros

        self.timer_pose = self.create_timer(0.05, self.publicar_pose_corregida)

        self.get_logger().info('Nodo Puente Tello <-> Mavic iniciado.')


    def callback_camara(self, msg_in):
        self.pub_cam_tello.publish(msg_in)

    def callback_velocidad(self, msg_in):
        lr, fb, ud, yv = msg_in.data
        
        msg_out = Twist()
        msg_out.linear.x = fb  # Tello 'fb' (forward/backward) -> Twist 'linear.x'
        msg_out.linear.y = -lr # Tello 'lr' (left/right) -> Twist 'linear.y' (ROS es Y-izq)
        msg_out.linear.z = ud  # Tello 'ud' (up/down) -> Twist 'linear.z'
        msg_out.angular.z = -yv # Tello 'yv' (yaw) -> Twist 'angular.z' (ROS es Z-CCW)
        
        self.pub_vel_mavic.publish(msg_out)

    def callback_imu(self, msg_in):
        q = msg_in.orientation
        (r, p, y) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        self.orientacion_actual = [r, p, y] 

    def callback_gps(self, msg_in):
        lat = msg_in.latitude
        lon = msg_in.longitude
        alt = msg_in.altitude 

        if self.gps_origen is None:
            self.gps_origen = {'lat': lat, 'lon': lon, 'alt': alt}
            return

        d_lat = np.radians(lat - self.gps_origen['lat'])
        d_lon = np.radians(lon - self.gps_origen['lon'])
        lat_rad = np.radians(self.gps_origen['lat'])
        
        x = d_lon * self.R_TIERRA * np.cos(lat_rad)
        y = d_lat * self.R_TIERRA
        z = alt - self.gps_origen['alt'] 

        self.posicion_actual = [x, y, z]

    def publicar_pose_corregida(self):
        data_out = [
            self.posicion_actual[0],
            self.posicion_actual[1],
            self.posicion_actual[2],
            self.orientacion_actual[0],
            self.orientacion_actual[1],
            self.orientacion_actual[2]
        ]
        
        msg_out = Float32MultiArray(data=data_out)
        self.pub_pose_tello.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    nodo_puente = TelloMavicPuente()
    rclpy.spin(nodo_puente)
    nodo_puente.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
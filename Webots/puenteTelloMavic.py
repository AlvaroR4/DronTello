#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import tf_transformations

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Imu  
from sensor_msgs.msg import NavSatFix 

# Factor para convertir las unidades de "Tello" (0-100 o similar) a m/s reales del Mavic
# Si tu nodo envía 25, esto lo convierte a 0.5 m/s, que es seguro.
FACTOR_ESCALA_VELOCIDAD = 0.05 
FACTOR_ESCALA_YAW = 0.1

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

        self.get_logger().info('Nodo Puente Tello <-> Mavic iniciado y corregido.')


    def callback_camara(self, msg_in):
        self.pub_cam_tello.publish(msg_in)

    def callback_velocidad(self, msg_in):
        # Datos recibidos del nodo de navegación (Sistema Mundo + Unidades Tello)
        # lr = Y mundo, fb = X mundo, ud = Z mundo (invertido), yv = Yaw rate
        lr, fb, ud, yv = msg_in.data
        
        # 1. Corrección de Signo Z (Altura)
        # Tu nodo envía negativo para subir. ROS necesita positivo.
        # Invertimos el signo aquí.
        cmd_z_mundo = -ud 

        # 2. Corrección de Escala
        # Convertimos las unidades grandes (ej. 25) a m/s razonables
        vx_mundo = fb * FACTOR_ESCALA_VELOCIDAD
        vy_mundo = lr * FACTOR_ESCALA_VELOCIDAD
        vz_mundo = cmd_z_mundo * FACTOR_ESCALA_VELOCIDAD
        wz_cuerpo = yv * FACTOR_ESCALA_YAW

        # 3. Transformación de MUNDO a CUERPO
        # El dron necesita comandos relativos a su frente, no al Norte geográfico.
        yaw_actual = self.orientacion_actual[2] # En radianes
        
        # Fórmula de rotación 2D para pasar de Mundo a Cuerpo
        # v_x_cuerpo = v_x_mundo * cos(yaw) + v_y_mundo * sin(yaw)
        # v_y_cuerpo = -v_x_mundo * sin(yaw) + v_y_mundo * cos(yaw)
        
        c = math.cos(yaw_actual)
        s = math.sin(yaw_actual)

        vx_cuerpo = vx_mundo * c + vy_mundo * s
        vy_cuerpo = -vx_mundo * s + vy_mundo * c

        # Corrección del signo de Yaw (si gira al revés, quita o pon el menos)
        # Tu nodo envía positivo para girar a un lado, el puente tenía un menos.
        # Generalmente ROS usa positivo para izquierda (CCW).
        # Lo ponemos directo (positivo) porque tu lógica parece estándar.
        wz_final = wz_cuerpo 

        # Crear mensaje Twist para ROS
        msg_out = Twist()
        msg_out.linear.x = float(vx_cuerpo)
        msg_out.linear.y = float(vy_cuerpo)
        msg_out.linear.z = float(vz_mundo) # Z no necesita rotación 2D
        msg_out.angular.z = float(wz_final)
        
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
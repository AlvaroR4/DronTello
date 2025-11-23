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

# Factor de escala (ajusta si va muy lento o muy rápido)
FACTOR_ESCALA_VELOCIDAD = 0.05 
FACTOR_ESCALA_YAW = 0.5

class TelloMavicPuente(Node):
    def __init__(self):
        super().__init__('tello_a_mavic_puente')

        self.sub_cam_mavic = self.create_subscription(
            Image, '/Mavic_2_PRO/camera/image_color', self.callback_camara, 10)
        self.pub_cam_tello = self.create_publisher(
            Image, '/tello/imagen', 10)

        self.sub_vel_tello = self.create_subscription(
            Float32MultiArray, '/tello/comandos_velocidad', self.callback_velocidad, 10)
        self.pub_vel_mavic = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.sub_imu_mavic = self.create_subscription(
            Imu, '/imu', self.callback_imu, 10)
        self.sub_gps_mavic = self.create_subscription(
            NavSatFix, '/Mavic_2_PRO/gps', self.callback_gps, 10)

        self.pub_pose_tello = self.create_publisher(
            Float32MultiArray, '/tello/pose_corregida', 10)

        self.posicion_actual = [0.0, 0.0, 0.0]
        self.orientacion_actual = [0.0, 0.0, 0.0]
        self.gps_origen = None
        self.R_TIERRA = 6371000

        self.timer_pose = self.create_timer(0.05, self.publicar_pose_corregida)
        self.get_logger().info('Nodo Puente Tello (Ubuntu) <-> Mavic iniciado.')

    def callback_camara(self, msg_in):
        self.pub_cam_tello.publish(msg_in)

    def callback_velocidad(self, msg_in):
        # msg_in.data = [lr, fb, ud, yv]
        # Según tu definición:
        # lr (Y): +Derecha
        # fb (X): +Adelante
        # ud (Z): +Abajo
        # yv (Yaw): +Derecha (Clockwise)
        
        lr_in, fb_in, ud_in, yv_in = msg_in.data
        
        # --- PASO 1: Mapear a Sistema Webots MUNDO (ENU: +X Este, +Y Norte, +Z Arriba) ---
        
        # X: Tello Adelante (+) -> Webots Adelante (+)
        vx_mundo = fb_in * FACTOR_ESCALA_VELOCIDAD
        
        # Y: Tello Derecha (+) -> Webots Izquierda (+). SON OPUESTOS.
        # Invertimos el signo para que coincidan (Si entra +Derecha, queremos ir a la Derecha que es -Y)
        vy_mundo = -lr_in * FACTOR_ESCALA_VELOCIDAD
        
        # Z: Tello Abajo (+) -> Webots Arriba (+). SON OPUESTOS.
        # Si tu nodo envía + para bajar, y Webots necesita - para bajar, invertimos.
        # PERO, si tu nodo calcula (Target - Current) y Target es más alto, enviará POSITIVO.
        # Si envías positivo para subir, y Tello Z es abajo, tu lógica anterior estaba invertida.
        # Asumiremos la lógica estándar de navegación (Target - Current):
        # Si quiero subir: Input es Positivo -> Webots necesita Positivo.
        vz_mundo = ud_in * FACTOR_ESCALA_VELOCIDAD 

        # Yaw: Tello Clockwise (+) -> Webots Counter-Clockwise (+). SON OPUESTOS.
        wz_cuerpo = -yv_in * FACTOR_ESCALA_YAW

        # --- PASO 2: Rotación MUNDO -> CUERPO ---
        # El dron recibe comandos relativos a su nariz, no al Norte.
        yaw_actual = self.orientacion_actual[2]
        c = math.cos(yaw_actual)
        s = math.sin(yaw_actual)

        # Matriz de rotación estándar 2D
        vx_cuerpo = vx_mundo * c + vy_mundo * s
        vy_cuerpo = -vx_mundo * s + vy_mundo * c

        msg_out = Twist()
        msg_out.linear.x = float(vx_cuerpo)
        msg_out.linear.y = float(vy_cuerpo)
        msg_out.linear.z = float(vz_mundo) # Z no se rota
        msg_out.angular.z = float(wz_cuerpo)
        
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
        roll_deg = math.degrees(self.orientacion_actual[0])
        pitch_deg = math.degrees(self.orientacion_actual[1])
        yaw_deg = math.degrees(self.orientacion_actual[2])
        
        # [x, y, z, roll, pitch, yaw]
        data_out = [
            self.posicion_actual[0],
            self.posicion_actual[1],
            self.posicion_actual[2],
            roll_deg,
            pitch_deg,
            yaw_deg 
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
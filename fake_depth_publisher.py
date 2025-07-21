#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 # Necesario para np.full, aunque no para procesar imagen real
import time

# --- CONFIGURACIÓN ---
ROS_TOPIC_OUTPUT_DEPTH = '/depth_camera' # Asegúrate que coincida con depth_image_topic_name en el YAML
TIMER_PERIOD_DEPTH = 1.0 / 30.0         # Publicar a 30 Hz

# Estas dimensiones DEBEN COINCIDIR con las de tu imagen RGB del Tello
# y con Camera.width/height en gazebo_rgbd.yaml
FRAME_WIDTH_DEPTH = 640
FRAME_HEIGHT_DEPTH = 480

# Valor de profundidad constante (ej. 1 metro)
FAKE_DEPTH_VALUE = 1.0 # En metros (float32)

class FakeDepthPublisher(Node):
    def __init__(self):
        super().__init__('fake_depth_publisher')
        self.get_logger().info("Iniciando nodo publicador de profundidad falsa.")
        self.get_logger().info(f"Publicando profundidad en: {ROS_TOPIC_OUTPUT_DEPTH}")

        self.bridge = CvBridge()

        # Crear Publicador ROS con el mismo QoS que el Tello

        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE 
        )
        self.depth_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT_DEPTH, qos_profile_publisher)
        self.get_logger().info("Publicador de profundidad falsa ROS creado.")

        # Crear Temporizador ROS para publicar frames
        self.timer = self.create_timer(TIMER_PERIOD_DEPTH, self.timer_callback)
        self.get_logger().info(f"Temporizador creado con periodo: {TIMER_PERIOD_DEPTH:.3f}s")

    def timer_callback(self):
        """ Se ejecuta periódicamente para publicar un frame de profundidad falsa """
        # Crea una imagen de profundidad falsa con un valor constante
        fake_depth_array = np.full((FRAME_HEIGHT_DEPTH, FRAME_WIDTH_DEPTH),
                                   FAKE_DEPTH_VALUE, dtype=np.float32)

        # Convierte la imagen de NumPy a un mensaje ROS Image
        # Codificación '32FC1' para float de 32 bits, 1 canal (profundidad en metros)
        ros_depth_msg = self.bridge.cv2_to_imgmsg(fake_depth_array, encoding="32FC1")

        # Añadir timestamp y frame_id al mensaje ROS
        ros_depth_msg.header.stamp = self.get_clock().now().to_msg()
        ros_depth_msg.header.frame_id = "tello_camera" # <--- ¡IMPORTANTE! Coincide con el frame_id del Tello

        # Publicar el mensaje
        self.depth_publisher_.publish(ros_depth_msg)
        # self.get_logger().debug("Frame de profundidad falsa publicado en ROS.", throttle_duration_sec=1.0)


    def destroy_node(self):
        """ Se llama automáticamente al cerrar el nodo """
        self.get_logger().info("Cerrando nodo publicador de profundidad falsa...")
        if self.timer:
            self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FakeDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo de profundidad falsa...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Publicador de profundidad falsa finalizado.")

if __name__ == '__main__':
    main()
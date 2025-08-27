#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

# --- CONFIGURACIÓN ---
ROS_TOPIC_OUTPUT_IMAGE = '/tello/imagen'  # Topic donde se publica la imagen BGR
TIMER_PERIOD_IMAGE = 1.0 / 30.0           # Publicar a 30 Hz
RUTA_IMAGEN = '/home/alvaro/DronTello/TopicsFalsos/imagen.jpg'

class FakeBGRPublisher(Node):
    def __init__(self):
        super().__init__('fake_bgr_publisher')
        self.get_logger().info("Iniciando nodo publicador de imagen BGR.")

        self.bridge = CvBridge()
        self.imagen_cargada = cv2.imread(RUTA_IMAGEN)

        if self.imagen_cargada is None:
            self.get_logger().warning(f"No se pudo cargar la imagen: {RUTA_IMAGEN}. Se publicará imagen negra.")
            self.imagen_cargada = cv2.zeros((480, 640, 3), dtype='uint8')

        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.image_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT_IMAGE, qos_profile_publisher)
        self.timer = self.create_timer(TIMER_PERIOD_IMAGE, self.timer_callback)

    def timer_callback(self):
        # Convertir la imagen BGR a mensaje ROS
        ros_image_msg = self.bridge.cv2_to_imgmsg(self.imagen_cargada, encoding="bgr8")
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = "tello_camera_link_raw"
        self.image_publisher_.publish(ros_image_msg)

    def destroy_node(self):
        if self.timer:
            self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FakeBGRPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Publicador de imagen BGR finalizado.")

if __name__ == '__main__':
    main()

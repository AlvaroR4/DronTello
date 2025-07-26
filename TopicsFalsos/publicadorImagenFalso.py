#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os 


RUTA_IMAGEN_A_PUBLICAR = '/home/alvaro/Desktop/imagen.jpg' 


ROS_TOPIC_OUTPUT_IMAGE = '/tello/imagen_raw'
TIMER_PERIOD_IMAGE = 1.0 / 30.0         # Publicar a 30 Hz

class FakeImagePublisher(Node):
    def __init__(self):
        super().__init__('fake_image_publisher')
        self.get_logger().info("Iniciando nodo de imagen falsa.")

        self.bridge = CvBridge()
        self.imagen_cargada = None
        
        self.imagen_cargada = cv2.imread(RUTA_IMAGEN_A_PUBLICAR)

        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE 
        )
        self.image_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT_IMAGE, qos_profile_publisher)

        self.timer = self.create_timer(TIMER_PERIOD_IMAGE, self.timer_callback)

    def timer_callback(self):
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
    node = FakeImagePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Publicador finalizado.")

if __name__ == '__main__':
    main()

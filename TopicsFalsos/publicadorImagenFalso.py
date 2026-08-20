#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os 


RUTA_IMAGEN_A_PUBLICAR = '/home/alvaro/DronTello/TopicsFalsos/imagen.png' 


ROS_TOPIC_OUTPUT_IMAGE = '/camera/image_raw'
TIMER_PERIOD_IMAGE = 1.0 / 30.0         # Publicar a 30 Hz

class FakeImagePublisher(Node):
    def __init__(self):
        super().__init__('fake_image_publisher')
        self.get_logger().info("Iniciando nodo de imagen falsa.")

        self.bridge = CvBridge()
        
        # Leer la imagen en BGR (por defecto) y pasarla a RGB
        self.imagen_cargada = cv2.imread(RUTA_IMAGEN_A_PUBLICAR)
        self.imagen_cargada = cv2.cvtColor(self.imagen_cargada, cv2.COLOR_BGR2RGB)

        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE 
        )
        self.image_publisher_ = self.create_publisher(Image, ROS_TOPIC_OUTPUT_IMAGE, qos_profile_publisher)
        self.timer = self.create_timer(TIMER_PERIOD_IMAGE, self.timer_callback)

    def timer_callback(self):
        ros_image_msg = self.bridge.cv2_to_imgmsg(self.imagen_cargada, encoding="rgb8")
        ros_image_msg.header.stamp = self.get_clock().now().to_msg()
        ros_image_msg.header.frame_id = "tello_camera_link_raw" 
        self.image_publisher_.publish(ros_image_msg)

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

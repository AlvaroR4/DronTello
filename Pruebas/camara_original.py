#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import time

TELLO_IP = '192.168.10.1'
ROS_TOPIC_OUTPUT = 'camera/image_raw'

class TelloRawPublisher(Node):
    def __init__(self):
        super().__init__('tello_raw_publisher')
        self.bridge = CvBridge()
        self.tello = Tello(host=TELLO_IP)

        self.tello.connect()
        self.get_logger().info(f"Conectado al Tello. Batería: {self.tello.get_battery()}%")

        self.tello.streamoff()
        time.sleep(0.5)
        self.tello.streamon()
        self.frame_reader = self.tello.get_frame_read()
        time.sleep(1.0)

        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(Image, ROS_TOPIC_OUTPUT, qos_profile_publisher)

        # 30 Hz aprox
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        frame = self.frame_reader.frame
        if frame is None:
            self.get_logger().warn("Frame vacío del Tello")
            return

        # Publicar tal cual lo entrega djitellopy (BGR, tamaño nativo 720p)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tello_camera"
        self.pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo...")
        self.tello.streamoff()
        self.tello.end()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = TelloRawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

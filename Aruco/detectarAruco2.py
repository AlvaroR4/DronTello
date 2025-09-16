#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import traceback

INPUT_IMAGE_TOPIC = 'camera/image_raw'          
OUTPUT_IMAGE_TOPIC = '/tello/imagen_aruco'
OUTPUT_DISTANCE_TOPIC = '/tello/aruco_distance' 

ARUCO_DICT = cv2.aruco.DICT_5X5_250
MARKER_SIZE = 0.175  # m
PROC_WIDTH = 960
PROC_HEIGHT = 710

# Matriz de cámara y distorsión
camera_matrix = np.array([[900.1766, 0, 481.5253],
                          [0, 894.8176, 371.0677],
                          [0, 0, 1]])
dist_coeffs = np.array([0.089014, -1.546625, 0.002167, 0.004498, 6.561574])

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.get_logger().info('Iniciando ArucoDetectorNode')
        self.bridge = CvBridge()

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_pub_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )
        qos_pub_dist = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=2
        )

        self.sub_image = self.create_subscription(Image, INPUT_IMAGE_TOPIC,
                                                 self.image_callback, qos_sub)
        self.pub_image = self.create_publisher(Image, OUTPUT_IMAGE_TOPIC, qos_pub_img)
        self.pub_distance = self.create_publisher(Float32, OUTPUT_DISTANCE_TOPIC, qos_pub_dist)

        try:
            # OpenCV >= 4.7
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        except AttributeError:
            # versiones antiguas
            self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)

        try:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters()

        self._axis_draw_not_available_logged = False

        self.get_logger().info(f"Suscrito a: {INPUT_IMAGE_TOPIC} - Publicando imagen: {OUTPUT_IMAGE_TOPIC} y distancia: {OUTPUT_DISTANCE_TOPIC}")
        self.get_logger().info(f"Usando diccionario ArUco: {ARUCO_DICT}, tamaño marcador: {MARKER_SIZE} m")

    def image_callback(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen ROS->CV: {e}")
            return

        if frame_bgr is None:
            return

        frame_resized = cv2.resize(frame_bgr, (PROC_WIDTH, PROC_HEIGHT))
        img_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        img_gray = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2GRAY)

        # Detectar marcadores
        corners, ids, rejected = cv2.aruco.detectMarkers(img_gray, self.aruco_dict, parameters=self.aruco_params)

        img_out = frame_resized.copy()
        distance_to_publish = None

        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_SIZE, camera_matrix, dist_coeffs)

            for i, c in enumerate(corners):
                pts = c.reshape((4,2)).astype(int)
                cv2.polylines(img_out, [pts], True, (0,255,0), 2)

                # Posición del marcador en el mundo (tvec)
                # rvecs/tvecs pueden venir con forma (N,1,3) o (N,3) - normalizamos a vectores 1D
                rvec = np.array(rvecs[i]).reshape(3,)
                tvec = np.array(tvecs[i]).reshape(3,)
                dist_m = np.linalg.norm(tvec)

                cx = int(np.mean(pts[:,0])); cy = int(np.mean(pts[:,1]))
                x_min = int(np.min(pts[:,0])); x_max = int(np.max(pts[:,0]))
                y_min = int(np.min(pts[:,1])); y_max = int(np.max(pts[:,1]))

                cv2.rectangle(img_out, (x_min, y_min), (x_max, y_max), (255,0,0), 2)
                cv2.circle(img_out, (cx, cy), 4, (0,0,255), -1)
                cv2.putText(img_out, f"ID:{ids[i].item()} Dist:{dist_m:.2f}m",
                            (x_min, max(y_min-8,0)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

                if hasattr(cv2, 'drawFrameAxes'):
                    cv2.drawFrameAxes(img_out, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                distance_to_publish = dist_m

        try:
            img_publish_rgb = cv2.cvtColor(img_out, cv2.COLOR_BGR2RGB)
            ros_img_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="bgr8")
            ros_img_out.header = msg.header
            ros_img_out.header.frame_id = "camera_aruco_processed"
            self.pub_image.publish(ros_img_out)
        except CvBridgeError as e:
            self.get_logger().error(f"Error publicando imagen: {e}")

        if distance_to_publish is not None:
            msgd = Float32()
            msgd.data = float(distance_to_publish)
            self.pub_distance.publish(msgd)

    def destroy_node(self):
        self.get_logger().info("Cerrando ArucoDetectorNode")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().fatal(f"Error: {e}\n{traceback.format_exc()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

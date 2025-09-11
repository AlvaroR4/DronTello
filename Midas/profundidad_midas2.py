
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from cv_bridge import CvBridge, CvBridgeError

import torch
import numpy as np
import os
import time
import traceback

class MidasPublisherNode(Node):
    def __init__(self):
        super().__init__('midas_publisher_node')

        # Parámetros (pueden sobreescribirse al lanzar el nodo)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('publish_multiarray', True)
        self.declare_parameter('save_txt', True)
        self.declare_parameter('txt_folder', '/tmp/midas_depths')
        self.declare_parameter('model_type', 'MiDaS_small')
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('depth_topic', '/midas/depth_display')
        self.declare_parameter('multiarray_topic', '/midas/depth_multiarray')

        self.publish_image = self.get_parameter('publish_image').get_parameter_value().bool_value
        self.publish_multiarray = self.get_parameter('publish_multiarray').get_parameter_value().bool_value
        self.save_txt = self.get_parameter('save_txt').get_parameter_value().bool_value
        self.txt_folder = self.get_parameter('txt_folder').get_parameter_value().string_value
        self.model_type = self.get_parameter('model_type').get_parameter_value().string_value
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.multiarray_topic = self.get_parameter('multiarray_topic').get_parameter_value().string_value

        self.get_logger().info(f"Parametros: publish_image={self.publish_image}, publish_multiarray={self.publish_multiarray}, save_txt={self.save_txt}")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")
        self.get_logger().info(f"Depth image topic: {self.depth_topic}")
        self.get_logger().info(f"Depth multiarray topic: {self.multiarray_topic}")
        self.get_logger().info(f"Modelo MiDaS: {self.model_type}")
        self.get_logger().info(f"Carpeta txt: {self.txt_folder}")

        # Crear carpeta si hace falta
        if self.save_txt:
            try:
                os.makedirs(self.txt_folder, exist_ok=True)
            except Exception as e:
                self.get_logger().error(f"No se pudo crear la carpeta {self.txt_folder}: {e}")

        # Suscripción al topic de la cámara RGB
        qos_sub = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.listener_callback,
            qos_sub
        )
        self.bridge = CvBridge()

        # Cargar modelo MiDaS
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Cargando modelo MiDaS ({self.model_type}) en {self.device}...")
        try:
            # Carga del modelo y transformaciones
            self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type, trust_repo=True)
            self.midas.to(self.device)
            self.midas.eval()

            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            if self.model_type == "MiDaS_small":
                self.transform = midas_transforms.small_transform
            else:
                # asume DPT variants u otras
                self.transform = midas_transforms.dpt_transform

            self.get_logger().info("Modelo MiDaS y transformaciones cargadas correctamente.")
        except Exception as e:
            self.get_logger().fatal(f"Error cargando modelo MiDaS: {e}")
            traceback.print_exc()
            rclpy.shutdown()
            raise SystemExit(f"Fallo al cargar modelo MiDaS: {e}")

        # Publicadores según parámetros
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        if self.publish_image:
            self.pub_image = self.create_publisher(Image, self.depth_topic, qos_pub)
            self.get_logger().info(f"Publicador Image creado en: {self.depth_topic}")
        else:
            self.pub_image = None

        if self.publish_multiarray:
            self.pub_multi = self.create_publisher(Float32MultiArray, self.multiarray_topic, qos_pub)
            self.get_logger().info(f"Publicador Float32MultiArray creado en: {self.multiarray_topic}")
        else:
            self.pub_multi = None

        self.save_counter = 0

    def listener_callback(self, msg):
        """
        Recibe Image (camera), calcula profundidad, y:
         - publica Image 32FC1
         - publica Float32MultiArray con layout
         - guarda matrix en txt
        """
        try:
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image_rgb = cv_image_bgr[:, :, ::-1]  # BGR -> RGB
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge al recibir: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen recibida: {e}")
            return

        try:
            # Preparar input y hacer inferencia
            input_batch = self.transform(cv_image_rgb).to(self.device)

            with torch.no_grad():
                prediction = self.midas(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=cv_image_rgb.shape[:2],  # (height, width)
                    mode="bicubic",
                    align_corners=False
                ).squeeze()

            depth_map = prediction.cpu().numpy().astype(np.float32)  # shape (H, W)

            # --- 1) Publicar como sensor_msgs/Image (32FC1) ---
            if self.publish_image and self.pub_image is not None:
                try:
                    img_msg_out = self.bridge.cv2_to_imgmsg(depth_map, encoding="32FC1")
                    img_msg_out.header.stamp = msg.header.stamp
                    img_msg_out.header.frame_id = msg.header.frame_id
                    self.pub_image.publish(img_msg_out)
                    # debug/ info con throttle
                    self.get_logger().debug("Publicado depth Image (32FC1).")
                except Exception as e:
                    self.get_logger().error(f"Error publicando Image depth: {e}")

            # --- 2) Publicar como Float32MultiArray con layout (height, width) ---
            if self.publish_multiarray and self.pub_multi is not None:
                try:
                    fa = Float32MultiArray()
                    # Flatten en row-major (C) order
                    fa.data = depth_map.flatten().tolist()

                    # Dimensiones: dim[0]=height, dim[1]=width
                    h, w = depth_map.shape
                    dim_h = MultiArrayDimension()
                    dim_h.label = "height"
                    dim_h.size = int(h)
                    dim_h.stride = int(h * w)

                    dim_w = MultiArrayDimension()
                    dim_w.label = "width"
                    dim_w.size = int(w)
                    dim_w.stride = int(w)

                    fa.layout.dim.append(dim_h)
                    fa.layout.dim.append(dim_w)
                    # opcional: fa.layout.data_offset = 0

                    self.pub_multi.publish(fa)
                    self.get_logger().debug("Publicado depth Float32MultiArray (layout con height/width).")
                except Exception as e:
                    self.get_logger().error(f"Error publicando Float32MultiArray: {e}")

            # --- 3) Guardar la matriz en un .txt ---
            if self.save_txt:
                try:
                    ts = time.time()
                    ts_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(ts))
                    micro = int((ts - int(ts)) * 1e6)
                    filename = f"depth_{ts_str}_{micro:06d}_{self.save_counter:06d}.txt"
                    filepath = os.path.join(self.txt_folder, filename)
                    # np.savetxt acepta 2D arrays, guarda filas en líneas
                    # Usamos formato con 6 decimales (ajustable)
                    np.savetxt(filepath, depth_map, fmt="%.6f", delimiter=' ')
                    self.save_counter += 1
                    self.get_logger().debug(f"Depth guardado en TXT: {filepath}")
                except Exception as e:
                    self.get_logger().error(f"Error guardando depth en txt: {e}")

            # También (opcional) podrías imprimir en consola la matriz en formato cuadriculado
            # Si quieres activar esto, cambia a logging.debug o print aquí. Por defecto lo dejamos comentado para no spam.
            # self.get_logger().debug("\n" + np.array2string(depth_map, precision=2, suppress_small=True))

        except Exception as e_proc:
            self.get_logger().error(f"Error durante procesamiento MiDaS: {e_proc}")
            traceback.print_exc()

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo publicador MiDaS...")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MidasPublisherNode()
        if node:
            node.get_logger().info("Nodo MiDaS listo. Ejecutando rclpy.spin() ...")
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        if node:
            try:
                node.destroy_node()
            except Exception as de:
                print(f"Error during destroy_node: {de}")
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa finalizado.")


if __name__ == '__main__':
    main()

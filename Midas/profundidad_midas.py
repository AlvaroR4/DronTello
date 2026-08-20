import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import torch
import numpy as np
import cv2
import traceback

CAMARA_TOPIC_ENTRADA = 'camera/image_raw'
DEPTH_MAP_TOPIC_SALIDA = '/midas/depth_display2' 

MODEL_TYPE = "MiDaS_small" 

class MidasPublisherNode(Node):
    def __init__(self):
        super().__init__('midas_publisher_node')
        self.get_logger().info(f"Iniciando Nodo Publicador MiDaS.")
        self.get_logger().info(f"Suscribiendo a: {CAMARA_TOPIC_ENTRADA}")
        self.get_logger().info(f"Publicando en: {DEPTH_MAP_TOPIC_SALIDA}")

        # Suscripción al topic de la cámara RGB
        self.subscription = self.create_subscription(
            Image,
            CAMARA_TOPIC_ENTRADA,
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

        # Cargar modelo MiDaS
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Cargando modelo MiDaS ({MODEL_TYPE}) en {self.device}...")
        try:
            self.midas = torch.hub.load("intel-isl/MiDaS", MODEL_TYPE, trust_repo=True)
            self.midas.to(self.device)
            self.midas.eval()

            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms", trust_repo=True)
            self.transform = midas_transforms.small_transform if MODEL_TYPE == "MiDaS_small" else midas_transforms.dpt_transform
            self.get_logger().info("Modelo MiDaS y transformaciones cargadas.")
        except Exception as e:
            self.get_logger().fatal(f"Error cargando modelo MiDaS: {e}.")
            traceback.print_exc()
            rclpy.shutdown()
            raise SystemExit(f"Fallo al cargar modelo MiDaS: {e}")

        # Publicador de la imagen de profundidad
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publisher_ = self.create_publisher(
            Image,
            DEPTH_MAP_TOPIC_SALIDA,
            qos_profile_publisher
        )
        self.get_logger().info(f"Publicador creado en '{DEPTH_MAP_TOPIC_SALIDA}'.")

    def listener_callback(self, msg):
        try:
            # Convertir mensaje ROS a OpenCV
            cv_image_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image_rgb = cv_image_bgr[:, :, ::-1]  # BGR -> RGB
        except CvBridgeError as e:
            self.get_logger().error(f"Error CvBridge al recibir: {e}")
            return
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen recibida: {e}")
            return

        try:
            # Inferencia con MiDaS
            input_batch = self.transform(cv_image_rgb).to(self.device)
            with torch.no_grad():
                prediction = self.midas(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=cv_image_rgb.shape[:2],  # Ajustar al tamaño de la RGB
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

            depth_map = prediction.cpu().numpy().astype(np.float32)

            # 1. Usar percentiles en lugar de mínimos y máximos absolutos
            # Ignoramos el 5% más lejano (cielo/fondo) y el 5% más cercano (muros/cámara)
            profundidad_minima = np.percentile(depth_map, 5)
            profundidad_maxima = np.percentile(depth_map, 95)

            # 2. Recortar la matriz (clip) para que los valores extremos no se salgan del rango
            depth_map_recortado = np.clip(depth_map, profundidad_minima, profundidad_maxima)

            # 3. Normalizar los valores al rango [0.0, 1.0] con el nuevo rango ajustado
            if profundidad_maxima - profundidad_minima > 0:
                mapa_normalizado = (depth_map_recortado - profundidad_minima) / (profundidad_maxima - profundidad_minima)
            else:
                mapa_normalizado = depth_map_recortado

            # 4. Escalar a [0, 255] y convertir a enteros de 8 bits
            mapa_uint8 = (mapa_normalizado * 255.0).astype(np.uint8)

            # 5. Aplicar el mapa de color
            mapa_color_calido = cv2.applyColorMap(mapa_uint8, cv2.COLORMAP_JET)

            # 6. Publicar como imagen en color estándar
            img_msg_out = self.bridge.cv2_to_imgmsg(mapa_color_calido, encoding="bgr8")
            img_msg_out.header.stamp = msg.header.stamp  
            img_msg_out.header.frame_id = msg.header.frame_id
            self.publisher_.publish(img_msg_out)

            self.get_logger().info("Mapa de profundidad publicado en color (bgr8).", throttle_duration_sec=1.0)

        except Exception as e_proc:
            self.get_logger().error(f"Error durante procesamiento MiDaS: {e_proc}")

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo publicador MiDaS...")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    midas_publisher_node = None
    try:
        midas_publisher_node = MidasPublisherNode()
        if midas_publisher_node and rclpy.ok():
            midas_publisher_node.get_logger().info("Iniciando bucle rclpy.spin()...")
            rclpy.spin(midas_publisher_node)
    except KeyboardInterrupt:
        print("Ctrl+C detectado, cerrando nodo...")
    except Exception as e:
        print(f"Error inesperado en main: {e}")
        traceback.print_exc()
    finally:
        print("Bloque finally en main...")
        if midas_publisher_node:
            try:
                midas_publisher_node.destroy_node()
            except Exception as destroy_e:
                print(f"Error durante destroy_node: {destroy_e}")
        if rclpy.ok():
            print("Cerrando rclpy...")
            rclpy.shutdown()
        print("Programa finalizado.")

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import traceback

# Definición de tópicos ROS2
ROS_TOPIC_IMAGEN_RAW_INPUT = '/tello/imagen'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'

# Dimensiones de procesamiento de la imagen
FRAME_WIDTH_PROC = 640
FRAME_HEIGHT_PROC = 480

# Rangos HSV para el color azul (para las esquinas de la puerta)
# Estos rangos pueden necesitar ajuste según las condiciones de iluminación.
COLOR_LOWER_BLUE = np.array([100, 150, 50])  # Tono, Saturación, Valor (mínimos)
COLOR_UPPER_BLUE = np.array([140, 255, 255]) # Tono, Saturación, Valor (máximos)

# Área mínima de un contorno para ser considerado una esquina
MIN_CORNER_AREA = 100 # Ajustar según el tamaño esperado de las esquinas en la imagen

class ModuloLocalizacion(Node):
    """
    Nodo ROS2 para la detección de puertas en las imágenes del dron Tello.
    Recibe imágenes RAW y publica una lista de puertas detectadas,
    además de una imagen con la visualización de las detecciones.
    """
    def __init__(self):
        super().__init__('modulo_localizacion')
        self.get_logger().info("Iniciando Módulo de Localización (Detección de Puertas).")
        self.bridge = CvBridge()

        # Suscriptor para la imagen RAW
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.suscripcion_imagen = self.create_subscription(
            Image,
            ROS_TOPIC_IMAGEN_RAW_INPUT,
            self.callback_procesamiento_imagen,
            qos_profile_sub
        )
        self.get_logger().info(f"Suscrito a imagen RAW en: {ROS_TOPIC_IMAGEN_RAW_INPUT}")

        # Publicador para la lista de puertas detectadas
        # El formato será: [num_puertas, x1, y1, w1, h1, x2, y2, w2, h2, ...]
        qos_profile_pub_data = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publicador_puertas_detectadas = self.create_publisher(
            Float32MultiArray, ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT, qos_profile_pub_data)
        self.get_logger().info(f"Publicando datos de puertas detectadas en: {ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT}")

        # Publicador para la imagen con visualización de las detecciones
        qos_profile_pub_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.publicador_imagen_visualizacion = self.create_publisher(
            Image, ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT, qos_profile_pub_img)
        self.get_logger().info(f"Publicando imagen de visualización en: {ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT}")

    def callback_procesamiento_imagen(self, msg_imagen_ros):
        """
        Callback que se ejecuta cada vez que se recibe una nueva imagen RAW.
        Realiza el pre-procesamiento y llama al algoritmo de detección de puertas.
        """
        try:
            # Convertir mensaje ROS Image a frame OpenCV (BGR8)
            frame_bgr_raw = self.bridge.imgmsg_to_cv2(msg_imagen_ros, desired_encoding="bgr8")
        except CvBridgeError as e_bridge:
            self.get_logger().error(f"Error CvBridge al convertir imagen RAW: {e_bridge}")
            return
        except Exception as e_conversion:
            self.get_logger().error(f"Error general convirtiendo imagen RAW: {e_conversion}")
            return

        if frame_bgr_raw is None:
            self.get_logger().warn("Frame BGR recibido es None después de conversión.", throttle_duration_sec=5.0)
            return

        try:
            # Redimensionar el frame para un procesamiento más rápido
            img_procesamiento = cv2.resize(frame_bgr_raw, (FRAME_WIDTH_PROC, FRAME_HEIGHT_PROC))
            
            # Convertir a HSV para la segmentación de color
            img_hsv = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2HSV)

            # Llamar al algoritmo de detección de puertas
            puertas_detectadas, img_con_dibujos = self.algoritmoDetectarPuertas(img_hsv, img_procesamiento.copy())

            # Publicar los datos de las puertas detectadas
            msg_puertas = Float32MultiArray()
            # Formato: [num_puertas, x1, y1, w1, h1, x2, y2, w2, h2, ...]
            data_to_publish = [float(len(puertas_detectadas))]
            for puerta in puertas_detectadas:
                data_to_publish.extend([float(puerta['x_centro']), float(puerta['y_centro']),
                                        float(puerta['ancho']), float(puerta['alto'])])
            
            msg_puertas.data = data_to_publish
            self.publicador_puertas_detectadas.publish(msg_puertas)

            # Publicar la imagen con las visualizaciones
            try:
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_con_dibujos, encoding="bgr8")
                ros_image_msg_out.header.stamp = msg_imagen_ros.header.stamp
                ros_image_msg_out.header.frame_id = "tello_camera_processed_localization"
                self.publicador_imagen_visualizacion.publish(ros_image_msg_out)
            except CvBridgeError as e_cv_bridge_pub:
                self.get_logger().error(f"Error CvBridge al convertir/publicar imagen de visualización: {e_cv_bridge_pub}")
            except Exception as e_publish_general:
                self.get_logger().error(f"Error general al publicar visualización de puertas: {e_publish_general}")

        except Exception as e_processing_callback:
            self.get_logger().error(f"Error general en procesamiento del callback_procesamiento_imagen: {e_processing_callback}")
            self.get_logger().error(traceback.format_exc())

    def algoritmoDetectarPuertas(self, img_hsv, img_visualizacion):
        """
        Algoritmo principal para la detección de puertas.
        1. Detecta los puntos de las esquinas (en esta versión, azules).
        2. Agrupa esos puntos para formar puertas.
        
        Args:
            img_hsv (numpy.array): Imagen en espacio de color HSV.
            img_visualizacion (numpy.array): Copia de la imagen original para dibujar las detecciones.

        Returns:
            tuple: Una tupla que contiene:
                - list: Lista de diccionarios, donde cada diccionario representa una puerta detectada
                        con sus propiedades (ej. 'x_centro', 'y_centro', 'ancho', 'alto' del centro).
                - numpy.array: La imagen con las visualizaciones de las detecciones.
        """
        # --- Fase 1: Detección de puntos individuales de las esquinas ---
        puntos_esquinas_detectados = [] # Lista de tuplas (cx, cy)

        # 1. Segmentar las esquinas azules
        mask_azul = cv2.inRange(img_hsv, COLOR_LOWER_BLUE, COLOR_UPPER_BLUE)

        # Opcional: Operaciones morfológicas para limpiar la máscara (abrir y cerrar)
        kernel = np.ones((5,5), np.uint8)
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, kernel)
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, kernel)

        # 2. Encontrar contornos en la máscara
        contornos, _ = cv2.findContours(mask_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 3. Filtrar contornos para identificar posibles esquinas
        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > MIN_CORNER_AREA:
                # Calcular el momento para encontrar el centro del contorno (cx, cy)
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    puntos_esquinas_detectados.append((cx, cy))
                    
                    # Dibujar las esquinas detectadas en la imagen de visualización
                    cv2.circle(img_visualizacion, (cx, cy), 7, (255, 0, 0), -1) # Azul
                    cv2.putText(img_visualizacion, f"({cx},{cy})", (cx + 10, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        self.get_logger().info(f"Esquinas azules detectadas: {len(puntos_esquinas_detectados)}")

        # --- Fase 2: Agrupación de puntos para formar puertas ---
        puertas_encontradas = self.agruparPuntosEnPuertas(puntos_esquinas_detectados, img_visualizacion)
        
        return puertas_encontradas, img_visualizacion

    def agruparPuntosEnPuertas(self, puntos_esquinas, img_visualizacion):
        """
        Función encargada de agrupar los puntos de las esquinas detectadas en objetos "puerta".
        Esta es la función que se ampliará para manejar múltiples puertas, rotaciones, etc.

        Args:
            puntos_esquinas (list): Lista de tuplas (x, y) de los centros de las esquinas detectadas.
            img_visualizacion (numpy.array): La imagen para dibujar las detecciones.

        Returns:
            list: Lista de diccionarios, donde cada diccionario representa una puerta detectada
                  con sus propiedades (ej. 'x_centro', 'y_centro', 'ancho', 'alto').
        """
        puertas = []

        # Versión Sencilla: Si hay al menos 4 puntos, asumimos que son una única puerta
        # sin rotación. Esta lógica será la que se mejorará en el futuro.
        if len(puntos_esquinas) >= 4:
            # Para una puerta sin rotación, simplemente encontramos el cuadro delimitador
            # de todos los puntos de las esquinas.
            x_coords = [p[0] for p in puntos_esquinas]
            y_coords = [p[1] for p in puntos_esquinas]

            x_min, x_max = min(x_coords), max(x_coords)
            y_min, y_max = min(y_coords), max(y_coords)

            ancho_puerta = x_max - x_min
            alto_puerta = y_max - y_min
            x_centro_puerta = x_min + ancho_puerta // 2
            y_centro_puerta = y_min + alto_puerta // 2

            # Pequeño umbral para evitar detecciones minúsculas por ruido
            if ancho_puerta > 50 and alto_puerta > 50: 
                puerta_detectada = {
                    'x_centro': x_centro_puerta,
                    'y_centro': y_centro_puerta,
                    'ancho': ancho_puerta,
                    'alto': alto_puerta
                }
                puertas.append(puerta_detectada)

                # Dibujar el rectángulo de la puerta y su centro en la imagen de visualización
                cv2.rectangle(img_visualizacion, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2) # Verde
                cv2.circle(img_visualizacion, (x_centro_puerta, y_centro_puerta), 5, (0, 0, 255), -1) # Rojo (centro)
                cv2.putText(img_visualizacion, f"Puerta ({x_centro_puerta},{y_centro_puerta})",
                            (x_centro_puerta - 50, y_centro_puerta - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        
        return puertas

    def destroy_node(self):
        """
        Método para limpiar recursos al destruir el nodo.
        """
        self.get_logger().info("Destruyendo ModuloLocalizacion...")
        super().destroy_node()

def main(args=None):
    """
    Función principal para inicializar y ejecutar el nodo.
    """
    rclpy.init(args=args)
    modulo_localizacion = None
    try:
        modulo_localizacion = ModuloLocalizacion()
        rclpy.spin(modulo_localizacion)
    except KeyboardInterrupt:
        if modulo_localizacion:
            modulo_localizacion.get_logger().info("Ctrl+C detectado, cerrando ModuloLocalizacion.")
        else:
            print("Ctrl+C detectado antes de inicializar ModuloLocalizacion.")
    except Exception as e_main:
        if modulo_localizacion:
            modulo_localizacion.get_logger().fatal(f"Error inesperado en main de ModuloLocalizacion: {e_main}")
            modulo_localizacion.get_logger().fatal(traceback.format_exc())
        else:
            print(f"Error inesperado en main antes de inicializar ModuloLocalizacion: {e_main}")
            print(traceback.format_exc())
    finally:
        if modulo_localizacion:
            modulo_localizacion.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Programa ModuloLocalizacion finalizado.")

if __name__ == '__main__':
    main()
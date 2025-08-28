import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import traceback
import math

# Definición de tópicos ROS2
ROS_TOPIC_IMAGEN_RAW_INPUT = '/tello/imagen'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'

# Dimensiones de procesamiento de la imagen
ANCHO_TOTAL = 640
ALTO_TOTAL = 480

# Rangos HSV

#Amarillo
#COLOR_MIN = np.array([20, 230, 100])  
#COLOR_MAX = np.array([35, 255, 255])
#Verde
#COLOR_MIN = np.array([45, 120, 80])
#COLOR_MAX = np.array([75, 255, 255])
#Naranja
COLOR_MIN = np.array([0, 178, 145])  
COLOR_MAX = np.array([14, 255, 255])

# Área mínima de un contorno para ser considerado una esquina
MIN_CORNER_AREA = 100 # Ajustar según el tamaño esperado de las esquinas en la imagen

ALTO_REAL = 0.29
ANCHO_REAL = 0.20
FOCAL = 617.0
FOV_H = 67.2
FOV_V = 52.3

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

        #temporizador que se ejecuta cada 3 segundos
        self.timer_log = self.create_timer(3.0, self.log_datos)
        self.distancia_estimada = None
        self.distancia_calculada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None

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


    def estimar_distancia(self, alto_puerta_px):
        # Fórmula: Distancia = (Ancho_Real * Longitud_Focal) / Ancho_en_Píxeles
        distancia_mts = (ALTO_REAL * FOCAL) / alto_puerta_px
        return distancia_mts
    
        
    def punto_cuerpo_a_mundo(self, roll_deg, pitch_deg, yaw_deg, pos_dron_mundo, punto_cuerpo):
        """
        Transforma un punto expresado en ejes cuerpo FRD (Forward, Right, Down)
        al sistema de coordenadas mundo usado por ORB-SLAM (X adelante, Y izquierda, Z arriba).

        Parámetros:
        -----------
        roll_deg, pitch_deg, yaw_deg : float
            Ángulos de orientación del dron en grados (roll, pitch, yaw).
            - roll: rotación sobre eje X cuerpo (forward)
            - pitch: rotación sobre eje Y cuerpo (right)
            - yaw: rotación sobre eje Z cuerpo (down)
        pos_dron_mundo : array-like (3,)
            Posición del dron en coordenadas mundo [x, y, z] (proporcionada por ORB-SLAM).
        punto_cuerpo : array-like (3,)
            Coordenadas del objeto detectado en ejes cuerpo FRD [x_forward, y_right, z_down].

        Devuelve:
        --------
        punto_mundo : ndarray (3,)
            Coordenadas del punto en el sistema mundo de ORB-SLAM.
        """

        # 1) Conversión de grados a radianes
        roll = np.deg2rad(roll_deg)
        pitch = np.deg2rad(pitch_deg)
        yaw = np.deg2rad(yaw_deg)

        # 2) Matrices de rotación elementales
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]
        ])

        R_y = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1]
        ])

        # 3) Matriz de corrección FRD -> FLU (porque ORB usa Y izquierda, Z arriba)
        S = np.diag([1, -1, -1])

        # 4) Matriz de rotación cuerpo -> mundo
        R = R_z @ R_y @ R_x @ S

        # 5) Transformar el punto: primero rotación, luego traslación
        punto_cuerpo = np.array(punto_cuerpo).reshape(3,)
        pos_dron_mundo = np.array(pos_dron_mundo).reshape(3,)
        punto_mundo = pos_dron_mundo + R @ punto_cuerpo

        return punto_mundo

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
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_TOTAL, ALTO_TOTAL))
            img_proc = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2RGB)
            # Convertir a HSV para la segmentación de color
            img_hsv = cv2.cvtColor(img_proc, cv2.COLOR_BGR2HSV)

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
                img_publish_rgb = cv2.cvtColor(img_con_dibujos, cv2.COLOR_BGR2RGB)
                ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_publish_rgb, encoding="bgr8")
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
        1. Detecta los puntos de las esquinas.
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

        # 1. Segmentar las esquinas 
        mask_azul = cv2.inRange(img_hsv, COLOR_MIN, COLOR_MAX)

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

        #self.get_logger().info(f"Esquinas detectadas: {len(puntos_esquinas_detectados)}")

        # --- Fase 2: Agrupación de puntos para formar puertas ---
        puertas_encontradas = self.tratarPuerta(puntos_esquinas_detectados, img_visualizacion)
        
        return puertas_encontradas, img_visualizacion
    
    

    def tratarPuerta(self, puntos_esquinas, img_visualizacion):
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
        if len(puntos_esquinas) >= 4:
            
            #ORDENAR ESQUINAS
            x_sorted = sorted(puntos_esquinas, key=lambda p: p[0])

            x_menor = x_sorted[0]
            x_menor2 = x_sorted[1]
            x_menor3 = x_sorted[2]
            x_menor4 = x_sorted[3]
            
            # esq4   esq3
            # esq1   esq2

            if (x_menor[1] > x_menor2[1]):
                esq1 = x_menor
                esq4 = x_menor2
            else:
                esq1 = x_menor2
                esq4 = x_menor

            if (x_menor3[1] > x_menor4[1]):
                esq2 = x_menor3
                esq3 = x_menor4
            else:
                esq2 = x_menor4
                esq3 = x_menor3

            ancho_puerta = math.sqrt((esq2[0] - esq1[0])**2 + (esq2[1] - esq1[1])**2)
            alto_puerta = math.sqrt((esq1[0] - esq4[0])**2 + (esq1[1] - esq4[1])**2)

            if ancho_puerta > alto_puerta:
                x = ancho_puerta
                ancho_puerta = alto_puerta
                alto_puerta = x

            #CALCULAR CENTRO, ÁNGULO ,PROPORCIÓN Y DISTANCIA DE LA PUERTA
            proporcion = ancho_puerta / alto_puerta
            proporcion_real = ANCHO_REAL / ALTO_REAL

            angulo = 90 -(90*proporcion/proporcion_real)

            x_centro_puerta = int(esq1[0] + ancho_puerta // 2)
            y_centro_puerta = int(esq4[1] + alto_puerta // 2)

            distancia_estimada = self.estimar_distancia(alto_puerta)

            #MÉTODO PARA CALCULAR LAS COORDENADAS DEL CENTRO DE LA PUERTA EN EJES CUERPO 
            fov_horizontal_rad = math.radians(FOV_H)
            fov_vertical_rad = math.radians(FOV_V)
            nx = (x_centro_puerta - ANCHO_TOTAL / 2) / (ANCHO_TOTAL / 2)
            ny = -(y_centro_puerta - ALTO_TOTAL / 2) / (ALTO_TOTAL / 2)

            angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
            angulo_vertical_rad = ny * (fov_vertical_rad / 2)

            coordenada_Z = -distancia_estimada * math.sin(angulo_vertical_rad)
            coordenada_Y = distancia_estimada * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
            coordenada_X = distancia_estimada * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)
            
            distancia_calculada = math.sqrt(coordenada_X**2 + coordenada_Y**2 + coordenada_Z**2)

            #CONVERTIR EL PUNTO DE EJES CUERPO(DRON) A EJES MUNDO (ORB-SLAM3)
            roll, pitch, yaw = 10, 5, 45 #datos de rotación(grados) que devolvería orb-slam
            pos_dron_mundo = [2, 3, 1] #posición del dron que devolvería orb-slam
            punto_cuerpo = [coordenada_X, coordenada_Y, coordenada_Z] #centro de la puerta en ejes mundo

            punto_mundo = self.punto_cuerpo_a_mundo(roll, pitch, yaw, pos_dron_mundo, punto_cuerpo)
            

            #IMPRIMIR DATOS
            self.distancia_estimada = distancia_estimada
            self.distancia_calculada = distancia_calculada
            self.coordenada_X = coordenada_X
            self.coordenada_Y = coordenada_Y
            self.coordenada_Z = coordenada_Z
            self.punto_mundo = punto_mundo

            #PUBLICAR DATOS DE LA PUERTA
            puerta_detectada = {
                'x_centro': x_centro_puerta,
                'y_centro': y_centro_puerta,
                'ancho': ancho_puerta,
                'alto': alto_puerta
            }
            puertas.append(puerta_detectada)

            #Dibujar el rectangulo
            esquinas = [esq1, esq2, esq3, esq4]
            puntos = np.array([[int(x), int(y)] for x, y in esquinas], np.int32)
            puntos = puntos.reshape((-1, 1, 2))
            cv2.polylines(img_visualizacion, [puntos], True, (0, 255, 0), 2)
            cv2.circle(img_visualizacion, (x_centro_puerta, y_centro_puerta), 5, (0, 0, 255), -1) # Rojo (centro)
            cv2.putText(img_visualizacion, f"Puerta ({x_centro_puerta},{y_centro_puerta},{angulo})",
                            (x_centro_puerta - 50, y_centro_puerta - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
        
        return puertas
    
    def log_datos(self):
        if self.distancia_estimada is not None:
            self.get_logger().info("--- DATOS ---")
            self.get_logger().info(f"Distancia a la puerta: {self.distancia_estimada:.2f} m")
            self.get_logger().info(f"Verificación: La distancia calculada es {self.distancia_calculada:.3f} m "
                                    f"(debería ser {self.distancia_estimada:.2f} m)")
            self.get_logger().info(f"Coordenada puerta: {self.coordenada_X:.2f} , "
                                    f"{self.coordenada_Y:.2f} , {self.coordenada_Z:.2f} m")
            self.get_logger().info(f"Coordenadas del punto en mundo (ORB-SLAM): {self.punto_mundo}")
            self.get_logger().info("-----------------------------")

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

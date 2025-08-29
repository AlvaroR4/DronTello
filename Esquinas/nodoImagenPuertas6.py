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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from transforms3d.euler import quat2euler 
from transforms3d.quaternions import quat2mat   # añadido para rotación directa de cuaternión

# Definición de tópicos ROS2
ROS_TOPIC_IMAGEN_RAW_INPUT = '/tello/imagen'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'

# Dimensiones de procesamiento de la imagen
ANCHO_IMAGEN = 640
ALTO_IMAGEN = 480

# Rangos HSV

#Amarillo
#COLOR_MIN = np.array([20, 230, 100])  
#COLOR_MAX = np.array([35, 255, 255])
#Verde
#COLOR_MIN = np.array([45, 120, 80])
#COLOR_MAX = np.array([75, 255, 255])
#Naranja
COLOR_MIN = np.array([0, 140, 145])  
COLOR_MAX = np.array([12, 255, 255])


# Área mínima de un contorno para ser considerado una esquina
MIN_CORNER_AREA = 50 # Ajustar según el tamaño esperado de las esquinas en la imagen

ALTO_REAL = 0.60
ANCHO_REAL = 0.40
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
        self.timer_log = self.create_timer(1.0, self.log_datos)
        self.distancia_estimada = None
        self.distancia_calculada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None
        self.normal_mundo = None
        self.angulo_orientacion = None

        self.publisher_ = self.create_publisher(PointStamped, '/punto', 10)

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

        # Suscriptor al tópico /robot_pose_slam
        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.suscripcion_pose = self.create_subscription(
            PoseStamped,
            '/robot_pose_slam',
            self.callback_pose,
            qos_profile_pose
        )

        self.get_logger().info("Suscrito a la pose del dron en: /robot_pose_slam")

        # Inicializamos variables de pose
        self.pos_dron_mundo = [0.0, 0.0, 0.0]
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.quaternion = [1.0, 0.0, 0.0, 0.0]   # (w,x,y,z) inicializado

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



    def callback_pose(self, msg: PoseStamped):
        # Posición
        self.pos_dron_mundo = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]

        # Orientación (cuaternión → almacenado directamente)
        q = msg.pose.orientation
        self.quaternion = [q.w, q.x, q.y, q.z]

        # También guardamos Euler para debug
        roll, pitch, yaw = quat2euler(self.quaternion, axes='sxyz')
        self.roll = np.rad2deg(roll)
        self.pitch = np.rad2deg(pitch)
        self.yaw = np.rad2deg(yaw)

        #self.get_logger().info(f"Pose recibida: pos=({self.pos_dron_mundo}), yaw={self.yaw:.2f}°")

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
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_IMAGEN, ALTO_IMAGEN))
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
    
    def publicar_punto(self, punto_mundo):
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(punto_mundo[0])
        msg.point.y = float(punto_mundo[1])
        msg.point.z = float(punto_mundo[2])

        self.publisher_.publish(msg)
        #self.get_logger().info(f'Publicado punto: {punto_mundo}')


    def estimar_distancia(self, alto_puerta_px):
        # Fórmula: Distancia = (Ancho_Real * Longitud_Focal) / Ancho_en_Píxeles
        distancia_mts = (ALTO_REAL * FOCAL) / alto_puerta_px
        return distancia_mts
    
        
    def punto_cuerpo_a_mundo(self, pos_dron_mundo, punto_cuerpo):
        """
        Transforma un punto expresado en ejes cuerpo FRD (Forward, Right, Down)
        al sistema de coordenadas mundo usado por ORB-SLAM (X adelante, Y izquierda, Z arriba).
        """
        # 1) Matriz de rotación cuerpo->mundo a partir del cuaternión
        R_wb = quat2mat(self.quaternion)

        # 2) Conversión FRD -> FLU
        S = np.diag([1, -1, -1])

        # 3) Transformar
        punto_cuerpo = np.array(punto_cuerpo).reshape(3,)
        pos_dron_mundo = np.array(pos_dron_mundo).reshape(3,)
        punto_mundo = pos_dron_mundo + R_wb @ (S @ punto_cuerpo)

        return punto_mundo
    
    def pixel_a_cuerpo(self, u, v, distancia):
        """
        Convierte coordenadas de píxel (u,v) de la imagen a coordenadas 3D en ejes cuerpo FRD.
        
        Parámetros:
        -----------
        u, v : int
            Coordenadas de la esquina en píxeles (imagen).
        distancia : float
            Distancia estimada a la puerta (misma usada para el centro).
        
        Devuelve:
        --------
        np.array([X, Y, Z]) en sistema cuerpo FRD
        """
        fov_horizontal_rad = math.radians(FOV_H)
        fov_vertical_rad = math.radians(FOV_V)

        # Normalizar píxeles [-1,1]
        nx = (u - ANCHO_IMAGEN / 2) / (ANCHO_IMAGEN / 2)
        ny = -(v - ALTO_IMAGEN / 2) / (ALTO_IMAGEN / 2)

        angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
        angulo_vertical_rad = ny * (fov_vertical_rad / 2)

        Z = -distancia * math.sin(angulo_vertical_rad)
        Y = distancia * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
        X = distancia * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)

        return np.array([X, Y, Z])


    def algoritmoDetectarPuertas(self, img_hsv, img_visualizacion):
        """
        Algoritmo principal para la detección de puertas.
        1. Detecta los puntos de las esquinas.
        2. Agrupa esos puntos para formar puertas.
        """
        puntos_esquinas_detectados = [] # Lista de tuplas (cx, cy)

        # 1. Segmentar las esquinas 
        mask_azul = cv2.inRange(img_hsv, COLOR_MIN, COLOR_MAX)

        # Opcional: Operaciones morfológicas para limpiar la máscara (abrir y cerrar)
        kernel = np.ones((5,5), np.uint8)
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_OPEN, kernel)
        mask_azul = cv2.morphologyEx(mask_azul, cv2.MORPH_CLOSE, kernel)

        # 2. Encontrar contornos
        contornos, _ = cv2.findContours(mask_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > MIN_CORNER_AREA:
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    puntos_esquinas_detectados.append((cx, cy))
                    
                    cv2.circle(img_visualizacion, (cx, cy), 7, (255, 0, 0), -1) # Azul
                    cv2.putText(img_visualizacion, f"({cx},{cy})", (cx + 10, cy + 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1, cv2.LINE_AA)

        puertas_encontradas = self.tratarPuerta(puntos_esquinas_detectados, img_visualizacion)
        return puertas_encontradas, img_visualizacion
    
    

    def tratarPuerta(self, puntos_esquinas, img_visualizacion):
        puertas = []

        #RECIBIR DATOS DE ORB-SLAM
        pos_dron_mundo = self.pos_dron_mundo
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

            #CALCULAR CENTRO, PROPORCIÓN Y DISTANCIA DE LA PUERTA
            ancho_puerta = math.sqrt((esq2[0] - esq1[0])**2 + (esq2[1] - esq1[1])**2)  # lado inferior
            alto_puerta  = math.sqrt((esq1[0] - esq4[0])**2 + (esq1[1] - esq4[1])**2)  # lado izquierdo

            x_centro_puerta = int(round((esq1[0] + esq2[0] + esq3[0] + esq4[0]) / 4.0))
            y_centro_puerta = int(round((esq1[1] + esq2[1] + esq3[1] + esq4[1]) / 4.0))

            distancia_estimada = self.estimar_distancia(alto_puerta)

            fov_horizontal_rad = math.radians(FOV_H)
            fov_vertical_rad = math.radians(FOV_V)
            nx = (x_centro_puerta - ANCHO_IMAGEN / 2) / (ANCHO_IMAGEN / 2)
            ny = -(y_centro_puerta - ALTO_IMAGEN / 2) / (ALTO_IMAGEN / 2)

            angulo_horizontal_rad = nx * (fov_horizontal_rad / 2)
            angulo_vertical_rad = ny * (fov_vertical_rad / 2)

            coordenada_Z = -distancia_estimada * math.sin(angulo_vertical_rad)
            coordenada_Y = distancia_estimada * math.cos(angulo_vertical_rad) * math.sin(angulo_horizontal_rad)
            coordenada_X = distancia_estimada * math.cos(angulo_vertical_rad) * math.cos(angulo_horizontal_rad)
            
            distancia_calculada = math.sqrt(coordenada_X**2 + coordenada_Y**2 + coordenada_Z**2)

            # CONVERTIR PIXEL EN PUNTO 3D CON EJES CUERPO
            esq1_cuerpo = self.pixel_a_cuerpo(esq1[0], esq1[1], distancia_estimada)
            esq2_cuerpo = self.pixel_a_cuerpo(esq2[0], esq2[1], distancia_estimada)
            esq3_cuerpo = self.pixel_a_cuerpo(esq3[0], esq3[1], distancia_estimada)
            esq4_cuerpo = self.pixel_a_cuerpo(esq4[0], esq4[1], distancia_estimada)


            #CONVERTIR EL PUNTO DE EJES CUERPO(DRON) A EJES MUNDO (ORB-SLAM3)
            punto_cuerpo = [coordenada_X, coordenada_Y, coordenada_Z]
            punto_mundo = self.punto_cuerpo_a_mundo(pos_dron_mundo, punto_cuerpo)
            
            #CALCULAR ÁNGULO CON VECTOR NORMAL
            # Vector en cuerpo (apunta hacia el centro de la puerta)
            vector_cuerpo = np.array(punto_cuerpo)
            # Transformar a mundo sin traslación
            R_wb = quat2mat(self.quaternion)
            S = np.diag([1, -1, -1])
            vector_mundo = R_wb @ (S @ vector_cuerpo)
            # Ángulo en mundo respecto al eje X
            angulo_puerta = math.degrees(math.atan2(vector_mundo[1], vector_mundo[0]))

            #CALCULAR ORIENTACIÓN DE LA PUERTA CON VECTOR NORMAL
            v_h_cuerpo = esq2_cuerpo - esq1_cuerpo   # horizontal (inferior)
            v_v_cuerpo = esq4_cuerpo - esq1_cuerpo   # vertical (izquierda)

            normal_cuerpo = np.cross(v_h_cuerpo, v_v_cuerpo)
            norma = np.linalg.norm(normal_cuerpo)
            #Evitar divisón por 0
            if norma > 1e-6:
                normal_cuerpo = normal_cuerpo / norma
            else:
                self.get_logger().info(f"SE HA INTENTADO /0")
                v_h2 = esq3_cuerpo - esq2_cuerpo
                v_v2 = esq4_cuerpo - esq2_cuerpo
                normal_cuerpo = np.cross(v_h2, v_v2)
                norma = np.linalg.norm(normal_cuerpo)
                if norma > 1e-6:
                    normal_cuerpo = normal_cuerpo / norma
                else:
                    self.get_logger().warn("Normal degenerada; se omite orientación de la puerta.")
                    normal_cuerpo = np.array([1.0, 0.0, 0.0])  # fallback neutro


            # Transformar a mundo (FLU)
            R_wb = quat2mat(self.quaternion)
            S = np.diag([1, -1, -1])
            normal_mundo = R_wb @ (S @ normal_cuerpo)

            # Ángulo azimutal en XY mundo
            angulo_orientacion = math.degrees(math.atan2(normal_mundo[1], normal_mundo[0]))

            #IMPRIMIR DATOS
            self.distancia_estimada = distancia_estimada
            self.distancia_calculada = distancia_calculada
            self.coordenada_X = coordenada_X
            self.coordenada_Y = coordenada_Y
            self.coordenada_Z = coordenada_Z
            self.punto_mundo = punto_mundo
            self.normal_mundo = normal_mundo
            self.angulo_orientacion = angulo_orientacion

            #PUBLICAR DATOS DE LA PUERTA
            puerta_detectada = {
                'x_centro': x_centro_puerta,
                'y_centro': y_centro_puerta,
                'ancho': ancho_puerta,
                'alto': alto_puerta
            }
            puertas.append(puerta_detectada)
            self.publicar_punto(punto_mundo)

            #Dibujar el rectangulo
            esquinas = [esq1, esq2, esq3, esq4]
            puntos = np.array(esquinas, np.int32)
            puntos = puntos.reshape((-1, 1, 2))
            cv2.polylines(img_visualizacion, [puntos], True, (0, 255, 0), 2)
            cv2.circle(img_visualizacion, (x_centro_puerta, y_centro_puerta), 5, (0, 0, 255), -1) # Rojo (centro)
            cv2.putText(img_visualizacion, f"Puerta ({x_centro_puerta},{y_centro_puerta},{angulo_puerta:.1f})",
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
            self.get_logger().info(f"Normal de la puerta en mundo: {self.normal_mundo}")
            self.get_logger().info(f"Ángulo orientación de la puerta en mundo: {self.angulo_orientacion:.2f}°")
            self.get_logger().info("-----------------------------")

    def destroy_node(self):
        self.get_logger().info("Destruyendo ModuloLocalizacion...")
        super().destroy_node()

def main(args=None):
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

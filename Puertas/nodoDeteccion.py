import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import traceback
import math
from geometry_msgs.msg import PointStamped

ROS_TOPIC_IMAGEN_RAW_INPUT = 'tello/imagen'
ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT = '/tello/puertas_detectadas'
ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT = '/tello/imagen_puertas'
ROS_TOPIC_POSE_ANGLES = '/tello/pose_corregida'

ANCHO_IMAGEN = 960
ALTO_IMAGEN = 720

MIN_CORNER_AREA = 10

ALTO_REAL = 0.7
ANCHO_REAL = 0.5
FOCAL = 900.0 #tello
#FOCAL = 617.0 #webots

class NodoDeteccion(Node):
    def __init__(self):
        super().__init__('nodo_deteccion')
        self.get_logger().info("Iniciando nodo de detección de puertas.")
        self.bridge = CvBridge()
        
        self.camera_matrix = np.array([
            [FOCAL, 0, ANCHO_IMAGEN / 2],
            [0, FOCAL, ALTO_IMAGEN / 2],
            [0, 0, 1]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((1, 5))
        ancho_mitad = ANCHO_REAL / 2.0
        alto_mitad = ALTO_REAL / 2.0
        self.puntos_objeto_3d = np.array([
            [-ancho_mitad,  alto_mitad, 0.0],  # Esquina superior izquierda (esq1)
            [ ancho_mitad,  alto_mitad, 0.0],  # Esquina superior derecha (esq2)
            [ ancho_mitad, -alto_mitad, 0.0],  # Esquina inferior derecha (esq3)
            [-ancho_mitad, -alto_mitad, 0.0]   # Esquina inferior izquierda (esq4)
        ], dtype=np.float32)

        self.timer_log = self.create_timer(3.0, self.log_datos)
        self.distancia_estimada = None
        self.distancia_calculada = None
        self.coordenada_X = None
        self.coordenada_Y = None
        self.coordenada_Z = None
        self.punto_mundo = None
        self.pose_recibida = False

        self.pub_punto_a = self.create_publisher(Float32MultiArray, '/tello/punto_y_angulo', 10)

        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        self.suscripcion_imagen = self.create_subscription(Image, ROS_TOPIC_IMAGEN_RAW_INPUT, self.callback_procesamiento_imagen, qos_profile_sub)

        qos_profile_pose = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.suscripcion_pose = self.create_subscription(Float32MultiArray, ROS_TOPIC_POSE_ANGLES, self.callback_pose, qos_profile_pose)

        self.pos_dron_mundo = np.array([0.0, 0.0, 0.0])
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        qos_profile_pub_data = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.publicador_puertas_detectadas = self.create_publisher(Float32MultiArray, ROS_TOPIC_PUERTAS_DETECTADAS_OUTPUT, qos_profile_pub_data)

        qos_profile_pub_img = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.publicador_imagen_visualizacion = self.create_publisher(Image, ROS_TOPIC_IMAGEN_VISUALIZACION_OUTPUT, qos_profile_pub_img)

    def publicar_punto_a(self, x, y , z , angulo):
        msg = Float32MultiArray()
        msg.data = [x, y, z, angulo]
        self.pub_punto_a.publish(msg)

    def callback_pose(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) >= 6:
            self.pos_dron_mundo = np.array([data[0], data[1], data[2]])
            self.roll = float(data[3])
            self.pitch = float(data[4])
            self.yaw = float(data[5])

        if not self.pose_recibida:
                self.pose_recibida = True
                self.get_logger().info("Pose recibida. Detección activada.")

    def callback_procesamiento_imagen(self, msg_imagen_ros):
        if not self.pose_recibida:
            return
        
        try:
            frame_bgr_raw = self.bridge.imgmsg_to_cv2(msg_imagen_ros, desired_encoding="bgr8")
            img_procesamiento = cv2.resize(frame_bgr_raw, (ANCHO_IMAGEN, ALTO_IMAGEN))
            img_hsv = cv2.cvtColor(img_procesamiento, cv2.COLOR_BGR2HSV)
            puertas_detectadas, img_con_dibujos = self.algoritmoDetectarPuertasAruco(img_hsv, img_procesamiento.copy())

            ros_image_msg_out = self.bridge.cv2_to_imgmsg(img_con_dibujos, encoding="bgr8")
            self.publicador_imagen_visualizacion.publish(ros_image_msg_out)

        except Exception as e_processing_callback:
            self.get_logger().error(f"Error general en callback_procesamiento_imagen: {e_processing_callback}")
            self.get_logger().error(traceback.format_exc())

    def publicar_punto(self, punto_mundo): #este se visualiza en RViz
        msg = PointStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(punto_mundo[0])
        msg.point.y = float(punto_mundo[1])
        msg.point.z = float(punto_mundo[2])
        self.publisher_.publish(msg)

    def transformar_punto_cuerpo_a_mundo(self, punto_cuerpo):
        roll_rad, pitch_rad, yaw_rad = map(math.radians, [self.roll, self.pitch, self.yaw])
        cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)
        cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
        cos_y, sin_y = math.cos(yaw_rad), math.sin(yaw_rad)
        
        R_x = np.array([[1, 0, 0], [0, cos_r, -sin_r], [0, sin_r, cos_r]])
        R_y = np.array([[cos_p, 0, sin_p], [0, 1, 0], [-sin_p, 0, cos_p]])
        R_z = np.array([[cos_y, -sin_y, 0], [sin_y, cos_y, 0], [0, 0, 1]])
        
        R_cuerpo_a_mundo = R_z @ R_y @ R_x
        
        punto_mundo = self.pos_dron_mundo + R_cuerpo_a_mundo @ punto_cuerpo
        return punto_mundo, R_cuerpo_a_mundo

    def ordenar_puntos(self, pts):
        pts = np.array(pts, dtype="float32")
        rect = np.zeros((4, 2), dtype="float32")
        
        ordenados_por_y = pts[np.argsort(pts[:, 1])]
        
        puntos_superiores = ordenados_por_y[:2]
        puntos_inferiores = ordenados_por_y[2:]
        
        puntos_superiores = puntos_superiores[np.argsort(puntos_superiores[:, 0])]
        
        puntos_inferiores = puntos_inferiores[np.argsort(puntos_inferiores[:, 0])]
        
        rect[0] = puntos_superiores[0]   
        rect[1] = puntos_superiores[1]   
        rect[2] = puntos_inferiores[1]  
        rect[3] = puntos_inferiores[0]
        
        return rect

    def algoritmoDetectarPuertas(self, img_hsv, img_visualizacion):
        todas_puertas_encontradas = []
        kernel = np.ones((5,5), np.uint8)

        config_colores = [
            {
                "nombre": "rojo",
                "color": (0, 0, 255),
                "rangos": [
                    (np.array([0, 120, 150]), np.array([10, 255, 255])),
                    (np.array([170, 120, 150]), np.array([179, 255, 255]))
                ]
            },
            {
                "nombre": "azul",
                "color": (255, 0, 0),
                "rangos": [
                    (np.array([100, 190, 0]), np.array([140, 255, 255])) 
                ]
            }
        ]

        for config in config_colores:
            mask_color = None
            
            for (bajo, alto) in config["rangos"]:
                mask_parcial = cv2.inRange(img_hsv, bajo, alto)
                if mask_color is None:
                    mask_color = mask_parcial
                else:
                    mask_color = cv2.bitwise_or(mask_color, mask_parcial)

            mask_color = cv2.morphologyEx(mask_color, cv2.MORPH_OPEN, kernel)
            mask_color = cv2.morphologyEx(mask_color, cv2.MORPH_CLOSE, kernel)

            contornos, _ = cv2.findContours(mask_color, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            puntos_esquinas_detectados = []

            for cnt in contornos:
                area = cv2.contourArea(cnt)
                if area > MIN_CORNER_AREA:
                    M = cv2.moments(cnt)
                    if M['m00'] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        puntos_esquinas_detectados.append((cx, cy))
                        cv2.circle(img_visualizacion, (cx, cy), 7, config["color"], -1)

            puertas_este_color = self.tratarPuerta(puntos_esquinas_detectados, img_visualizacion)
            todas_puertas_encontradas.extend(puertas_este_color)

        return todas_puertas_encontradas, img_visualizacion
    
    def algoritmoDetectarPuertasAruco(self, img_hsv, img_visualizacion):
        todas_puertas_encontradas = []
        
        config_arucos = [
            {
                "nombre": "id7",
                "color": (0, 0, 255),
                "id": 7
            },
            {
                "nombre": "id3",
                "color": (255, 0, 0),
                "id": 3
            },
            {
                "nombre": "id8",
                "color": (255, 0, 0),
                "id": 8
            }
        ]

        img_gray = cv2.cvtColor(img_visualizacion, cv2.COLOR_BGR2GRAY)
        
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        aruco_params = cv2.aruco.DetectorParameters()

        corners, ids, rejected = cv2.aruco.detectMarkers(img_gray, aruco_dict, parameters=aruco_params)

        if ids is not None:
            ids = ids.flatten()
            
            for configuracion in config_arucos:
                id_buscado = configuracion["id"]
                puntos_esquinas_detectados = []
                
                for (marker_corners, marker_id) in zip(corners, ids):
                    if marker_id == id_buscado:
                        pts = marker_corners.reshape((4, 2))
                        
                        cx = int(np.mean(pts[:, 0]))
                        cy = int(np.mean(pts[:, 1]))
                        
                        puntos_esquinas_detectados.append((cx, cy))
                        
                        cv2.polylines(img_visualizacion, [pts.astype(int)], True, (0, 255, 0), 2)
                        cv2.circle(img_visualizacion, (cx, cy), 5, configuracion["color"], -1)
                
                if len(puntos_esquinas_detectados) > 0:
                    puertas_encontradas = self.tratarPuerta(puntos_esquinas_detectados, img_visualizacion)
                    todas_puertas_encontradas.extend(puertas_encontradas)

        return todas_puertas_encontradas, img_visualizacion

    def tratarPuerta(self, puntos_esquinas, img_visualizacion):
        puertas = []
        if len(puntos_esquinas) < 4:
            return puertas
        
        puntos_reales_ordenados = self.ordenar_puntos(puntos_esquinas)
        if puntos_reales_ordenados is not None:
            puntos_imagen_2d = np.array(puntos_reales_ordenados, dtype=np.float32)

            esq1 = puntos_imagen_2d[0]  # Superior izquierda (TL)
            esq2 = puntos_imagen_2d[1]  # Superior derecha (TR)
            esq3 = puntos_imagen_2d[2]  # Inferior derecha (BR)
            esq4 = puntos_imagen_2d[3]  # Inferior izquierda (BL)
            success, rvec, tvec = cv2.solvePnP(self.puntos_objeto_3d, puntos_imagen_2d, self.camera_matrix, self.dist_coeffs)
            
            if not success:
                return puertas
                        
            _, R_dron_a_mundo = self.transformar_punto_cuerpo_a_mundo(np.array([0,0,0]))
            R_puerta_a_camara, _ = cv2.Rodrigues(rvec)

            R_camara_a_dron = np.array([
                [ 0,  0,  1],  # Eje X dron (Adelante)  = Z camara (Adelante)
                [-1,  0,  0],  # Eje Y dron (Izquierda) = -X camara (- Derecha)
                [ 0, -1,  0]   # Eje Z dron (Arriba)    = -Y camara (- Abajo)
            ], dtype=np.float32)
            
            R_puerta_a_mundo = R_dron_a_mundo @ R_camara_a_dron @ R_puerta_a_camara
            
            vector_normal_mundo = R_puerta_a_mundo[:, 2]
            vector_normal_mundo = -vector_normal_mundo
            angulo_yaw = math.degrees(math.atan2(vector_normal_mundo[1], vector_normal_mundo[0]))

            largo_lado_izq = np.linalg.norm(esq1 - esq4)
            largo_lado_der = np.linalg.norm(esq2 - esq3)
            largo1 = max(largo_lado_izq, largo_lado_der)
            largo2 = min(largo_lado_izq, largo_lado_der)
            #angulo_yaw = 90 * (1 -(largo2/largo1))

            #self.get_logger().info(f"==== : {nuevo_angulo:.2f}")

            #if (largo_lado_izq < largo_lado_der):
            #    angulo_yaw = abs(angulo_yaw)
            #else:
            #    angulo_yaw = -angulo_yaw
                
            angulo_yaw = (angulo_yaw + 180) % 360 - 180
            #angulo_yaw = -angulo_yaw #revisar para que coincida con los nuevos ejes
            
            tvec_camara = tvec.reshape(3)
            punto_cuerpo_pnp = np.array([tvec_camara[2], -tvec_camara[0], -tvec_camara[1]])

            punto_mundo, _ = self.transformar_punto_cuerpo_a_mundo(punto_cuerpo_pnp)
            
            self.distancia_calculada = np.linalg.norm(punto_cuerpo_pnp)
            self.distancia_estimada = self.distancia_calculada 
            self.coordenada_X, self.coordenada_Y, self.coordenada_Z = punto_cuerpo_pnp
            self.punto_mundo = punto_mundo

            puerta_detectada = {
                'x_mundo': punto_mundo[0], 
                'y_mundo': punto_mundo[1], 
                'z_mundo': punto_mundo[2],
                'yaw_mundo': angulo_yaw,
                'distancia': self.distancia_calculada
            }
            puertas.append(puerta_detectada)

            self.publicar_punto_a(punto_mundo[0], punto_mundo[1], punto_mundo[2], angulo_yaw)
            x_centro_puerta = int(np.mean([p[0] for p in puntos_imagen_2d]))
            y_centro_puerta = int(np.mean([p[1] for p in puntos_imagen_2d]))
            esquinas = [esq1, esq2, esq3, esq4]
            cv2.polylines(img_visualizacion, [np.array(esquinas, np.int32)], True, (0, 255, 0), 2)
            cv2.circle(img_visualizacion, (x_centro_puerta, y_centro_puerta), 5, (0, 255, 0), -1)
            cv2.putText(img_visualizacion, f"Puerta ({punto_mundo[0]:.1f},{punto_mundo[1]:.1f},{angulo_yaw:.1f})",
                            (x_centro_puerta - 50, y_centro_puerta - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)

        return puertas

    def log_datos(self):
        if self.distancia_estimada is not None:
            self.get_logger().info("--- DATOS ---")
            self.get_logger().info(f"Distancia a la puerta: {self.distancia_estimada:.2f} m")
            self.get_logger().info(f"Verificación: La distancia calculada es {self.distancia_calculada:.3f} m")
            self.get_logger().info(f"Coordenada puerta (cuerpo): X={self.coordenada_X:.2f}, Y={self.coordenada_Y:.2f}, Z={self.coordenada_Z:.2f} m")
            if self.punto_mundo is not None:
                self.get_logger().info(f"Coordenadas del punto en mundo: ({self.punto_mundo[0]:.2f}, {self.punto_mundo[1]:.2f}, {self.punto_mundo[2]:.2f})")
            self.get_logger().info("-----------------------------")

    def destroy_node(self):
        self.get_logger().info("Fin NodoDeteccion")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo_deteccion = None
    try:
        nodo_deteccion = NodoDeteccion()
        rclpy.spin(nodo_deteccion)
    except KeyboardInterrupt:
        if nodo_deteccion:
            nodo_deteccion.get_logger().info("Ctrl+C")
    finally:
        if nodo_deteccion:
            nodo_deteccion.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
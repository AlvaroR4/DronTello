#!/usr/bin/env python3
"""
fotosCalibrar_fixed.py

Nodo ROS2 que:
 - se conecta al Tello (djitellopy)
 - publica frames en camera/image_raw (como tu ejemplo)
 - guarda imágenes para calibración (modo periodic u on_request)
 - maneja cierre limpio del stream/Tello

Parámetros (ROS2 params o editar en código):
 - save_mode: "periodic" | "on_request" | "off"
 - output_dir: carpeta para imágenes
 - save_interval: segundos entre guardados en periodic
 - max_images: máximo guardar
 - width,height: resolución guardada
 - request_topic: topic Bool para on_request
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from djitellopy import Tello
import cv2
import os
import time
from threading import Lock

TELLO_IP = '192.168.10.1'
ROS_TOPIC_OUTPUT = 'camera/image_raw'
TIMER_PERIOD = 1.0 / 30.0  # publicar ~30 FPS

class TelloImagePublisherSaver(Node):
    def __init__(self):
        super().__init__('tello_image_publisher_saver')

        # --- parámetros configurables ---
        self.declare_parameter("tello_ip", TELLO_IP)
        self.declare_parameter("topic", ROS_TOPIC_OUTPUT)
        self.declare_parameter("save_mode", "periodic")   # "periodic", "on_request", "off"
        self.declare_parameter("output_dir", "tello_calib_images")
        self.declare_parameter("save_interval", 2.0)      # segundos (periodic)
        self.declare_parameter("max_images", 60)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("request_topic", "camera/save_image")

        self.tello_ip = self.get_parameter("tello_ip").get_parameter_value().string_value
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.save_mode = self.get_parameter("save_mode").get_parameter_value().string_value
        self.output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self.save_interval = self.get_parameter("save_interval").get_parameter_value().double_value
        self.max_images = int(self.get_parameter("max_images").get_parameter_value().integer_value)
        self.width = int(self.get_parameter("width").get_parameter_value().integer_value)
        self.height = int(self.get_parameter("height").get_parameter_value().integer_value)
        self.request_topic = self.get_parameter("request_topic").get_parameter_value().string_value

        os.makedirs(self.output_dir, exist_ok=True)

        # estado interno
        self.bridge = CvBridge()
        self.tello = None
        self.frame_reader = None
        self.latest_frame = None   # BGR numpy image
        self.latest_ts = None
        self.lock = Lock()
        self.save_count = 0
        self.last_save_time = 0.0
        self.saving_enabled = (self.save_mode == "periodic")

        # QoS para publisher (igual al tuyo)
        qos_profile_publisher = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        # Conectar Tello
        self.get_logger().info(f"Intentando conectar al Tello en IP: {self.tello_ip}")
        try:
            self.tello = Tello(host=self.tello_ip)
            self.tello.connect()
            battery = self.tello.get_battery()
            self.get_logger().info(f"Conectado al Tello. Batería: {battery}%")
        except Exception as e:
            self.get_logger().error(f"Error conectando al Tello: {e}")
            raise

        # activar stream
        try:
            self.tello.streamoff()
            time.sleep(0.2)
            self.tello.streamon()
            time.sleep(0.5)
            self.frame_reader = self.tello.get_frame_read()
            time.sleep(0.5)
            self.get_logger().info("Stream activado y frame_reader obtenido.")
        except Exception as e:
            self.get_logger().error(f"Error inicializando stream: {e}")
            raise

        # publicador ROS
        self.image_publisher_ = self.create_publisher(Image, self.topic, qos_profile_publisher)
        self.get_logger().info(f"Publicando imágenes en: {self.topic}")

        # subscriber para requests (solo si on_request)
        if self.save_mode == "on_request":
            self.req_sub = self.create_subscription(Bool, self.request_topic, self.request_callback, qos_profile_publisher)
            self.get_logger().info(f"Modo on_request: escuchando {self.request_topic} para guardar imágenes.")

        # temporizador para publicar y gestionar guardado
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        self.get_logger().info(f"Temporizador creado con periodo: {TIMER_PERIOD:.3f}s. Modo guardado: {self.save_mode}")

    def timer_callback(self):
        # leer frame desde frame_reader (protegido por lock)
        frame = None
        try:
            if self.frame_reader is not None:
                frame = self.frame_reader.frame
        except Exception as e:
            self.get_logger().error(f"Error leyendo frame: {e}")
            frame = None

        if frame is None:
            self.get_logger().warn("No se ha recibido frame válido aún.", throttle_duration_sec=5.0)
            return

        ts = time.time()

        # publicar (convertir BGR -> RGB para publicar con encoding rgb8)
        try:
            img_pub_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            ros_image_msg = self.bridge.cv2_to_imgmsg(img_pub_rgb, encoding="rgb8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera"
            self.image_publisher_.publish(ros_image_msg)
        except Exception as e:
            self.get_logger().error(f"Error publicando frame en ROS: {e}")

        # actualizar latest_frame protegido
        with self.lock:
            # guardamos copia BGR para usar en saving
            self.latest_frame = frame.copy()
            self.latest_ts = ts

        # lógica de guardado en modo periodic
        if self.save_mode == "periodic" and self.save_count < self.max_images:
            if ts - self.last_save_time >= self.save_interval:
                self._save_current_frame()
                self.last_save_time = ts
                if self.save_count >= self.max_images:
                    self.get_logger().info(f"Alcanzado max_images ({self.max_images}). Parando guardado periódico.")

    def request_callback(self, msg: Bool):
        # modo on_request: si recibimos True guardamos una imagen
        if not msg.data:
            return
        if self.save_count >= self.max_images:
            self.get_logger().info(f"Alcanzado max_images ({self.max_images}). Ignorando request.")
            return
        self._save_current_frame()

    def _save_current_frame(self):
        with self.lock:
            frame = None if self.latest_frame is None else self.latest_frame.copy()
            ts = self.latest_ts
        if frame is None:
            self.get_logger().warn("Intento de guardado pero no hay frame disponible.", throttle_duration_sec=5.0)
            return
        # redimensionar si es necesario
        if (frame.shape[1], frame.shape[0]) != (self.width, self.height):
            frame = cv2.resize(frame, (self.width, self.height))
        # nombre archivo
        tstr = time.strftime("%Y%m%d_%H%M%S", time.localtime(ts)) if ts else time.strftime("%Y%m%d_%H%M%S")
        fname = os.path.join(self.output_dir, f"img_{self.save_count:04d}_{tstr}.png")
        try:
            cv2.imwrite(fname, frame)
            self.save_count += 1
            self.get_logger().info(f"Guardada imagen {self.save_count}/{self.max_images}: {fname}")
        except Exception as e:
            self.get_logger().error(f"Error guardando imagen: {e}")

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo: deteniendo timer y stream del Tello...")
        try:
            if self.timer:
                self.timer.cancel()
        except Exception:
            pass
        try:
            if self.tello is not None:
                self.tello.streamoff()
                time.sleep(0.2)
                self.tello.end()
        except Exception as e:
            self.get_logger().warn(f"Error al cerrar Tello: {e}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelloImagePublisherSaver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Ctrl-C detectado. Cerrando...")
    except Exception as e:
        print("Excepción en main:", e)
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()
        print("Nodo finalizado.")

if __name__ == "__main__":
    main()

import time
import subprocess
import threading
import sys

import numpy as np
import cv2
import tellopy
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


READ_CHUNK = 4096
TOPIC_CAMARA = 'tello/imagen'
TOPIC_POSE = '/tello/pose'
TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
FPS = 25

class Tello(Node):
    def __init__(self):
        super().__init__('tello')
        self.get_logger().info('Nodo iniciando...')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, TOPIC_CAMARA, 1)

        self.tello = tellopy.Tello()
        self.ffmpeg = None
        self._stop = False
        self._buffer = bytearray()

        self.tello.connect()
        time.sleep(0.2)
        self.tello.start_video()

        # Lanzar ffmpeg (entrada H264 por stdin, salida MJPEG por stdout)
        ff_cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error',
            '-fflags', 'nobuffer', '-flags', 'low_delay',
            '-f', 'h264', '-i', 'pipe:0',
            '-r', str(FPS),
            '-f', 'image2pipe', '-vcodec', 'mjpeg', 'pipe:1'
        ]
        self.ffmpeg = subprocess.Popen(
            ff_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0
        )

        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        #TOPIC PARA RECIBIR COMANDOS
        self.suscriptor_comandos_velocidad = self.create_subscription(Float32MultiArray,TOPIC_COMANDOS_VELOCIDAD, self.comandos_velocidad, qos_profile_cmd)
        #TOPICS PARA PUBLICAR
        self.publicador_pose = self.create_publisher(Float32MultiArray, TOPIC_POSE, qos_profile_pose)
        self.timer_pose = self.create_timer(0.1, self.publicar_pose)

        #SUSCRIPTORES PARA LEER DE LOS EVENTOS
        self.ultimo_flight = None #datos del vuelo
        self.ultimo_log = None #en este vienen mvo e imu
        # suscriptores
        self.tello.set_loglevel(self.tello.LOG_ERROR) #para menos ruido
        self.tello.subscribe(self.tello.EVENT_VIDEO_FRAME, self.evento_video)
        self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self.evento_flight)
        self.tello.subscribe(self.tello.EVENT_LOG_DATA, self.evento_log)
        
        # lector simple de stdout (ensambla JPEGs)
        self._reader = threading.Thread(target=self._read_stdout_loop, daemon=True)
        self._reader.start()
        self.get_logger().info('Inicio')

    def evento_flight(self, event, sender, data, **args):
        self.ultimo_flight = data

    def evento_log(self, event, sender, data, **args):
        self.ultimo_log = data
        
    def evento_video(self, event, sender, data):
        if self.ffmpeg and self.ffmpeg.stdin:
            self.ffmpeg.stdin.write(data)

    def _read_stdout_loop(self):
        out = self.ffmpeg.stdout
        buf = self._buffer
        while not self._stop:
            chunk = out.read(READ_CHUNK)
            if not chunk:
                time.sleep(0.005)
                continue
            buf.extend(chunk)
            # buscar JPEG completo
            while True:
                start = buf.find(b'\xff\xd8')
                if start < 0:
                    break
                end = buf.find(b'\xff\xd9', start + 2)
                if end < 0:
                    # esperar más datos
                    break
                jpg = bytes(buf[start:end+2])
                del buf[:end+2]
                arr = np.frombuffer(jpg, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is None:
                    # JPEG corrupto => ignorar
                    continue
                msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'tello_camera'
                self.image_pub.publish(msg)

    def comandos_velocidad(self, msg: Float32MultiArray):
        #recibir velocidad de cada eje
        lr = int(msg.data[0])
        fb = int(msg.data[1])
        ud = int(msg.data[2])
        yv = int(msg.data[3])

        #comandos especiales
        if lr == 2 and fb == 0 and ud == 0:
            self.get_logger().info("Comando: land")
            self.tello.land()
        elif lr == 2 and fb == 2 and ud == 0:
            self.get_logger().info("Comando: emergency")
            self.tello.emergency()
        elif lr == 2 and fb == 2 and ud == 2:
            self.get_logger().info("Comando: takeoff")
            self.tello.takeoff()
        
        #comandos velocidad
        if (lr == 0 and fb == 0 and ud == 0 and yv == 0):
            self.tello.right(0)
            self.tello.left(0)
            self.tello.forward(0)
            self.tello.backward(0)
            self.tello.up(0)
            self.tello.down(0)
            self.tello.clockwise(0)
            self.tello.counter_clockwise(0)

        if lr > 0:
            self.tello.right(lr)
        else:
            self.tello.left(abs(lr))
    
        if fb > 0:
            self.tello.forward(fb)
        else:
            self.tello.backward(abs(fb))
    
        if ud > 0:
            self.tello.up(ud)
        else:
            self.tello.down(abs(ud))
    
        if yv > 0:
            self.tello.clockwise(yv)
        else:
            self.tello.counter_clockwise(abs(yv))

    def publicar_pose(self):
        #[altura_altimetro, posx, posy, posz, roll_deg, pitch_deg, yaw_deg]
        msg = Float32MultiArray()
        altura_altimetro = 0.0
        posx = 0.0
        posy = 0.0
        posz = 0.0
        roll_deg = 0.0
        pitch_deg = 0.0
        yaw_deg = 0.0

        if self.ultimo_flight is not None:
            raw_alt = getattr(self.ultimo_flight, 'height', 0.0) or 0.0
            altura_altimetro = float(raw_alt)
            altura_altimetro = altura_altimetro*0.1

        if self.ultimo_log is not None:
            log = self.ultimo_log
            if hasattr(log, 'mvo') and log.mvo is not None:
                posx = float(log.mvo.pos_x)
                posy = float(log.mvo.pos_y)
                posz = float(log.mvo.pos_z)
            if hasattr(log, 'imu') and log.imu is not None:
                q0 = float(getattr(log.imu, 'q0', 1.0))
                q1 = float(getattr(log.imu, 'q1', 0.0))
                q2 = float(getattr(log.imu, 'q2', 0.0))
                q3 = float(getattr(log.imu, 'q3', 0.0))
                roll_rad, pitch_rad, yaw_rad = self._quaternion_to_euler(q0, q1, q2, q3)
                roll_deg = math.degrees(roll_rad)
                pitch_deg = math.degrees(pitch_rad)
                yaw_deg = math.degrees(yaw_rad)


        msg.data = [float(altura_altimetro), float(posx), float(posy), float(posz),
                    float(roll_deg), float(pitch_deg), float(yaw_deg)]
        self.publicador_pose.publish(msg)

    def _quaternion_to_euler(self, w, x, y, z):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def destroy_node(self):
        self.get_logger().info('FIN')
        self._stop = True
        try:
            if self.tello:
                self.tello.stop_video()
                self.tello.quit()
                self.tello.close()
        except Exception:
            pass
        try:
            if self.ffmpeg:
                if self.ffmpeg.stdin:
                    try:
                        self.ffmpeg.stdin.close()
                    except Exception:
                        pass
                try:
                    self.ffmpeg.kill()
                except Exception:
                    pass
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Tello()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Ctrl+C')
        node.tello.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
EJEMPLO PARA SEPARAR EL TRABAJO DE LOS NODOS ROS2 CON HILOS
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)
    node = Tello()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        ...
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()

"""

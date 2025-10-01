import time
import subprocess
import threading
import sys

import numpy as np
import cv2
import tellopy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

READ_CHUNK = 4096
TOPIC_CAMARA = 'camera/image_raw'
FPS = 25

class Tello(Node):
    def __init__(self):
        super().__init__('tello')
        self.get_logger().info('Nodo iniciando...')
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, TOPIC, 1)

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

        # suscribirse a eventos tellopy
        self.tello.subscribe(self.tello.EVENT_VIDEO_FRAME, self._on_video_frame)

        # lector simple de stdout (ensambla JPEGs)
        self._reader = threading.Thread(target=self._read_stdout_loop, daemon=True)
        self._reader.start()
        self.get_logger().info('Inicio')

    def _on_video_frame(self, event, sender, data):
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
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

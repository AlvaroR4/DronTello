#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Nodo ROS2 para DJI Tello usando la librería TelloPy (tellopy).
Publica:
 - camera/image_raw (sensor_msgs/Image)
 - /tello/pose (std_msgs/Float32MultiArray): [altura_altimetro, posx, posy, posz, roll_deg, pitch_deg, yaw_deg]

Se suscribe a:
 - /tello/comandos_velocidad (std_msgs/Float32MultiArray): [lr, fb, ud, yv]
   - casos especiales: (lr==2,fb==0,ud==0) -> land
                    (lr==2,fb==2,ud==0) -> emergency (land + quit)
                    (lr==2,fb==2,ud==2) -> takeoff

Incluye validación de log data corruptos, suavizado de pose y rampa de comandos para mayor estabilidad.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

import tellopy
import threading
import time
import math
import numpy as np
import cv2
import sys

ROS_TOPIC_IMAGEN_RAW = 'camera/image_raw'
ROS_TOPIC_COMANDOS_VELOCIDAD = '/tello/comandos_velocidad'
ROS_TOPIC_POSE = '/tello/pose'
TIMER_PERIOD_POSE = 0.1         # 10 Hz
TIMER_PERIOD_CAMARA = 1.0 / 30  # ~30 FPS

class NodoTelloPy(Node):
    def __init__(self):
        super().__init__('nodo_tellopy_tello')
        self.get_logger().info("Iniciando Nodo Tello (tellopy)")

        # --- variables internas en español (preferencia del usuario) ---
        self.bridge = CvBridge()
        self.tello = tellopy.Tello()
        self.ultimo_log = None
        self.ultimo_frame = None
        self.ultimo_flight = None

        # --- filtrado y suavizado ---
        self.pose_filtrada = {
            'altura': 0.0,
            'posx': 0.0,
            'posy': 0.0,
            'posz': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        self.pose_alpha = 0.3          # EMA alpha para pose (0..1) -> mayor alpha = menos suavizado
        self.cmd_last = [0.0, 0.0, 0.0, 0.0]   # último comando enviado [lr,fb,ud,yv] normalizado -1..1
        self.cmd_alpha = 0.25          # suavizado comandos (rampa)
        # límites razonables para detección de logs corruptos
        self.limites = {
            'pos_max_m': 20.0,       # posición máxima plausible en metros (Tello no llega a 20 m normalmente)
            'altura_max_cm': 1000.0, # 10 m en cm
            'angle_max_deg': 180.0
        }

        # Si quieres silenciar las líneas repetitivas 'LogData: corrupted data' pon esto a True:
        # (por defecto False: no altera sys.stdout)
        self.silenciar_logs = False
        self._orig_stdout_write = None
        self._orig_stderr_write = None

        # publicadores
        qos_profile_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.publicador_imagen = self.create_publisher(Image, ROS_TOPIC_IMAGEN_RAW, qos_profile_img)
        self.get_logger().info(f"Publicando imagen RAW en: {ROS_TOPIC_IMAGEN_RAW}")

        qos_profile_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publicador_pose = self.create_publisher(Float32MultiArray, ROS_TOPIC_POSE, qos_profile_pose)
        self.get_logger().info(f"Publicando pose en: {ROS_TOPIC_POSE}")

        # suscriptor de comandos de velocidad
        qos_profile_cmd = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.suscriptor_comandos_velocidad = self.create_subscription(
            Float32MultiArray,
            ROS_TOPIC_COMANDOS_VELOCIDAD,
            self.callback_comandos_velocidad,
            qos_profile_cmd
        )
        self.get_logger().info(f"Suscrito a comandos de velocidad en: {ROS_TOPIC_COMANDOS_VELOCIDAD}")

        # timers
        self.timer_pose = self.create_timer(TIMER_PERIOD_POSE, self._publicar_pose)
        self.timer_camara = self.create_timer(TIMER_PERIOD_CAMARA, self._publicar_frame)

        # conectar al tello y suscribirse a eventos (en hilo para no bloquear)
        self.get_logger().info("Conectando al Tello (tellopy)...")
        hilo_conexion = threading.Thread(target=self._iniciar_conexion, daemon=True)
        hilo_conexion.start()

    # ----------------- manejo de supresión de logs (opcional) -----------------
    def _aplicar_filtro_logs(self):
        """Aplica filtro temporal a stdout/stderr para suprimir líneas concretas."""
        if not self.silenciar_logs:
            return
        try:
            self._orig_stdout_write = sys.stdout.write
            self._orig_stderr_write = sys.stderr.write
        except Exception:
            self._orig_stdout_write = None
            self._orig_stderr_write = None

        def _filtro_write_stdout(s):
            try:
                if 'LogData: corrupted data' in s or 'video recv: timeout' in s:
                    return
            except Exception:
                pass
            return self._orig_stdout_write(s)

        def _filtro_write_stderr(s):
            try:
                if 'LogData: corrupted data' in s or 'video recv: timeout' in s:
                    return
            except Exception:
                pass
            return self._orig_stderr_write(s)

        try:
            sys.stdout.write = _filtro_write_stdout
            sys.stderr.write = _filtro_write_stderr
        except Exception:
            pass

    def _restaurar_logs(self):
        """Restaura stdout/stderr si fueron modificados."""
        try:
            if self._orig_stdout_write is not None:
                sys.stdout.write = self._orig_stdout_write
            if self._orig_stderr_write is not None:
                sys.stderr.write = self._orig_stderr_write
        except Exception:
            pass

    # ----------------- conexión e inicialización tellopy -----------------
    def _iniciar_conexion(self):
        try:
            # aplicar filtro temporal si se desea
            self._aplicar_filtro_logs()

            self.tello.set_loglevel(self.tello.LOG_ERROR)
            # Suscripciones a eventos
            try:
                self.tello.subscribe(self.tello.EVENT_FLIGHT_DATA, self._on_flight_data)
                self.tello.subscribe(self.tello.EVENT_LOG_DATA, self._on_log_data)
                self.tello.subscribe(self.tello.EVENT_VIDEO_FRAME, self._on_video_frame)
            except Exception:
                # si la versión de tellopy no soporta subscribe de la misma forma, seguir intentando
                pass

            self.tello.connect()
            self.tello.wait_for_connection(30.0)

            # iniciar streaming de vídeo
            try:
                self.tello.start_video()
            except Exception:
                try:
                    self.tello.send_command('streamon')
                except Exception:
                    pass

            # restaurar salida si la habíamos silenciado
            self._restaurar_logs()

            self.get_logger().info("Tello conectado y stream iniciado.")
        except Exception as ex:
            # restaurar por si acaso
            self._restaurar_logs()
            self.get_logger().info(f"Error conexión: {ex}")

    # ----------------- handlers de tellopy -----------------
    def _on_flight_data(self, event, sender, data, **args):
        self.ultimo_flight = data

    def _on_log_data(self, event, sender, data, **args):
        # data puede contener mvo, imu...
        # Simplemente guardamos; validación se hace en la publicación.
        self.ultimo_log = data

    def _on_video_frame(self, event, sender, data, **args):
        try:
            frame = data
            image_pil = frame.to_image()
            np_img = np.array(image_pil)
            frame_bgr = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
            self.ultimo_frame = frame_bgr
        except Exception:
            pass

    # ----------------- publicación de imagen -----------------
    def _publicar_frame(self):
        if self.ultimo_frame is None:
            return
        try:
            ros_image_msg = self.bridge.cv2_to_imgmsg(self.ultimo_frame, encoding="bgr8")
            ros_image_msg.header.stamp = self.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "tello_camera_link_raw"
            self.publicador_imagen.publish(ros_image_msg)
        except Exception:
            pass

    # ----------------- validación y extracción del log -----------------
    def _extraer_y_validar_log(self, log):
        """
        Intenta extraer posx,posy,posz,altura (flight.height) y quaternion desde log.
        Devuelve (ok, altura_cm, posx, posy, posz, q0,q1,q2,q3) donde ok es True si los datos son válidos.
        """
        try:
            if log is None:
                # Aun así, devolvemos False; la función que llama decidirá usar último filtrado
                return False, None, None, None, None, None, None, None, None

            altura = None
            # preferimos la altura que viene en ultimo_flight si existe; aquí devolvemos la de log únicamente
            # altura se leerá en _publicar_pose preferentemente desde self.ultimo_flight
            posx = posy = posz = None
            q0 = q1 = q2 = q3 = None

            if hasattr(log, 'mvo') and log.mvo is not None:
                try:
                    posx = float(getattr(log.mvo, 'pos_x', float('nan')))
                    posy = float(getattr(log.mvo, 'pos_y', float('nan')))
                    posz = float(getattr(log.mvo, 'pos_z', float('nan')))
                except Exception:
                    posx = posy = posz = None

            if hasattr(log, 'imu') and log.imu is not None:
                try:
                    q0 = float(getattr(log.imu, 'q0', 1.0))
                    q1 = float(getattr(log.imu, 'q1', 0.0))
                    q2 = float(getattr(log.imu, 'q2', 0.0))
                    q3 = float(getattr(log.imu, 'q3', 0.0))
                except Exception:
                    q0 = q1 = q2 = q3 = None

            # Validaciones básicas
            if None in (q0, q1, q2, q3):
                # no tenemos quaternion válido -> consideramos corrupto
                return False, None, None, None, None, None, None, None, None

            normq = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
            if not (math.isfinite(normq) and normq > 0.01):
                return False, None, None, None, None, None, None, None, None

            # Si hay posición, validarla
            if posx is not None and posy is not None and posz is not None:
                if not (all(math.isfinite(v) for v in (posx, posy, posz))):
                    return False, None, None, None, None, None, None, None, None
                if abs(posx) > self.limites['pos_max_m'] or abs(posy) > self.limites['pos_max_m'] or abs(posz) > self.limites['pos_max_m']:
                    return False, None, None, None, None, None, None, None, None

            return True, altura, posx, posy, posz, q0, q1, q2, q3
        except Exception:
            return False, None, None, None, None, None, None, None, None

    # ----------------- conversión quaternion -> euler (roll,pitch,yaw) -----------------
    def _quaternion_to_euler(self, w, x, y, z):
        # Fórmula estándar (asume q = [w, x, y, z])
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

    # ----------------- publicación de pose (con validación + suavizado) -----------------
    def _publicar_pose(self):
        msg = Float32MultiArray()

        # Si no tenemos log aún, publicamos el filtrado actual
        if self.ultimo_log is None and self.ultimo_flight is None:
            msg.data = [
                float(self.pose_filtrada['altura']),
                float(self.pose_filtrada['posx']),
                float(self.pose_filtrada['posy']),
                float(self.pose_filtrada['posz']),
                float(self.pose_filtrada['roll']),
                float(self.pose_filtrada['pitch']),
                float(self.pose_filtrada['yaw'])
            ]
            self.publicador_pose.publish(msg)
            return

        # Extraer y validar
        ok, altura_log, px, py, pz, q0, q1, q2, q3 = self._extraer_y_validar_log(self.ultimo_log)

        if ok:
            # altura preferentemente desde flight data si está disponible
            altura = None
            try:
                if self.ultimo_flight is not None:
                    altura = float(self.ultimo_flight.height)
            except Exception:
                altura = altura_log

            # posición
            posx = px if px is not None else None
            posy = py if py is not None else None
            posz = pz if pz is not None else None

            # quaternion -> euler
            roll_deg = pitch_deg = yaw_deg = None
            if q0 is not None:
                roll_rad, pitch_rad, yaw_rad = self._quaternion_to_euler(q0, q1, q2, q3)
                roll_deg = math.degrees(roll_rad)
                pitch_deg = math.degrees(pitch_rad)
                yaw_deg = math.degrees(yaw_rad)
        else:
            # paquete inválido: usamos valores actuales filtrados (no sobrescribimos con basura)
            altura = self.pose_filtrada['altura']
            posx = self.pose_filtrada['posx']
            posy = self.pose_filtrada['posy']
            posz = self.pose_filtrada['posz']
            roll_deg = self.pose_filtrada['roll']
            pitch_deg = self.pose_filtrada['pitch']
            yaw_deg = self.pose_filtrada['yaw']

        # fallback para None
        if altura is None:
            altura = self.pose_filtrada['altura']
        if posx is None:
            posx = self.pose_filtrada['posx']
        if posy is None:
            posy = self.pose_filtrada['posy']
        if posz is None:
            posz = self.pose_filtrada['posz']
        if roll_deg is None:
            roll_deg = self.pose_filtrada['roll']
        if pitch_deg is None:
            pitch_deg = self.pose_filtrada['pitch']
        if yaw_deg is None:
            yaw_deg = self.pose_filtrada['yaw']

        # aplicar EMA (low-pass)
        a = self.pose_alpha
        # altura en cm (float)
        try:
            self.pose_filtrada['altura'] = a * float(altura) + (1 - a) * self.pose_filtrada['altura']
        except Exception:
            pass
        try:
            self.pose_filtrada['posx'] = a * float(posx) + (1 - a) * self.pose_filtrada['posx']
            self.pose_filtrada['posy'] = a * float(posy) + (1 - a) * self.pose_filtrada['posy']
            self.pose_filtrada['posz'] = a * float(posz) + (1 - a) * self.pose_filtrada['posz']
        except Exception:
            pass

        # mezcla angular cuidando discontinuidades
        def _angle_mix(prev, new):
            # normalizamos new y prev a 0..360 primero si fuera necesario
            # calculamos diferencia en -180..180 y aplicamos alpha * diff
            diff = (new - prev + 180.0) % 360.0 - 180.0
            return prev + a * diff

        try:
            self.pose_filtrada['roll'] = _angle_mix(self.pose_filtrada['roll'], float(roll_deg))
            self.pose_filtrada['pitch'] = _angle_mix(self.pose_filtrada['pitch'], float(pitch_deg))
            self.pose_filtrada['yaw'] = _angle_mix(self.pose_filtrada['yaw'], float(yaw_deg))
        except Exception:
            pass

        msg.data = [
            float(self.pose_filtrada['altura']),
            float(self.pose_filtrada['posx']),
            float(self.pose_filtrada['posy']),
            float(self.pose_filtrada['posz']),
            float(self.pose_filtrada['roll']),
            float(self.pose_filtrada['pitch']),
            float(self.pose_filtrada['yaw'])
        ]
        self.publicador_pose.publish(msg)

    # ----------------- callback para recibir velocidades de control (con casos especiales y rampa) -----------------
    def callback_comandos_velocidad(self, msg: Float32MultiArray):
        # Intentamos leer cada componente; si falla, se queda en 0.0
        try:
            lr = float(msg.data[0])
        except Exception:
            lr = 0.0
        try:
            fb = float(msg.data[1])
        except Exception:
            fb = 0.0
        try:
            ud = float(msg.data[2])
        except Exception:
            ud = 0.0
        try:
            yv = float(msg.data[3])
        except Exception:
            yv = 0.0

        # Casos especiales
        if lr == 2.0 and fb == 0.0 and ud == 0.0:
            self.get_logger().info("Comando: land")
            try:
                self.tello.land()
            except Exception:
                pass
            return
        elif lr == 2.0 and fb == 2.0 and ud == 0.0:
            self.get_logger().info("Comando: emergency (kill)")
            try:
                self.tello.land()
                time.sleep(0.1)
                self.tello.quit()
            except Exception:
                pass
            return
        elif lr == 2.0 and fb == 2.0 and ud == 2.0:
            self.get_logger().info("Comando: takeoff")
            try:
                self.tello.takeoff()
            except Exception:
                pass
            return

        # Si no es caso especial, normalizamos y aplicamos rampa (suavizado)
        def _norm(v):
            try:
                nv = float(v) / 100.0
                if nv > 1.0:
                    nv = 1.0
                if nv < -1.0:
                    nv = -1.0
                return nv
            except Exception:
                return 0.0

        target = [_norm(lr), _norm(fb), _norm(ud), _norm(yv)]
        alpha = self.cmd_alpha
        smoothed = [alpha * t + (1 - alpha) * l for t, l in zip(target, self.cmd_last)]
        self.cmd_last = smoothed

        try:
            self.tello.set_roll(smoothed[0])
            self.tello.set_pitch(smoothed[1])
            self.tello.set_throttle(smoothed[2])
            self.tello.set_yaw(smoothed[3])
        except Exception:
            # fallback silencioso a comando 'rc' si no hay set_*
            try:
                cmd = f'rc {int(smoothed[0]*100)} {int(smoothed[1]*100)} {int(smoothed[2]*100)} {int(smoothed[3]*100)}'
                try:
                    self.tello.send_command(cmd)
                except Exception:
                    try:
                        self.tello.send_packet_data(cmd)
                    except Exception:
                        pass
            except Exception:
                pass

    # ----------------- limpieza y cierre -----------------
    def destroy_node(self):
        # cancelar timers
        try:
            if self.timer_camara:
                self.timer_camara.cancel()
        except Exception:
            pass
        try:
            if self.timer_pose:
                self.timer_pose.cancel()
        except Exception:
            pass

        # restablecer stdout/stderr por si quedó modificado
        self._restaurar_logs()

        # detener control y stream, aterrizar suavemente
        try:
            # detener mandos
            try:
                self.tello.set_roll(0)
                self.tello.set_pitch(0)
                self.tello.set_throttle(0)
                self.tello.set_yaw(0)
            except Exception:
                pass
            time.sleep(0.1)
            try:
                self.tello.land()
            except Exception:
                pass
            time.sleep(2.0)
            try:
                self.tello.send_command('streamoff')
            except Exception:
                pass
            try:
                self.tello.quit()
            except Exception:
                pass
        except Exception:
            pass

        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoTelloPy()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        try:
            nodo.tello.emergency()
        except Exception:
            pass
        nodo.get_logger().info("Emergencia.")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()
        print("Programa NodoTello (tellopy) finalizado.")

if __name__ == '__main__':
    main()

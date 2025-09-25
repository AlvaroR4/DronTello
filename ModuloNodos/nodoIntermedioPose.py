#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
nodoIntermiedioPose.py

Comportamiento simplificado:
- Al recibir la primera pose en /tello/posicion inicia un temporizador (post_takeoff_delay segundos).
- Cuando el temporizador termina toma la última pose recibida como offset (x,y,z,roll,pitch,yaw).
- A partir de ese momento publica en /tello/pose_angles las poses corregidas restando ese offset.
- Se puede resetear manualmente publicando std_msgs/Bool(True) en /tello/pose_angles/reset.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool
import threading
import traceback

# Tópicos
TOPIC_TELLO_POS_IN = '/tello/posicion'         # espera: [altura, x, y, z, (roll_deg, pitch_deg, yaw_deg)]
TOPIC_POSE_OUT = '/tello/pose_angles'          # publica: [x_corr, y_corr, z_corr, roll_deg, pitch_deg, yaw_deg]
TOPIC_RESET_OFFSET = '/tello/pose_angles/reset'  # Bool(True) para reset manual

# Parámetros por defecto
DEFAULT_POST_TAKEOFF_DELAY = 5.0  # segundos de espera desde la primera lectura

class NodoIntermedioPose(Node):
    def __init__(self):
        super().__init__('nodo_intermedio_pose_simple')
        self.get_logger().info('Iniciando nodoIntermiedioPose (modo simple, espera fija).')

        # Parámetro: tiempo de espera en segundos desde la primera pose recibida
        self.declare_parameter('post_takeoff_delay', DEFAULT_POST_TAKEOFF_DELAY)
        self.post_takeoff_delay = float(self.get_parameter('post_takeoff_delay').value)

        # Estado de offsets (serán fijados tras el temporizador)
        self.offset_pos = [0.0, 0.0, 0.0]      # x_offset, y_offset, z_offset
        self.offset_angles = [0.0, 0.0, 0.0]   # roll_offset, pitch_offset, yaw_offset
        self.origin_fijado = False             # True cuando ya se fijó el 0,0,0

        # Estado de temporizador / primera lectura
        self.first_pose_received = False
        self.wait_timer = None                 # objeto threading.Timer
        self.last_pose = None                  # tuplas: (altura, x, y, z, roll, pitch, yaw)

        qos_sub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        # Suscripción a la pose cruda
        self.sub_pos = self.create_subscription(
            Float32MultiArray,
            TOPIC_TELLO_POS_IN,
            self.callback_posicion,
            qos_sub
        )
        # Publicador de pose corregida (sin cuaterniones)
        self.pub_pose = self.create_publisher(Float32MultiArray, TOPIC_POSE_OUT, qos_pub)

        # Suscripción para reset manual
        try:
            self.sub_reset = self.create_subscription(
                Bool,
                TOPIC_RESET_OFFSET,
                self.callback_reset_offset,
                qos_sub
            )
        except Exception:
            self.sub_reset = None

        self.get_logger().info(f"Escuchando {TOPIC_TELLO_POS_IN} y publicando {TOPIC_POSE_OUT}")
        self.get_logger().info(f"Tiempo de espera para fijar origen: {self.post_takeoff_delay} s")

    def callback_reset_offset(self, msg: Bool):
        if msg.data:
            self._cancel_timer_if_any()
            self.offset_pos = [0.0, 0.0, 0.0]
            self.offset_angles = [0.0, 0.0, 0.0]
            self.origin_fijado = False
            self.first_pose_received = False
            self.last_pose = None
            self.get_logger().info("Offset reseteado manualmente (/tello/pose_angles/reset).")

    def _cancel_timer_if_any(self):
        try:
            if self.wait_timer is not None:
                try:
                    self.wait_timer.cancel()
                except Exception:
                    pass
                self.wait_timer = None
        except Exception:
            pass

    def _timer_callback_set_origin(self):
        """
        Callback ejecutado al expirar el temporizador.
        Fija el offset tomando la última pose registrada en self.last_pose.
        """
        try:
            if self.last_pose is None:
                self.get_logger().warning("Temporizador expiró pero no hay pose registrada. No se fija origen.")
                return

            # last_pose puede contener [altura, x, y, z, roll, pitch, yaw] o menos
            data = list(self.last_pose)
            # Extraer x,y,z (si la entrada vino con altura en índice 0 -> alt, x, y, z)
            # Tenemos que ser flexibles según lo que publique nodoTelloPos: en nuestro diseño la entrada
            # esperada es [altura, x, y, z, roll, pitch, yaw]
            # Si la estructura es distinta, intentamos manejarlo lo mejor posible.
            if len(data) >= 4:
                # data[0]=altura, data[1]=x, data[2]=y, data[3]=z
                x = float(data[1])
                y = float(data[2])
                z = float(data[3])
            else:
                # estructura inesperada -> no fijamos origen
                self.get_logger().warning("Estructura de primera pose inesperada, no se pudo fijar origen.")
                return

            roll = 0.0
            pitch = 0.0
            yaw = 0.0
            if len(data) >= 7:
                roll = float(data[4])
                pitch = float(data[5])
                yaw = float(data[6])
            elif len(data) >= 5:
                # si sólo hay un extra lo interpretamos como yaw (compatibilidad)
                yaw = float(data[4])

            # Fijar offset: tomamos la lectura actual como origen
            self.offset_pos = [x, y, z]
            self.offset_angles = [roll, pitch, yaw]
            self.origin_fijado = True
            self.get_logger().info(f"Origen fijado (0,0,0). Offset guardado: pos={self.offset_pos}, angles={self.offset_angles}")

        except Exception as e:
            self.get_logger().error(f"Error en timer callback al fijar origen: {e}")
            self.get_logger().error(traceback.format_exc())
        finally:
            # aseguramos limpiar referencia al timer
            self.wait_timer = None

    def callback_posicion(self, msg: Float32MultiArray):
        """
        Recibe msg.data:
          esperado: [altura, x, y, z, (roll_deg, pitch_deg, yaw_deg)]
        - En la primera recepción arranca el temporizador para fijar origen.
        - Siempre actualiza self.last_pose con la última pose recibida.
        - Publica en /tello/pose_angles: si origin_fijado True -> pose corregida (resta offset),
                                           si False -> publica la pose sin corregir.
        """
        try:
            data = list(msg.data)
            if len(data) < 4:
                self.get_logger().warning("msg /tello/posicion con datos insuficientes (<4). Ignorando.")
                return

            # Actualizamos la última pose recibida
            self.last_pose = data

            # Si es la primera pose que recibimos y no hay temporizador activo, iniciamos la espera
            if (not self.first_pose_received) and (not self.origin_fijado):
                self.first_pose_received = True
                # arrancar temporizador no bloqueante
                self.get_logger().info(f"Primera pose recibida: iniciando espera de {self.post_takeoff_delay}s antes de fijar origen.")
                # cancelar cualquier timer previo por seguridad
                self._cancel_timer_if_any()
                self.wait_timer = threading.Timer(self.post_takeoff_delay, self._timer_callback_set_origin)
                self.wait_timer.daemon = True
                self.wait_timer.start()

            # Interpretar pose (flexible)
            # Asumimos formato: [altura, x, y, z, roll, pitch, yaw] si está disponible
            altura = float(data[0])
            x = float(data[1])
            y = float(data[2])
            z = float(data[3])

            roll = 0.0
            pitch = 0.0
            yaw = 0.0
            if len(data) >= 7:
                roll = float(data[4])
                pitch = float(data[5])
                yaw = float(data[6])
            elif len(data) >= 5:
                yaw = float(data[4])

            # Si el origen ya está fijado aplicamos corrección (restamos offsets)
            if self.origin_fijado:
                x_corr = x - self.offset_pos[0]
                y_corr = y - self.offset_pos[1]
                z_corr = z - self.offset_pos[2]
                roll_corr = roll - self.offset_angles[0]
                pitch_corr = pitch - self.offset_angles[1]
                yaw_corr = yaw - self.offset_angles[2]
            else:
                # Antes de fijar origen publicamos sin corregir (para depuración/compatibilidad)
                x_corr, y_corr, z_corr = x, y, z
                roll_corr, pitch_corr, yaw_corr = roll, pitch, yaw

            out = Float32MultiArray()
            out.data = [float(x_corr), float(y_corr), float(z_corr),
                        float(roll_corr), float(pitch_corr), float(yaw_corr)]
            self.pub_pose.publish(out)

        except Exception as e:
            self.get_logger().error(f"Error en callback_posicion: {e}")
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        # cancelar temporizador si queda activo
        try:
            self._cancel_timer_if_any()
        except Exception:
            pass
        self.get_logger().info("Destruyendo nodoIntermiedioPose (modo simple).")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    nodo = None
    try:
        nodo = NodoIntermedioPose()
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if nodo:
            nodo.get_logger().fatal(f"Error inesperado: {e}")
            nodo.get_logger().fatal(traceback.format_exc())
    finally:
        if nodo:
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

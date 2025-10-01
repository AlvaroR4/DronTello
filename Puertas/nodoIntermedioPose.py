import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool
import threading
import traceback

# Tópicos
TOPIC_TELLO_POS_IN = '/tello/posicion'         # [altura, x, y, z, (roll_deg, pitch_deg, yaw_deg)]
TOPIC_POSE_OUT = '/tello/pose_corregida'          # publica: [x_corr, y_corr, z_corr, roll_deg, pitch_deg, yaw_deg]
TOPIC_RESET_OFFSET = '/tello/pose_angles/reset'  

ESPERA_TAKEOFF = 5.0  # segundos de espera desde la primera lectura

class NodoIntermedioPose(Node):
    def __init__(self):
        super().__init__('nodo_intermedio_pose_simple')
        self.get_logger().info('Iniciando nodoIntermiedioPose (modo simple, espera fija).')

        self.declare_parameter('post_takeoff_delay', ESPERA_TAKEOFF)
        self.post_takeoff_delay = float(self.get_parameter('post_takeoff_delay').value)

        self.offset_pos = [0.0, 0.0, 0.0]
        self.offset_angles = [0.0, 0.0, 0.0]
        self.origin_fijado = False

        self.first_pose_received = False
        self.wait_timer = None
        self.last_pose = None

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
        try:
            if self.last_pose is None:
                self.get_logger().warning("Temporizador expiró pero no hay pose registrada. No se fija origen.")
                return

            data = list(self.last_pose)

            if len(data) >= 4:
                # data[0]=altura, data[1]=x, data[2]=y, data[3]=z
                altura = float(data[0])
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
                yaw = float(data[4])

            self.offset_pos = [x, y, altura]
            self.offset_angles = [roll, pitch, yaw]
            self.origin_fijado = True
            self.get_logger().info(f"Origen fijado (0,0,0). Offset guardado: pos={self.offset_pos}, angles={self.offset_angles}")

        except Exception as e:
            self.get_logger().error(f"Error en timer callback al fijar origen: {e}")
            self.get_logger().error(traceback.format_exc())
        finally:
            self.wait_timer = None

    def callback_posicion(self, msg: Float32MultiArray):
        try:
            data = list(msg.data)
            if len(data) < 4:
                self.get_logger().warning("msg /tello/posicion con datos insuficientes (<4). Ignorando.")
                return

            self.last_pose = data

            if (not self.first_pose_received) and (not self.origin_fijado):
                self.first_pose_received = True
                # arrancar temporizador 
                self.get_logger().info(f"Primera pose recibida: iniciando espera de {self.post_takeoff_delay}s antes de fijar origen.")
                self._cancel_timer_if_any()
                self.wait_timer = threading.Timer(self.post_takeoff_delay, self._timer_callback_set_origin)
                self.wait_timer.daemon = True
                self.wait_timer.start()

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

            if self.origin_fijado:
                x_corr = x - self.offset_pos[0]
                y_corr = y - self.offset_pos[1]
                z_corr = z - self.offset_pos[2]
                roll_corr = roll - self.offset_angles[0]
                pitch_corr = pitch - self.offset_angles[1]
                yaw_corr = yaw - self.offset_angles[2]
            else:
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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool
import threading
import traceback

TOPIC_TELLO_POS_IN = '/tello/pose'        
TOPIC_POSE_OUT = '/tello/pose_corregida'
TOPIC_RESET_OFFSET = '/tello/pose_angles/reset'  
ESPERA_TAKEOFF = 11.0  

class NodoIntermedioPose(Node):
    def __init__(self):
        super().__init__('nodo_intermedio_pose')
        self.get_logger().info('--- NODO INTERMEDIO POSE --')

        self.post_takeoff_delay = ESPERA_TAKEOFF

        self.offset_pos = [0.0, 0.0, 0.0]    
        self.offset_angles = [0.0, 0.0, 0.0] 
        
        self.origin_fijado = False
        self.primera_lectura = True
        self.first_pose_received = False
        self.wait_timer = None
        self.last_pose = None
        
        self.x_previo = 0.0
        self.y_previo = 0.0
        self.z_previo = 0.0

        qos_reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.sub_pos = self.create_subscription(Float32MultiArray, TOPIC_TELLO_POS_IN, self.callback_posicion, qos_reliable)
        self.pub_pose = self.create_publisher(Float32MultiArray, TOPIC_POSE_OUT, qos_reliable)
        
        try:
            self.sub_reset = self.create_subscription(Bool, TOPIC_RESET_OFFSET, self.callback_reset_offset, qos_reliable)
        except Exception:
            pass

    def callback_reset_offset(self, msg: Bool):
        if msg.data:
            self._cancel_timer_if_any()
            self.origin_fijado = False
            self.first_pose_received = False
            self.offset_pos = [0.0, 0.0, 0.0]
            self.get_logger().info("RESET MANUAL: Offset borrado.")

    def _cancel_timer_if_any(self):
        try:
            if self.wait_timer is not None:
                self.wait_timer.cancel()
                self.wait_timer = None
        except Exception:
            pass

    def _timer_callback_set_origin(self):
        try:
            if self.last_pose is None: return

            data = list(self.last_pose)
            if len(data) < 4: return

            
            z_altimetro_real = float(data[0]) 
            z_mvo_actual = -float(data[3]) 
            
            x_actual = float(data[1])
            y_actual = float(data[2])
            
            offset_z = z_mvo_actual - z_altimetro_real

            self.offset_pos = [x_actual, y_actual, offset_z] 
            
            roll = 0.0; pitch = 0.0; yaw = 0.0
            if len(data) >= 7:
                roll = float(data[4]); pitch = float(data[5]); yaw = float(data[6])
            elif len(data) >= 5:
                yaw = float(data[4])
            
            self.offset_angles = [roll, pitch, yaw]
            self.origin_fijado = True
            
            self.get_logger().info(f"--- ORIGEN FIJADO ---")

        except Exception as e:
            self.get_logger().error(f"Error timer: {e}")
        finally:
            self.wait_timer = None

    def callback_posicion(self, msg: Float32MultiArray):
        try:
            data = list(msg.data)
            if len(data) < 4: return
            self.last_pose = data

            if (not self.first_pose_received) and (not self.origin_fijado):
                self.first_pose_received = True
                self._cancel_timer_if_any()
                self.wait_timer = threading.Timer(self.post_takeoff_delay, self._timer_callback_set_origin)
                self.wait_timer.daemon = True
                self.wait_timer.start()
            
            if not self.origin_fijado:
                return
            
            z_raw = -float(data[3]) 
            x_raw = float(data[1])
            y_raw = float(data[2])

            roll = 0.0; pitch = 0.0; yaw = 0.0
            if len(data) >= 7:
                roll = float(data[4]); pitch = float(data[5]); yaw = float(data[6])
            elif len(data) >= 5:
                yaw = float(data[4])

            x_corr = x_raw - self.offset_pos[0]
            y_corr = y_raw - self.offset_pos[1]
            z_corr = z_raw - self.offset_pos[2] 
            
            yaw_corr = yaw - self.offset_angles[2]
            roll_corr = roll - self.offset_angles[0]
            pitch_corr = pitch - self.offset_angles[1]

            out = Float32MultiArray()
            out.data = [
                float(x_corr), 
                float(-y_corr), 
                float(z_corr),   
                float(roll_corr), 
                float(pitch_corr), 
                float(-yaw_corr) 
            ]
            self.pub_pose.publish(out)

            self.x_previo = x_corr
            self.y_previo = -y_corr
            self.z_previo = z_corr

        except Exception as e:
            self.get_logger().error(f"Error callback: {e}")

    def destroy_node(self):
        self._cancel_timer_if_any()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoIntermedioPose()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
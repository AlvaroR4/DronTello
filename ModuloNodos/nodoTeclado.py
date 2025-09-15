import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import time
import sys
import select
import termios
import tty

#NO PONER VELOCIDAD = 2
VELOCIDAD_AVANCE = 10.0
VELOCIDAD_LATERAL = 10.0 
VELOCIDAD_VERTICAL = 10.0
VELOCIDAD_YAW = 10.0     
FREQ_PUBLISH = 10.0       # Hz
TOPIC = '/tello/comandos_velocidad'

class NodoTeclado(Node):
    def __init__(self):
        super().__init__('nodo_control_teclado')
        self.get_logger().info("nodoTeclado iniciando...")
        self.pub = self.create_publisher(Float32MultiArray, TOPIC, 10)

        # estado actual [lr, fb, ud, yv]
        self._lock = threading.Lock()
        self.comando_actual = [0.0, 0.0, 0.0, 0.0]

        periodo = 1.0 / FREQ_PUBLISH
        self.timer = self.create_timer(periodo, self._publicar_comando)
        self._running = True

        self._kb_thread = None
        self._kb_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._kb_thread.start()

        self.avance = 0.0
        self.vertical = 0.0
        self.lateral = 0.0
        self.yaw = 0.0

        self.get_logger().info("TECLAS : w/a/s/d/q/e ; espacio -> PARAR")
        self.get_logger().info(" y -> SUBIR ; h -> BAJAR")
        self.get_logger().info("t -> TAKEOFF; l -> LAND; o -> EMERGENCY")


    def _publicar_comando(self):
        with self._lock:
            lr, fb, ud, yv = self.comando_actual
        msg = Float32MultiArray()
        msg.data = [float(lr), float(fb), float(ud), float(yv)]
        self.pub.publish(msg)

    def _stdin_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and self._running:
                dr, _, _ = select.select([sys.stdin], [], [], 0.1)
                if dr:
                    ch = sys.stdin.read(1)
                    if not ch:
                        continue
                    if ch == '\x03':
                        self.get_logger().info("Ctrl-C detectado en stdin, cerrando...")
                        rclpy.shutdown()
                        break
                    self._procesar_tecla(ch)
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    def _procesar_tecla(self, ch):
        ch = ch.lower()
        nuevo = [0.0, 0.0, 0.0, 0.0]
        if ch == 'w':
            self.avance += VELOCIDAD_AVANCE
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]    
            nombre = "AVANZAR (w)"
        elif ch == 's':
            self.avance -= VELOCIDAD_AVANCE
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]
            nombre = "ATRÁS (s)"
        elif ch == 'a':
            self.lateral -= VELOCIDAD_LATERAL
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]
            nombre = "IZQUIERDA (a)"
        elif ch == 'd':
            self.lateral += VELOCIDAD_LATERAL
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]
            nombre = "DERECHA (d)"
        elif ch == 'q':
            self.yaw -=VELOCIDAD_YAW
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]
            nombre = "GIRAR IZQUIERDA (q)"
        elif ch == 'e':
            self.yaw += VELOCIDAD_YAW
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]
            nombre = "GIRAR DERECHA (e)"
        elif ch == 'y':
            self.vertical += VELOCIDAD_VERTICAL
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]
            nombre = "SUBIR (y)"
        elif ch == 'h':
            self.vertical -= VELOCIDAD_VERTICAL
            nuevo = [self.lateral, self.avance, self.yaw, self.vertical]
            nombre = "BAJAR (h)"
        elif ch == ' ':
            self.avance = 0.0
            self.vertical = 0.0
            self.lateral = 0.0
            self.yaw = 0.0
            nuevo = [self.vertical, self.avance, self.yaw, self.vertical]
            nombre = "PARAR (space)"
        elif ch == 'l':
            nuevo = [2.0, 0.0, 0.0, 0.0] #se interpreta como land
            nombre = "LAND (l)" 
        elif ch == 'o':
            nuevo = [2.0, 2.0, 0.0, 0.0] #se interpreta como emergency
            nombre = "EMERGENCY (o)" 
        elif ch == 't':
            nuevo = [2.0, 2.0, 2.0, 0.0] #se interpreta como takeoff
            nombre = "TAKEOFF (t)" 
        else:
            return

        with self._lock:
            if any(abs(a - b) > 1e-6 for a, b in zip(self.comando_actual, nuevo)):
                self.get_logger().info(f"Comando activo: {nombre} .VELOCIDAD = {self.lateral}, {self.avance}, {self.yaw}, {self.vertical}")
            self.comando_actual = nuevo

    def destroy_node(self):
        self._running = False
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)
        time.sleep(0.05)

        super().destroy_node()
        if self._kb_thread and self._kb_thread.is_alive():
            self._kb_thread.join(timeout=0.2)


def main(args=None):
    rclpy.init(args=args)
    nodo = NodoTeclado()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Ctrl+C")
    finally:
        try:
            if rclpy.ok():
                nodo.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()

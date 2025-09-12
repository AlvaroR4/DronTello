import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import threading
import time
import sys
import select
import termios
import tty


VELOCIDAD_AVANCE = 10.0
VELOCIDAD_LATERAL = 15.0 
VELOCIDAD_VERTICAL = 0.0 #sin implementar
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

        self.get_logger().info("TECLAS : w/a/s/d/q/e ; ESPACIO -> parar")

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
            nuevo = [0.0, VELOCIDAD_AVANCE, 0.0, 0.0]
            label = "AVANZAR (w)"
        elif ch == 's':
            nuevo = [0.0, -VELOCIDAD_AVANCE, 0.0, 0.0]
            label = "ATRÁS (s)"
        elif ch == 'a':
            nuevo = [-VELOCIDAD_LATERAL, 0.0, 0.0, 0.0]
            label = "IZQUIERDA (a)"
        elif ch == 'd':
            nuevo = [VELOCIDAD_LATERAL, 0.0, 0.0, 0.0]
            label = "DERECHA (d)"
        elif ch == 'q':
            nuevo = [0.0, 0.0, 0.0, -VELOCIDAD_YAW]
            label = "GIRAR IZQUIERDA (q)"
        elif ch == 'e':
            nuevo = [0.0, 0.0, 0.0, VELOCIDAD_YAW]
            label = "GIRAR DERECHA (e)"
        elif ch == ' ':
            nuevo = [0.0, 0.0, 0.0, 0.0]
            label = "PARAR (space)"
        else:
            return

        with self._lock:
            if any(abs(a - b) > 1e-6 for a, b in zip(self.comando_actual, nuevo)):
                self.get_logger().info(f"Comando activo: {label}")
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

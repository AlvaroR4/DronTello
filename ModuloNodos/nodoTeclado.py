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
        nuevo = self.comando_actual[:] # Empezamos con el comando actual
        nombre = ""

        # Comandos que establecen una velocidad continua
        if ch == 'w':
            self.avance += VELOCIDAD_AVANCE
            nombre = "AVANZAR (w)"
        elif ch == 's':
            self.avance -= VELOCIDAD_AVANCE
            nombre = "ATRÁS (s)"
        elif ch == 'a':
            self.lateral -= VELOCIDAD_LATERAL
            nombre = "IZQUIERDA (a)"
        elif ch == 'd':
            self.lateral += VELOCIDAD_LATERAL
            nombre = "DERECHA (d)"
        elif ch == 'q':
            self.yaw -= VELOCIDAD_YAW
            nombre = "GIRAR IZQUIERDA (q)"
        elif ch == 'e':
            self.yaw += VELOCIDAD_YAW
            nombre = "GIRAR DERECHA (e)"
        elif ch == 'y':
            self.vertical += VELOCIDAD_VERTICAL
            nombre = "SUBIR (y)"
        elif ch == 'h':
            self.vertical -= VELOCIDAD_VERTICAL
            nombre = "BAJAR (h)"
        elif ch == ' ':
            self.avance = 0.0
            self.vertical = 0.0
            self.lateral = 0.0
            self.yaw = 0.0
            nombre = "PARAR (space)"
        
        # Comandos especiales que se publican una vez y luego paran
        elif ch in ['l', 'o', 't']:
            comando_especial = [0.0, 0.0, 0.0, 0.0]
            if ch == 'l':
                comando_especial = [2.0, 0.0, 0.0, 0.0] # se interpreta como land
                nombre = "LAND (l)"
            elif ch == 'o':
                comando_especial = [2.0, 2.0, 0.0, 0.0] # se interpreta como emergency
                nombre = "EMERGENCY (o)"
            elif ch == 't':
                comando_especial = [2.0, 2.0, 2.0, 0.0] # se interpreta como takeoff
                nombre = "TAKEOFF (t)"

            # Publica el comando inmediatamente
            msg = Float32MultiArray()
            msg.data = [float(val) for val in comando_especial]
            self.pub.publish(msg)
            self.get_logger().info(f"Comando enviado: {nombre}")

            self.avance = 0.0
            self.vertical = 0.0
            self.lateral = 0.0
            self.yaw = 0.0
            
            nuevo = [0.0, 0.0, 0.0, 0.0]
            with self._lock:
                self.comando_actual = nuevo
            return

        else:
            return # Tecla no reconocida

        nuevo = [self.lateral, self.avance, self.vertical, self.yaw]

        with self._lock:
            if any(abs(a - b) > 1e-6 for a, b in zip(self.comando_actual, nuevo)):
                self.get_logger().info(f"Comando activo: {nombre} .VELOCIDAD = {[self.lateral, self.avance, self.vertical, self.yaw]}")
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
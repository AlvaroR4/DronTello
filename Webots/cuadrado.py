#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MavicCuadrado(Node):
    def __init__(self):
        super().__init__('mavic_cuadrado')
        
        # Publicador de velocidad
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Temporizador de control (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Variables de estado
        self.start_time = time.time()
        self.estado = 'DESPEGUE'
        self.lado_actual = 0
        self.total_lados = 4
        
        # --- CONFIGURACIÓN ---
        self.TIEMPO_DESPEGUE = 4.0   # Segundos subiendo
        self.TIEMPO_AVANCE = 5.0     # Segundos avanzando (para hacer ~5 metros)
        self.TIEMPO_GIRO = 2.0       # Segundos girando (ajustar para 90 grados)
        self.TIEMPO_ESPERA = 1.0     # Pausa entre movimientos
        
        # Velocidades
        self.VEL_SUBIDA = 1.0
        self.VEL_AVANCE = 1.0        # 1 m/s * 5 seg = 5 metros
        self.VEL_GIRO = 0.785        # Radianes/seg (aprox PI/4 para girar 90º en 2s)
        
        self.get_logger().info('Iniciando misión cuadrada...')

    def timer_callback(self):
        msg = Twist()
        tiempo_actual = time.time()
        tiempo_transcurrido = tiempo_actual - self.start_time

        # --- MÁQUINA DE ESTADOS ---
        
        # 1. DESPEGUE
        if self.estado == 'DESPEGUE':
            if tiempo_transcurrido < self.TIEMPO_DESPEGUE:
                msg.linear.z = self.VEL_SUBIDA
                self.get_logger().info('Despegando...', throttle_duration_sec=1)
            else:
                self.cambiar_estado('AVANZAR')

        # 2. AVANZAR (Hacer un lado)
        elif self.estado == 'AVANZAR':
            if tiempo_transcurrido < self.TIEMPO_AVANCE:
                msg.linear.x = self.VEL_AVANCE
                self.get_logger().info(f'Avanzando Lado {self.lado_actual + 1}...', throttle_duration_sec=1)
            else:
                self.cambiar_estado('ESPERA_1')

        # 3. ESPERA (Frenar antes de girar)
        elif self.estado == 'ESPERA_1':
            if tiempo_transcurrido < self.TIEMPO_ESPERA:
                pass # Velocidad 0
            else:
                self.cambiar_estado('GIRAR')

        # 4. GIRAR (90 Grados a la izquierda)
        elif self.estado == 'GIRAR':
            if tiempo_transcurrido < self.TIEMPO_GIRO:
                msg.angular.z = self.VEL_GIRO
                self.get_logger().info('Girando 90 grados...', throttle_duration_sec=1)
            else:
                self.cambiar_estado('ESPERA_2')

        # 5. ESPERA (Frenar antes del siguiente lado)
        elif self.estado == 'ESPERA_2':
            if tiempo_transcurrido < self.TIEMPO_ESPERA:
                pass
            else:
                self.lado_actual += 1
                if self.lado_actual < self.total_lados:
                    self.cambiar_estado('AVANZAR')
                else:
                    self.cambiar_estado('ATERRIZAR')

        # 6. ATERRIZAR
        elif self.estado == 'ATERRIZAR':
            msg.linear.z = -0.5 # Bajar despacio
            self.get_logger().info('Misión completada. Aterrizando...', throttle_duration_sec=1)
            # Opcional: Salir después de unos segundos
            if tiempo_transcurrido > 5.0:
                msg.linear.z = 0.0 # Parar motores (o dejar que Webots lo pose)

        # Publicar el comando
        self.publisher_.publish(msg)

    def cambiar_estado(self, nuevo_estado):
        self.estado = nuevo_estado
        self.start_time = time.time() # Reiniciar reloj para el nuevo estado
        self.get_logger().info(f'--> CAMBIO DE ESTADO: {nuevo_estado}')

def main(args=None):
    rclpy.init(args=args)
    nodo = MavicCuadrado()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar el dron al cerrar
        parar = Twist()
        nodo.publisher_.publish(parar)
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

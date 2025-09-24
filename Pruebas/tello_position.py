#!/usr/bin/env python3
"""
Script mínimo para conectarse al drone Tello y mostrar su posición
Requiere que el drone esté encendido y conectado por WiFi
"""

import tellopy
import time
import sys

# Variables globales para almacenar los datos más recientes
latest_flight_data = None
latest_log_data = None

def flight_data_handler(event, sender, data, **args):
    """Maneja los eventos de datos de vuelo básicos"""
    global latest_flight_data
    latest_flight_data = data

def log_data_handler(event, sender, data, **args):
    """Maneja los eventos de datos de log (incluye posición MVO)"""
    global latest_log_data
    latest_log_data = data

def print_position_info():
    """Imprime la información de posición disponible"""
    print("\n" + "="*60)
    print("INFORMACIÓN DE POSICIÓN DEL DRONE TELLO")
    print("="*60)
    
    # Datos básicos de vuelo
    if latest_flight_data:
        print(f"DATOS BÁSICOS DE VUELO:")
        print(f"  Altura: {latest_flight_data.height} cm")
        print(f"  Velocidad Norte: {latest_flight_data.north_speed} cm/s")
        print(f"  Velocidad Este: {latest_flight_data.east_speed} cm/s")
        print(f"  Velocidad Suelo: {latest_flight_data.ground_speed} cm/s")
        print(f"  Batería: {latest_flight_data.battery_percentage}%")
        print(f"  WiFi: {latest_flight_data.wifi_strength}")
    else:
        print("  Datos de vuelo: No disponibles")
    
    print()
    
    # Datos de posición MVO (más precisos)
    if latest_log_data and latest_log_data.mvo:
        print(f"POSICIÓN MVO (Visual Odometry):")
        print(f"  Posición X: {latest_log_data.mvo.pos_x:.2f} m")
        print(f"  Posición Y: {latest_log_data.mvo.pos_y:.2f} m") 
        print(f"  Posición Z: {latest_log_data.mvo.pos_z:.2f} m")
        print(f"  Velocidad X: {latest_log_data.mvo.vel_x:.2f} m/s")
        print(f"  Velocidad Y: {latest_log_data.mvo.vel_y:.2f} m/s")
        print(f"  Velocidad Z: {latest_log_data.mvo.vel_z:.2f} m/s")
    else:
        print("  Datos MVO: No disponibles")
    
    # Datos IMU si están disponibles
    if latest_log_data and latest_log_data.imu:
        print(f"\nDATOS IMU:")
        print(f"  Aceleración X: {latest_log_data.imu.acc_x:.2f}")
        print(f"  Aceleración Y: {latest_log_data.imu.acc_y:.2f}")
        print(f"  Aceleración Z: {latest_log_data.imu.acc_z:.2f}")

def main():
    """Función principal"""
    print("Iniciando script de posición del drone Tello...")
    print("Asegúrate de que:")
    print("1. El drone esté encendido")
    print("2. Estés conectado a la red WiFi del Tello")
    print("3. El drone esté en una superficie plana")
    print("\nPresiona Ctrl+C para salir")
    
    # Crear instancia del drone
    drone = tellopy.Tello()
    
    try:
        # Configurar nivel de log para menos ruido
        drone.set_loglevel(drone.LOG_ERROR)
        
        # Suscribirse a los eventos de datos
        drone.subscribe(drone.EVENT_FLIGHT_DATA, flight_data_handler)
        drone.subscribe(drone.EVENT_LOG_DATA, log_data_handler)
        
        # Conectar al drone
        print("\nConectando al drone...")
        drone.connect()
        drone.wait_for_connection(60.0)
        print("¡Conectado exitosamente!")
        
        # Bucle principal para mostrar información
        print("\nRecopilando datos de posición...")
        print("(Los datos pueden tardar unos segundos en aparecer)")
        
        while True:
            print_position_info()
            time.sleep(2)  # Actualizar cada 2 segundos
            
    except KeyboardInterrupt:
        print("\n\nDeteniendo script...")
        
    except Exception as ex:
        print(f"\nError: {ex}")
        print("Verifica que:")
        print("- El drone esté encendido")
        print("- Estés conectado a su red WiFi")
        print("- No haya otros programas usando el drone")
        
    finally:
        # Desconectar del drone
        try:
            drone.quit()
            print("Desconectado del drone.")
        except:
            pass

if __name__ == '__main__':
    main()

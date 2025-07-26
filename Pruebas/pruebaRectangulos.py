import cv2
import numpy as np

def detectar_puertas_rectangulares(frame):

    alto, ancho, _ = frame.shape
    
    # 1. Convertir a espacio de color HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 2. Definir el rango de color rojo en HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Crear máscaras para cada rango de rojo
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    # Combinar ambas máscaras para obtener todos los puntos rojos
    mask_rojo = cv2.bitwise_or(mask1, mask2)

    # 3. Procesar la máscara para encontrar los puntos rojos
    # Aplicar operaciones morfológicas para limpiar el ruido
    kernel = np.ones((5,5), np.uint8)
    mask_rojo = cv2.morphologyEx(mask_rojo, cv2.MORPH_OPEN, kernel)
    mask_rojo = cv2.morphologyEx(mask_rojo, cv2.MORPH_CLOSE, kernel)

    # 4. Encontrar contornos de los puntos rojos
    contornos, _ = cv2.findContours(mask_rojo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    puntos_rojos_detectados = []
    for cnt in contornos:
        # Filtrar ruido
        area = cv2.contourArea(cnt)
        if 50 < area < 2000: 
            # Calcular el centroide
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                puntos_rojos_detectados.append((cx, cy))

    # 5. Agrupar puntos para formar puertas
    puertas_detectadas = []
    # Idea inicial: 4 puntos forman una puerta no rotada
    
    if len(puntos_rojos_detectados) >= 4:
        # Ordenar por Y y luego por X
        puntos_ordenados = sorted(puntos_rojos_detectados, key=lambda p: (p[1], p[0]))
        
        if len(puntos_ordenados) >= 4:
            # Seleccionamos los 4 primeros puntos como candidatos
            candidatos_puerta = puntos_ordenados[:4]
            
            # Ordenamos los 4 candidatos para que sean (arriba_izq, arriba_der, abajo_izq, abajo_der)
            
            # Puntos de arriba: los dos con menor Y
            puntos_arriba = sorted(candidatos_puerta[:2], key=lambda p: p[0])
            # Puntos de abajo: los dos con mayor Y
            puntos_abajo = sorted(candidatos_puerta[2:4], key=lambda p: p[0])
            
            esquinas_puerta_ordenadas = [
                puntos_arriba[0],    # Arriba izquierda
                puntos_arriba[1],    # Arriba derecha
                puntos_abajo[1],     # Abajo derecha
                puntos_abajo[0]      # Abajo izquierda
            ]
            
            # Aquí iría la validación geométrica real:
            # - Distancia entre puntos (para que no estén demasiado cerca o lejos)
            # - Ángulos (cercanos a 90 grados si es un rectángulo)
            # - Relación de aspecto (0.5m x 0.7m -> 0.7 / 0.5 = 1.4 o 0.5 / 0.7 = 0.71)
            #   (distancia vertical / distancia horizontal)
 
            puertas_detectadas.append(esquinas_puerta_ordenadas)

    return puertas_detectadas

if __name__ == "__main__":
    ancho_img, alto_img = 640, 480
    frame_prueba = np.zeros((alto_img, ancho_img, 3), dtype=np.uint8)

    #cv2.circle(imagen, centro, radio, color, grosor)
    #cv2.rectangle(imagen, esquina1, esquina2, color, grosor):

    cv2.circle(frame_prueba, (100, 100), 10, (0, 0, 255), -1) 
    cv2.circle(frame_prueba, (300, 100), 10, (0, 0, 255), -1) 
    cv2.circle(frame_prueba, (300, 300), 10, (0, 0, 255), -1) 
    cv2.circle(frame_prueba, (100, 300), 10, (0, 0, 255), -1) 
    
    cv2.circle(frame_prueba, (50, 50), 10, (0, 0, 255), -1)
    cv2.circle(frame_prueba, (500, 200), 10, (0, 0, 255), -1)

    cv2.rectangle(frame_prueba, (100, 100), (300, 300), (255, 0, 0), 2)

    puertas_encontradas = detectar_puertas_rectangulares(frame_prueba)

    if puertas_encontradas:
        print(f"Se detectaron {len(puertas_encontradas)} puertas:")
        for i, puerta in enumerate(puertas_encontradas):
            print(f"  Puerta {i+1}: {puerta}")
            cv2.polylines(frame_prueba, [np.array(puerta, np.int32).reshape((-1, 1, 2))], True, (0, 255, 0), 3)
            for punto in puerta:
                cv2.circle(frame_prueba, punto, 5, (255, 255, 0), -1) # Círculo amarillo en las esquinas detectadas
    else:
        print("No se detectaron puertas.")

    cv2.imshow("Deteccion de Puertas", frame_prueba)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

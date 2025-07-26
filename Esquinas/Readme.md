-nodoLogicaPuertas:
    Este nodo se encarga de, recibiendo la imagen original, aplicar los filtros de color y calcular las posibles puertas
    en base a los puntos detectados.
    
    Recibe el topic /tello/imagen que es la imagen original sin procesar
    Devuelve el topic /tello/puertas_detectadas , siendo las puertas del tipo:
        puerta_detectada = {
                    'x_centro': x_centro_puerta,
                    'y_centro': y_centro_puerta,
                    'ancho': ancho_puerta,
                    'alto': alto_puerta
                }

        También devuelve la imagen modificada con las puertas dibujadas en el topic /tello/imagen_puertas

                
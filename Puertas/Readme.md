-nodoImagenPuertas5(nodoDeteccion): transforma el punto del centro de la puerta, de ejes cuerpo a ejes mundo

-nodoImagenPuertas6(nodoDeteccion2): igual que el 5, pero esta vez sabiendo que publica la pose el dron y sin cuaterniones

-nodoArucoDeteccion: en vez de detectar puntos naranjas, detecta el aruco como la puerta

-nodoTrayectoria: Dado un punto y un angulo, calcula otros 2 puntos mas y las velocidades necesarias para navegar

-nodoIntermedioPose: Recibe la pose del dron y la publica teniendo en cuenta el error inicial

-nodoTelloPose: El unico que habla y escucha al dron 



cd ~/DronTello/Puertras
source ~/telloPos/bin/activate
python3 nodoTelloPose.py

cd ~/DronTello/Puertras
source ~/tello/bin/activate
python3 nodoIntermedioPose.py

cd ~/DronTello/Puertras
source ~/tello/bin/activate
python3 nodoArucoDeteccion.py

cd ~/DronTello/Puertras
source ~/tello/bin/activate
python3 nodoDeteccion.py


#!/bin/bash

# ./playTello.sh [Aruco|Puertas]

if [ "$1" == "Aruco" ]; then
    DETECTION_NODE="nodoArucoDeteccion.py"
elif [ "$1" == "Puertas" ]; then
    DETECTION_NODE="nodoDeteccion.py"
else
    echo "Uso: $0 [Aruco|Puertas]"
    exit 1
fi

echo "[INICIO] modo: $1"

echo "[LANZADO] nodoTelloPose.py"
gnome-terminal -- bash -c "python3 nodoTelloPose.py; exec bash"

echo "[PAUSA] 6 segundos"
sleep 6

echo "[LANZADO] nodoIntermedioPose.py..."
gnome-terminal -- bash -c "python3 nodoIntermedioPose.py; exec bash"

echo "[PAUSA] 1 segundo"
sleep 1

echo "[LANZADO] $DETECTION_NODE..."
gnome-terminal -- bash -c "python3 $DETECTION_NODE; exec bash"

echo "[PAUSA] 1 segundo"
sleep 1

echo "[LANZADO-DESPEGUE] nodoNavegacion.py"
gnome-terminal -- bash -c "python3 nodoNavegacion.py; exec bash"

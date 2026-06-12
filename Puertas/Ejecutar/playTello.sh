#!/bin/bash

# Uso: ./playTello.sh [Aruco|Puertas] [Visualizar]

SESSION_NAME="dron"
VENV_TELLOPOS="source ~/telloPos/bin/activate"
VENV_TELLO="source ~/tello/bin/activate"
RUTA_SCRIPTS="~/DronTello/Puertas"
#SSID="TELLO-D2AA73" #oreo
SSID="TELLO-621717"


if [ "$1" == "Aruco" ]; then
    DETECTION_NODE="nodoArucoDeteccion.py"
elif [ "$1" == "Puertas" ]; then
    DETECTION_NODE="nodoDeteccion.py"
else
    echo "Uso: $0 [Aruco|Puertas] (Opcional: Visualizar)"
    exit 1
fi

MODO_VISUAL="OFF"
if [ "$2" == "Visualizar" ]; then
    MODO_VISUAL="ON"
    echo "[MODO] Visualización activada (Rviz)"
fi

while true; do
    if nmcli device wifi connect "$SSID" &> /dev/null; then
        echo "[WIFI] Conexión establecida con '$SSID'"
        break
    else
        echo "[BUSCANDO] Red '$SSID'"
        sleep 3
    fi
done
sleep 2

echo "[INICIO] Lanzando todo en una sesión de TMUX llamada '$SESSION_NAME'. Modo: $1"

#tmux kill-session -t $SESSION_NAME 2>/dev/null
tmux new-session -d -s $SESSION_NAME -n "Nodos" 

tmux split-window -v -p 50 -t $SESSION_NAME:0
tmux select-pane -t 0
tmux split-window -h -p 50 -t $SESSION_NAME:0
tmux select-pane -t 2
tmux split-window -h -p 50 -t $SESSION_NAME:0

if [ "$MODO_VISUAL" == "ON" ]; then
    tmux new-window -t $SESSION_NAME -n "Visual"
    tmux split-window -h -p 50 -t $SESSION_NAME:1
fi

tmux select-window -t $SESSION_NAME:0

tmux send-keys -t $SESSION_NAME:0.0 "$VENV_TELLOPOS && python3 $RUTA_SCRIPTS/nodoTelloPose.py" C-m
echo "[LANZADO] nodoTelloPose.py"
sleep 6

tmux send-keys -t $SESSION_NAME:0.1 "$VENV_TELLO && python3 $RUTA_SCRIPTS/nodoIntermedioPose.py" C-m
echo "[LANZADO] nodoIntermedioPose.py"
sleep 1

tmux send-keys -t $SESSION_NAME:0.2 "$VENV_TELLO && python3 $RUTA_SCRIPTS/$DETECTION_NODE" C-m
echo "[LANZADO] $DETECTION_NODE"
sleep 1

tmux send-keys -t $SESSION_NAME:0.3 "$VENV_TELLO && python3 $RUTA_SCRIPTS/nodoNavegacionProbado.py" C-m
echo "[LANZADO-DESPEGUE] nodoNavegacion.py"
sleep 1

if [ "$MODO_VISUAL" == "ON" ]; then
    echo "[LANZANDO] Visualización"
    tmux send-keys -t $SESSION_NAME:1.0 "$VENV_TELLO && python3 $RUTA_SCRIPTS/Visualizar/visualizadorRviz.py" C-m
    tmux send-keys -t $SESSION_NAME:1.1 "rviz2" C-m
fi

tmux attach-session -t $SESSION_NAME
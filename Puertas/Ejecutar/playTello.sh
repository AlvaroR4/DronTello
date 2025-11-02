#!/bin/bash

# ./playTello.sh [Aruco|Puertas]

SESSION_NAME="dron"

VENV_TELLOPOS="source ~/telloPos/bin/activate"
VENV_TELLO="source ~/tello/bin/activate"
RUTA_SCRIPTS="~/DronTello/Puertas"
if [ "$1" == "Aruco" ]; then
    DETECTION_NODE="nodoArucoDeteccion.py"
elif [ "$1" == "Puertas" ]; then
    DETECTION_NODE="nodoDeteccion3.py"
else
    echo "Uso: $0 [Aruco|Puertas]"
    exit 1
fi

echo "[INICIO] Lanzando todo en una sesión de TMUX llamada '$SESSION_NAME'. Modo: $1"

tmux kill-session -t $SESSION_NAME 2>/dev/null
tmux new-session -d -s $SESSION_NAME -n "Nodos" 
tmux split-window -v -p 50
tmux select-pane -t 0
tmux split-window -h -p 50 
tmux select-pane -t 2
tmux split-window -h -p 50 

tmux send-keys -t 0 "$VENV_TELLOPOS && python3 $RUTA_SCRIPTS/nodoTelloPose.py" C-m
echo "[LANZADO] nodoTelloPose.py (Espera 6 segs)"
sleep 6

tmux send-keys -t 1 "$VENV_TELLO && python3 $RUTA_SCRIPTS/nodoIntermedioPose.py" C-m
echo "[LANZADO] nodoIntermedioPose.py (Espera 1 seg)"
sleep 1

tmux send-keys -t 2 "$VENV_TELLO && python3 $RUTA_SCRIPTS/$DETECTION_NODE" C-m
echo "[LANZADO] $DETECTION_NODE (Espera 1 seg)"
sleep 1

tmux send-keys -t 3 "$VENV_TELLO && python3 $RUTA_SCRIPTS/nodoNavegacion.py" C-m
echo "[LANZADO-DESPEGUE] nodoNavegacion.py"

tmux attach-session -t $SESSION_NAME
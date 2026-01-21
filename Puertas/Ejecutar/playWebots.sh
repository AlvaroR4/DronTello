#!/bin/bash

# ./playWebots.sh 

SESION_SIM="simulacion"
SESION_NODOS="nodos"

VENV_TELLO="source ~/tello/bin/activate"
RUTA_WEBOTS="~/DronTello/Webots"
RUTA_SCRIPTS="~/DronTello/Puertas"
WORLD_FILE="/home/$USER/DronTello/Webots/worlds/mavic_world.wbt"

DETECTION_NODE="nodoDeteccion.py"


tmux kill-session -t $SESION_SIM 2>/dev/null
tmux kill-session -t $SESION_NODOS 2>/dev/null

tmux new-session -d -s $SESION_SIM -n "Entorno"

tmux send-keys -t $SESION_SIM:0.0 "$VENV_TELLO && python3 $RUTA_WEBOTS/puenteTelloMavic.py" C-m

tmux split-window -v -p 66 -t $SESION_SIM:0
tmux send-keys -t $SESION_SIM:0.1 "ros2 launch webots_ros2_mavic robot_launch.py world:=$WORLD_FILE" C-m

tmux split-window -v -p 50 -t $SESION_SIM:0
tmux send-keys -t $SESION_SIM:0.2 "rqt" C-m

sleep 10


tmux new-session -d -s $SESION_NODOS -n "Logica"

tmux send-keys -t $SESION_NODOS:0.0 "$VENV_TELLO && cd $RUTA_SCRIPTS && python3 $DETECTION_NODE" C-m
sleep 4

tmux split-window -h -p 50 -t $SESION_NODOS:0
tmux send-keys -t $SESION_NODOS:0.1 "$VENV_TELLO && cd $RUTA_SCRIPTS && python3 nodoNavegacion.py" C-m

echo "tmux ls"
echo "tmux attach -t simulacion"
echo "tmux attach -t nodos"
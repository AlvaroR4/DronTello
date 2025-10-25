#!/bin/bash

SESSION_NAME="dron_complejo"

CMD_TELLO_DIR="cd ~/DronTello"
CMD_TELLO_ENV="cd ~/DronTello; source ~/tello/bin/activate"
CMD_BAGS_DIR="cd ~/Bags"

tmux kill-session -t $SESSION_NAME 2>/dev/null

tmux new-session -d -s $SESSION_NAME -n principal

tmux split-window -h -p 50 

tmux select-pane -t 0
tmux split-window -v -p 66 
tmux select-pane -t 1 
tmux split-window -v -p 50

tmux send-keys -t 0 "$CMD_TELLO_DIR" C-m
tmux send-keys -t 1 "$CMD_TELLO_DIR" C-m
tmux send-keys -t 2 "$CMD_TELLO_DIR" C-m

tmux select-pane -t 3
tmux split-window -v -p 50 

tmux send-keys -t 3 "$CMD_TELLO_ENV" C-m


tmux select-pane -t 4
tmux split-window -h -p 50 

tmux send-keys -t 4 "$CMD_BAGS_DIR" C-m
tmux send-keys -t 5 "$CMD_TELLO_DIR" C-m

tmux select-pane -t 0

tmux attach-session -t $SESSION_NAME

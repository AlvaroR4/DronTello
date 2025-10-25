#!/bin/bash

SESSION_NAME="dron"

CMD_TELLO="cd ~/DronTello; source ~/tello/bin/activate"
CMD_TELLOPOS="cd ~/DronTello; source ~/telloPos/bin/activate"

tmux kill-session -t $SESSION_NAME 2>/dev/null

tmux new-session -d -s $SESSION_NAME

tmux split-window -v

tmux select-pane -t 0
tmux split-window -h

tmux select-pane -t 2
tmux split-window -h

tmux send-keys -t 0 "$CMD_TELLOPOS" C-m  
tmux send-keys -t 1 "$CMD_TELLO" C-m 
tmux send-keys -t 2 "$CMD_TELLO" C-m 
tmux send-keys -t 3 "$CMD_TELLO" C-m 

tmux select-pane -t 0

tmux attach-session -t $SESSION_NAME

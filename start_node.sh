#!/bin/bash

# Configuration
WORKSPACE_PATH=$HOME/<your ros2 workspace>
SESSION=micro_ros

# Kill existing session if present
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "Killing existing tmux session: $SESSION"
    tmux kill-session -t $SESSION
fi

# Create new tmux session with Pane 0 (Micro XRCE Agent)
tmux new-session -d -s $SESSION -n agent
tmux send-keys -t $SESSION:0.0 "MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600" C-m

# Split horizontally to create Pane 1 (ROS 2 node)
tmux split-window -h -t $SESSION:0.0
tmux send-keys -t $SESSION:0.1 "cd $WORKSPACE_PATH && source install/setup.bash" C-m
tmux send-keys -t $SESSION:0.1 "ros2 run example_mode_manual_cpp example_mode_manual" C-m

# Optional: Arrange layout
tmux select-layout -t $SESSION tiled

# Attach to session
tmux attach-session -t $SESSION

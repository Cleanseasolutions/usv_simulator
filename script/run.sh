#!/bin/bash

tmux new-session -d -s my_session

# Pane 1
tmux send-keys -t my_session:0 'source /home/rosuser/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t my_session:0 'roslaunch otter_gazebo otter.launch' C-m

# Split into pane 2 (horizontal split)
tmux split-window -h
tmux send-keys -t my_session:0.1 'source /home/rosuser/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t my_session:0.1 'rosrun controller_node controller_node' C-m

# Split into pane 3 (vertical split)
tmux select-pane -t my_session:0
tmux split-window -v
tmux send-keys -t my_session:0.2 'source /home/rosuser/catkin_ws/devel/setup.bash' C-m
tmux send-keys -t my_session:0.2 'rosrun los_guidance_node los_guidance_node' C-m

# Split into pane 4 (vertical split)
tmux select-pane -t my_session:0.1
tmux split-window -v
tmux send-keys -t my_session:0.3 'source /home/rosuser/catkin_ws/devel/setup.bash' C-m

# Evenly distribute panes
tmux select-layout -t my_session:0 tiled

# Attach to the session
tmux attach-session -t my_session

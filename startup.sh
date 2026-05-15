#!/bin/bash

SESSION="dev"

tmux new-session -d -s "$SESSION" -n 'driver'

# Terminal 1: stretch_driver
tmux send-keys -t "$SESSION:driver" "ros2 launch stretch_core stretch_driver.launch.py" C-m

# Terminal 2: camera
tmux new-window -t "$SESSION" -n 'camera'
tmux send-keys -t "$SESSION:camera" "ros2 launch stretch_core d435i_high_resolution.launch.py" C-m

# Terminal 3: aruco
tmux new-window -t "$SESSION" -n 'aruco'
tmux send-keys -t "$SESSION:aruco" "ros2 launch stretch_core stretch_aruco.launch.py" C-m

# Terminal 4: rviz
tmux new-window -t "$SESSION" -n 'rviz'
tmux send-keys -t "$SESSION:rviz" "ros2 run rviz2 rviz2 -d /home/hello-robot/ament_ws/src/stretch_tutorials/rviz/aruco_detector_example.rviz" C-m

# Terminal 5: rosbridge
tmux new-window -t "$SESSION" -n 'rosbridge'
tmux send-keys -t "$SESSION:rosbridge" "ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

# Terminal 6: web server
tmux new-window -t "$SESSION" -n 'webserver'
tmux send-keys -t "$SESSION:webserver" "cd ~/bresenham/lab13 && python3 -m http.server 8000" C-m

# Opens browser
xdg-open "http://slinky.hcrlab.cs.washington.edu:8000"

# Terminal 7: navigator
tmux new-window -t "$SESSION" -n 'navigator'
tmux send-keys -t "$SESSION:navigator" "cd ~/bresenham/lab13 && python3 aruco_navigator.py" C-m

tmux attach-session -t "$SESSION"
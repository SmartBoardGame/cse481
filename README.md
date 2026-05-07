Terminal 1: ros2 launch stretch_core stretch_driver.launch.py

Terminal 2: ros2 launch rosbridge_server rosbridge_websocket_launch.xml

Terminal 3: 
```
cd ~/bresenham/lab13
python3 -m http.server 8000
```
Can open website in: http://slinky.hcrlab.cs.washington.edu:8000

Terminal 4: 
2 options. Option 1 (if just saving tfs): python3 lab13/pose_saver.py
Option 2 (saving tfs and joint states): python3 lab13/pose_saver_claude.py

Terminal 5: python3 lab13/robot_executor.py (only works with poses saved from old pose_saver.py -- can edit it to fix this though)

Terminal 6 (optional): Keyboard teleop

#!/bin/bash

cd yuvaan_ws/src/yuvaan_controller/script
#rosrun rosserial_python serial_node.py /dev/ttyACM0 & 
rosrun yuvaan_controller controller.py &
roslaunch rosbridge_server rosbridge_websocket.launch & 
python3 ~/app.py &
python3 test2.py

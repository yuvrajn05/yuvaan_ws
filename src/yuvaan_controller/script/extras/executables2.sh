#!/bin/bash

rosrun rosserial_python serial_node.py /dev/ttyACM0 & 
rosrun yuvaan_controller controller.py & 
python3 ~/app.py

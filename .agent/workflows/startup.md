---
description: System startup workflow for Yuvaan robot
---

# Yuvaan Robot System Startup Workflow

This workflow guides you through starting up the Yuvaan robot system, including ROS nodes, motor controllers, and camera streaming.

## Prerequisites
- Jetson device should be powered on
- Network connectivity between laptop and Jetson
- USB devices connected to Jetson (ttyUSB0 for main motors, ttyUSB1 for manipulator)

## Startup Steps

### 1. Check Network Connectivity
First, verify that the Jetson is reachable:
```bash
ping 192.168.2.101
```
Press `Ctrl+C` to stop the ping once you confirm connectivity.

### 2. SSH to Jetson
Connect to the Jetson device:
```bash
ssh jetson@192.168.2.101
```

### 3. Start ROS Core (on Jetson)
In the SSH session, start the ROS master:
```bash
roscore
```
**Note:** Keep this terminal open. Open a new terminal/SSH session for subsequent commands.

### 4. Start Main Controller (on Jetson)
In a new SSH session to Jetson:
```bash
rosrun yuvaan_controller merged_control.py
```
This starts the merged control script that handles both unified and manipulator controls.

### 5. Start Joystick Node (on Laptop - OPTIONAL)
If you want to control the robot with a joystick, run this on your laptop:
```bash
rosrun joy joy_node
```

### 6. Verify Unified Commands (on Jetson - OPTIONAL)
To check that unified commands are being published correctly:
```bash
rostopic echo /unified_command
```
Press `Ctrl+C` to stop monitoring.

### 7. Start Main Motor Controller (on Jetson - OPTIONAL)
To enable physical motor control:
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 __name:=serial_node_1
```

### 8. Verify Manipulator Commands (on Jetson - OPTIONAL)
If using the manipulator, verify its commands:
```bash
rostopic echo /mani_motor_command
```
Press `Ctrl+C` to stop monitoring.

### 9. Start Manipulator Motor Controller (on Jetson - OPTIONAL)
To enable manipulator motor control:
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB1 __name:=serial_node_2
```

### 10. Start Camera Video Stream (on Jetson - OPTIONAL)
To stream camera feed:
```bash
python3 yuvaan_ws/src/yuvaan_controller/script/video_stream.py
```

### 11. Access Camera Feed
Open a web browser and navigate to the appropriate URL (typically the Jetson's IP address with the port specified by the video stream script).

## Notes
- Steps marked as OPTIONAL can be run based on your current needs
- Each step that needs to keep running should be executed in a separate terminal/SSH session
- Make sure to set up ROS environment variables if they're not already in your `.bashrc`
- The order of steps is important - roscore must be running before other ROS nodes
- Check USB device names (/dev/ttyUSB0, /dev/ttyUSB1) as they may change between reboots

## Troubleshooting
- If ping fails, check network connection and Jetson power
- If SSH fails, verify credentials and network settings
- If ROS commands fail, ensure ROS environment is sourced: `source /opt/ros/noetic/setup.bash` (or your ROS version)
- If serial nodes fail, check USB device permissions and connections

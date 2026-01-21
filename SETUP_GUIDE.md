# Setup Guide: Unified Drive + Servo Control

## Complete Build and Setup Instructions

### Step 1: Build the ROS Workspace
```bash
cd /home/neem/yuvaan_ws
catkin_make
source devel/setup.bash
```

This will:
- ✅ Compile the new `drive_servo.msg` message
- ✅ Generate C++, Python, and other language bindings
- ✅ Build all ROS nodes

### Step 2: Regenerate Arduino ROS Libraries
```bash
cd /home/neem/yuvaan_ws
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

This will:
- ✅ Create fresh `ros_lib` folder with ALL ROS message headers
- ✅ Include the new `drive_servo.h` header for Arduino
- ✅ Export over 400+ message/service definitions

> **Important**: You MUST run this whenever you add/modify custom messages!

### Step 3: Copy Arduino Libraries (if needed)
If your Arduino IDE doesn't see the libraries:
```bash
# Option 1: Copy to Arduino libraries folder
cp -r /home/neem/yuvaan_ws/ros_lib ~/Arduino/libraries/

# Option 2: Use the ros_lib from workspace directly
# In Arduino IDE → Preferences → Set Sketchbook location to /home/neem/yuvaan_ws
```

### Step 4: Upload Arduino Code
1. Open Arduino IDE
2. File → Open → `/home/neem/yuvaan_ws/drive_servo_controller/drive_servo_controller.ino`
3. Install required libraries:
   - **CytronMotorDriver** (from Library Manager)
4. Verify the sketch compiles (check for `#include <yuvaan_controller/drive_servo.h>`)
5. Upload to Arduino Uno

### Step 5: Test the System
```bash
# Terminal 1: Launch everything
roslaunch yuvaan_controller drive_servo.launch

# Terminal 2: Monitor messages
rostopic echo /drive_servo_command

# Terminal 3: Check message structure
rostopic info /drive_servo_command
rosmsg show yuvaan_controller/drive_servo
```

## Verification Checklist

- [ ] `catkin_make` completes without errors
- [ ] `ros_lib/yuvaan_controller/drive_servo.h` exists
- [ ] Arduino sketch compiles without errors
- [ ] Launch file starts all 3 nodes (joy, control, rosserial)
- [ ] `/drive_servo_command` topic appears in `rostopic list`
- [ ] D-pad and triggers control drive + servo

## Common Issues

**"fatal error: yuvaan_controller/drive_servo.h: No such file"**
- Solution: Regenerate Arduino libraries (Step 2)

**"Package 'yuvaan_controller' not found"**
- Solution: Run `source devel/setup.bash` in every terminal

**Arduino upload fails**
- Check serial port permissions: `sudo usermod -a -G dialout $USER`
- Logout and login again

**No messages on topic**
- Check joystick connection: `ls /dev/input/js*`
- Check Arduino connection: `ls /dev/ttyACM*`
- Check rosserial: `rostopic list | grep rosout`

## File Locations Reference

```
/home/neem/yuvaan_ws/
├── src/yuvaan_controller/
│   ├── msg/
│   │   └── drive_servo.msg          # Custom message definition
│   ├── script/
│   │   └── drive_servo_control.py   # ROS node
│   └── launch/
│       └── drive_servo.launch       # Launch file
├── drive_servo_controller/
│   └── drive_servo_controller.ino   # Arduino sketch
└── ros_lib/                         # Generated Arduino libraries
    └── yuvaan_controller/
        └── drive_servo.h            # Arduino header (auto-generated)
```

## Quick Reference Commands

```bash
# Build
catkin_make

# Source environment
source devel/setup.bash

# Regenerate Arduino libs
rosrun rosserial_arduino make_libraries.py .

# Run system
roslaunch yuvaan_controller drive_servo.launch

# Debug
rostopic echo /drive_servo_command
rostopic hz /drive_servo_command
rosmsg show yuvaan_controller/drive_servo
```

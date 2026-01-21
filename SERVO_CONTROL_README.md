# 360° Continuous Rotation Servo Joystick Control System

Control your Robokits RKI 1201 **360-degree continuous rotation servo** using a gamepad/joystick D-pad!

## 📁 Files Created

1. **ROS Node**: `script/servo_control.py` - Python node that reads joystick and publishes servo speed commands
2. **Arduino Sketch**: `servo_controller/servo_controller.ino` - Arduino code for continuous servo control
3. **Launch File**: `launch/servo_joy.launch` - Easy one-command startup

## 🎮 Controls

- **D-pad Left** → Servo rotates **CCW** (counter-clockwise) continuously
- **D-pad Right** → Servo rotates **CW** (clockwise) continuously  
- **Release D-pad** → Servo **STOPS**

> **Note**: This is a continuous rotation servo - it doesn't hold positions like a standard servo. Instead, it rotates continuously based on speed commands. The further you push the D-pad, the faster it rotates!

## 🔌 Hardware Setup

### Arduino Connections
```
Servo Signal → Arduino Pin 9
Servo VCC    → 5V (or external power if high torque)
Servo GND    → GND
```

### Arduino Setup
1. Open Arduino IDE
2. Install `rosserial_arduino` library:
   - Go to **Sketch → Include Library → Manage Libraries**
   - Search for "rosserial"
   - Install "Rosserial Arduino Library"
3. Open `servo_controller.ino`
4. Upload to your Arduino Uno

## 🚀 Running the System

### Step 1: Build the workspace (if needed)
```bash
cd /home/neem/yuvaan_ws
catkin_make
source devel/setup.bash
```

### Step 2: Launch everything with one command
```bash
roslaunch yuvaan_controller servo_joy.launch
```

### Alternative: Run nodes separately
```bash
# Terminal 1 - ROS core
roscore

# Terminal 2 - Joystick node
roslaunch yuvaan_controller joy.launch

# Terminal 3 - Servo control node
rosrun yuvaan_controller servo_control.py

# Terminal 4 - Arduino serial connection
rosrun rosserial_python serial_node.py /dev/ttyACM0 115200
```

## 🔧 Configuration

### Adjust servo speed range
Edit `servo_control.py` line 23:
```python
self.max_speed = 90  # Maximum speed offset (0-90)
# Lower values = slower max speed, Higher values = faster max speed
```

### Understanding servo control values
The servo receives values from 0-180:
- **90** = STOPPED
- **0-89** = CCW rotation (0 = full speed CCW, 89 = very slow CCW)
- **91-180** = CW rotation (91 = very slow CW, 180 = full speed CW)

### Change Arduino pin
Edit `servo_controller.ino` line 19:
```cpp
const int SERVO_PIN = 9;  // Change to your preferred pin
```

### Different serial port
Edit `servo_joy.launch` line 13 if your Arduino is on a different port:
```xml
<param name="port" value="/dev/ttyUSB0" />
```

## 🐛 Troubleshooting

**Joystick not detected?**
- Check: `ls /dev/input/js*`
- Update device path in launch file

**Arduino not connecting?**
- Check: `ls /dev/ttyACM* /dev/ttyUSB*`
- Ensure user is in `dialout` group: `sudo usermod -a -G dialout $USER`
- Logout and login again

**Servo not moving?**
- Check Arduino Serial Monitor (disable rosserial first)
- Verify servo power supply
- Check servo wire connections

## 🔍 Monitoring

Check servo speed commands:
```bash
rostopic echo /servo_command
# You'll see values: 90 = stop, <90 = CCW, >90 = CW
```

Check joystick input:
```bash
rostopic echo /joy
```

## 📊 ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Joystick input |
| `/servo_command` | `std_msgs/Int16` | Servo speed (0-180, where 90=stop) |

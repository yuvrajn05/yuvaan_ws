# Dual Servo Control System

Control 2 independent 360° continuous rotation servos using the **right analog stick**!

## 📁 Files Created

1. **[dual_servo.msg](file:///home/neem/yuvaan_ws/src/yuvaan_controller/msg/dual_servo.msg)** - Custom message definition
2. **[dual_servo_control.py](file:///home/neem/yuvaan_ws/src/yuvaan_controller/script/dual_servo_control.py)** - ROS node
3. **[dual_servo_controller.ino](file:///home/neem/yuvaan_ws/dual_servo_controller/dual_servo_controller.ino)** - Arduino sketch
4. **[dual_servo.launch](file:///home/neem/yuvaan_ws/src/yuvaan_controller/launch/dual_servo.launch)** - Launch file

## 🎮 Controls

| Input | Servo | Action |
|-------|-------|--------|
| **Right Stick Left/Right** (X axis) | Servo 1 | Hold left = CCW, Hold right = CW |
| **Right Stick Up/Down** (Y axis) | Servo 2 | Hold up = CCW, Hold down = CW |
| **Release stick** | Both | STOP |

> **Note**: Includes 0.1 deadzone to prevent drift when stick is centered

## 🔌 Hardware Setup

### Arduino Connections
```
SERVO 1 (Right Stick X - horizontal):
  Signal → Pin 9
  VCC → 5V (or external power)
  GND → GND

SERVO 2 (Right Stick Y - vertical):
  Signal → Pin 10
  VCC → 5V (or external power)
  GND → GND
```

### Required Libraries
- **Rosserial Arduino Library**

## 🚀 Quick Start

### Step 1: Build the Workspace
```bash
cd /home/neem/yuvaan_ws
catkin_make
source devel/setup.bash
```

### Step 2: Regenerate Arduino Libraries
```bash
cd /home/neem/yuvaan_ws
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

### Step 3: Upload Arduino Code
1. Open `dual_servo_controller.ino` in Arduino IDE
2. Verify library `rosserial_arduino` is installed
3. Upload to Arduino Uno

### Step 4: Run the System
```bash
roslaunch yuvaan_controller dual_servo.launch
```

## ✨ Key Features

### 🎯 Hold-to-Move Control
- **Hold** analog stick in any direction → servo rotates
- **Release** → servo stops immediately (1500μs)
- Works continuously at 20Hz

### 🛡️ Safety Features
- **500ms timeout** - stops servos if connection lost
- **Deadzone** - 0.1 threshold prevents stick drift
- **Range protection** - constrained to 1000-2000μs

### 🔄 Independent Control
- Both servos operate independently
- Smooth simultaneous movement
- No interference between axes

## 🔧 Configuration

### Adjust Rotation Speed
Edit `dual_servo_control.py` line 20:
```python
self.rotation_speed = 100  # Microseconds offset from 1500
# Higher = faster rotation, Lower = slower rotation
```

### Adjust Deadzone
Edit `dual_servo_control.py` lines 66, 75:
```python
if R_Analog_X < -0.1:  # Change 0.1 to adjust deadzone
```

### Change Arduino Pins
Edit `dual_servo_controller.ino` lines 22-23:
```cpp
const int SERVO1_PIN = 9;   // Change pin numbers
const int SERVO2_PIN = 10;
```

## 🐛 Troubleshooting

**Build error: "dual_servo.msg not found"**
```bash
cd /home/neem/yuvaan_ws
catkin_make
source devel/setup.bash
```

**Arduino compile error: Missing header**
```bash
cd /home/neem/yuvaan_ws
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

**Servos drifting when stick centered**
- Increase deadzone threshold (try 0.15 or 0.2)

**Servos not responding**
- Check serial port in launch file
- Verify servo power supply
- Test: `rostopic echo /dual_servo_command`

## 📊 ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Joystick input |
| `/dual_servo_command` | `yuvaan_controller/dual_servo` | Servo commands |

## 📈 Message Structure

```
dual_servo.msg:
  int32 servo1_speed  # 1000-2000 microseconds (1500=stop)
  int32 servo2_speed  # 1000-2000 microseconds (1500=stop)
```

## 🔍 Monitoring

```bash
# Watch servo commands
rostopic echo /dual_servo_command

# Check message rate
rostopic hz /dual_servo_command

# View message details
rosmsg show yuvaan_controller/dual_servo
```

## 🎛️ Joystick Axis Reference

| Axis | Index | Description |
|------|-------|-------------|
| Right Stick X | `axes[3]` | Horizontal (servo 1) |
| Right Stick Y | `axes[4]` | Vertical (servo 2) |

**Note**: Axis values range from -1.0 (left/down) to +1.0 (right/up)

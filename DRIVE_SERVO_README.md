# Unified Drive + Servo Control System

Complete control system combining differential drive with 360° continuous rotation servo, all controlled from a single joystick!

## 📁 Files Created

### Core Implementation
1. **[drive_servo_control.py](file:///home/neem/yuvaan_ws/src/yuvaan_controller/script/drive_servo_control.py)** - Unified ROS node
2. **[drive_servo_controller.ino](file:///home/neem/yuvaan_ws/drive_servo_controller/drive_servo_controller.ino)** - Unified Arduino sketch
3. **[drive_servo.msg](file:///home/neem/yuvaan_ws/src/yuvaan_controller/msg/drive_servo.msg)** - Custom message definition
4. **[drive_servo.launch](file:///home/neem/yuvaan_ws/src/yuvaan_controller/launch/drive_servo.launch)** - Launch file

## 🎮 Complete Control Scheme

### Drive Controls
| Input | Function |
|-------|----------|
| **RT** (Right Trigger) | Forward throttle |
| **LT** (Left Trigger) | Reverse throttle |
| **Left Analog X** | Steering (turning) |
| **Button A** | Cycle speed modes |

### Speed Modes
1. **SLOW** - 64 max speed (25% power)
2. **MEDIUM** - 127 max speed (50% power)
3. **FAST** - 191 max speed (75% power)
4. **MAX** - 255 max speed (100% power)

### Servo Controls
| Input | Function |
|-------|----------|
| **D-pad Left** | Step CCW (decrease speed by 10μs) |
| **D-pad Right** | Step CW (increase speed by 10μs) |

> **Note**: Each D-pad press moves the servo by **10 microseconds**. The servo remembers its speed and continues rotating until you press D-pad again. Press repeatedly to go faster! 1500μs = stopped.

## 🔌 Hardware Setup

### Arduino Connections
```
LEFT MOTOR DRIVER:
  PWM → Pin 3
  DIR → Pin 2

RIGHT MOTOR DRIVER:
  PWM → Pin 5
  DIR → Pin 4

360° SERVO:
  Signal → Pin 9
  VCC → 5V (or external power)
  GND → GND
```

### Required Libraries
Arduino IDE → Library Manager:
- **Rosserial Arduino Library**
- **CytronMotorDriver** (for motor control)

## 🚀 Quick Start

### Step 1: Build the Message
```bash
cd /home/neem/yuvaan_ws
catkin_make
source devel/setup.bash
```

### Step 2: Upload Arduino Code
1. Open `drive_servo_controller.ino` in Arduino IDE
2. Install required libraries (rosserial, CytronMotorDriver)
3. Upload to Arduino Uno

### Step 3: Run Everything
```bash
roslaunch yuvaan_controller drive_servo.launch
```

## ✨ Key Features

### 🎯 Microsecond Servo Control
- Uses `writeMicroseconds()` for maximum precision
- Range: **1000-2000μs** (vs 0-180° = 10x more precision!)
- **1500μs** = perfect stop position
- **Incremental step control** - each D-pad press adjusts by 10μs
- Servo "remembers" its speed between joystick commands

### 🛡️ Safety Features
- **500ms timeout** - automatically stops everything if connection lost
- Speed constraining on motors (-255 to 255)
- Servo range protection (1000-2000μs)

### 🔄 Differential Drive
- Tank-style steering with velocity mixing
- Left motor: `linear + angular`
- Right motor: `linear - angular`

### 🎛️ Mode Management
- 4 speed modes with button debouncing
- Mode feedback via ROS logging
- Smooth mode transitions

## 🔧 Configuration

### Adjust Servo Step Size
Edit `drive_servo_control.py` line 24:
```python
self.step_size_us = 10  # Microseconds per D-pad press
# Smaller = finer control, Larger = faster speed changes
```

### Adjust Starting Servo Speed
Edit `drive_servo_control.py` line 23:
```python
self.servo_speed = 1500  # Start at stopped (1000-2000μs range)
```

### Adjust Speed Limits
Edit the mode speed multipliers in `drive_servo_control.py`:
```python
if mode == 1:   # Customize slow mode
    vel_linear_x = int(64 * (RT - LT) / 2)
```

### Change Safety Timeout
Edit `drive_servo_controller.ino`:
```cpp
const unsigned long TIMEOUT_MS = 500;  // Milliseconds
```

## 🐛 Troubleshooting

**Build error: "drive_servo.msg not found"**
```bash
cd /home/neem/yuvaan_ws
catkin_make
source devel/setup.bash
```

**Arduino compile error: Missing libraries**
- Install `rosserial_arduino` and `CytronMotorDriver` from Library Manager

**Motors not responding**
- Check motor driver pin connections (PWM/DIR)
- Verify power supply to motor drivers
- Check ROS topic: `rostopic echo /drive_servo_command`

**Servo not stopping at neutral**
- Calibrate neutral point (try 1495-1505μs)
- Check servo power supply

## 📊 ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/joy` | `sensor_msgs/Joy` | Joystick input |
| `/drive_servo_command` | `yuvaan_controller/drive_servo` | Unified control commands |

## 📈 Message Structure

```
drive_servo.msg:
  int32 vel_linear_x    # -255 to 255
  int32 vel_angular_z   # -255 to 255
  int32 mode            # 1, 2, 3, or 4
  int32 servo_speed     # 1000-2000 microseconds
```

## 🔍 Monitoring

```bash
# Watch all commands
rostopic echo /drive_servo_command

# Watch only servo speed
rostopic echo /drive_servo_command/servo_speed

# Check message rate
rostopic hz /drive_servo_command
```

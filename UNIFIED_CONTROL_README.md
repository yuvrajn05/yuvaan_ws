# Unified Control System

**Everything in One Controller**: Drive Motors + 3 Servos from a Single Joystick!

## 📦 What This Controls

- **2x Drive Motors** (differential drive)
- **1x D-pad Servo** (hold-to-move)
- **2x Analog Stick Servos** (proportional control)

## 🎮 Complete Control Layout

### Drive System
| Input | Function |
|-------|----------|
| **RT/LT** (Triggers) | Forward/Reverse throttle |
| **Left Stick X** | Steering (turning) |
| **Button A** | Cycle speed modes (Slow/Med/Fast/Max) |

### Servo 1 - D-pad (Hold-to-Move)
| Input | Function |
|-------|----------|
| **D-pad Left** (hold) | Rotate CCW @ fixed speed |
| **D-pad Right** (hold) | Rotate CW @ fixed speed |
| **D-pad Release** | STOP (1500μs) |

### Servos 2 & 3 - Right Stick (Proportional)
| Input | Function |
|-------|----------|
| **Right Stick X** | Servo 2 speed (proportional) |
| **Right Stick Y** | Servo 3 speed (proportional) |
| **Stick Position** | = Servo speed (gentle push = slow) |

## 🔌 Hardware Setup

```
ARDUINO UNO CONNECTIONS:

MOTORS:
  Left Motor Driver:  PWM → Pin 3, DIR → Pin 2
  Right Motor Driver: PWM → Pin 5, DIR → Pin 4

SERVOS:
  D-pad Servo:        Signal → Pin 9
  Right Stick X Servo: Signal → Pin 10
  Right Stick Y Servo: Signal → Pin 11
```

### Required Hardware
- Arduino Uno
- 2x Cytron Motor Drivers
- 3x 360° Continuous Rotation Servos
- Gamepad/Joystick

### Required Libraries (Arduino)
- Rosserial Arduino Library
- CytronMotorDriver

## 🚀 Quick Start

```bash
# 1. Build workspace
cd /home/neem/yuvaan_ws
catkin_make
source devel/setup.bash

# 2. Upload unified_controller.ino to Arduino Uno

# 3. Run everything
roslaunch yuvaan_controller unified.launch
```

## 📝 Files

| File | Description |
|------|-------------|
| [unified_control.msg](file:///home/neem/yuvaan_ws/src/yuvaan_controller/msg/unified_control.msg) | Message (6 fields) |
| [unified_control.py](file:///home/neem/yuvaan_ws/src/yuvaan_controller/script/unified_control.py) | ROS node |
| [unified_controller.ino](file:///home/neem/yuvaan_ws/unified_controller/unified_controller.ino) | Arduino sketch |
| [unified.launch](file:///home/neem/yuvaan_ws/src/yuvaan_controller/launch/unified.launch) | Launch file |

## ⚙️ Configuration

### Adjust D-pad Servo Speed
Edit `unified_control.py` line 25:
```python
self.dpad_speed = 100  # Microseconds offset (higher = faster)
```

### Adjust Drive Speeds
Edit mode multipliers in `unified_control.py` lines 98-111

### Change Servo Pins
Edit `unified_controller.ino` lines 33-35:
```cpp
const int SERVO_DPAD_PIN = 9;
const int SERVO_STICK_X_PIN = 10;
const int SERVO_STICK_Y_PIN = 11;
```

## 🎯 Use Cases

**Perfect for:**
- **Mobile robots** with camera pan/tilt
- **RC vehicles** with accessory control
- **Robotic arms** on mobile platforms
- **Any project** needing drive + multiple servos

## 📊 Message Structure

```
unified_control.msg:
  int32 vel_linear_x    # -255 to 255 (drive)
  int32 vel_angular_z   # -255 to 255 (drive)
  int32 mode            # 1, 2, 3, or 4
  int32 servo_dpad      # 1000-2000μs
  int32 servo_stick_x   # 1000-2000μs
  int32 servo_stick_y   # 1000-2000μs
```

## 🔍 Monitoring

```bash
# Watch all commands
rostopic echo /unified_command

# Check message rate
rostopic hz /unified_command

# View message structure
rosmsg show yuvaan_controller/unified_control
```

## 🆚 Comparison with Other Systems

| System | Motors | Servos | Inputs Used |
|--------|--------|--------|-------------|
| **Unified** | 2 | 3 | Triggers, L-stick, D-pad, R-stick |
| Drive+Servo | 2 | 1 | Triggers, L-stick, D-pad |
| Dual Servo | 0 | 2 | R-stick only |

## ✨ Key Features

- **20Hz continuous publishing** - No timeout issues
- **Proportional control** - Variable speed servos
- **500ms safety timeout** - Auto-stops on disconnect
- **4 speed modes** - Flexible drive speeds
- **All-in-one** - Single launch file, single Arduino sketch

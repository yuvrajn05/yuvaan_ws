# ⚠️ IMPORTANT: Continuous Publishing

## The Problem with Callback-Only Publishing

The original code had a critical issue:

### Before (Callback-Only):
```python
def joy_callback(self, msg):
    # Process joystick and publish
    self.command_pub.publish(cmd)
```

**Problem**: ROS joystick callbacks only fire when values **CHANGE**. If you:
- Hold a trigger down → callback stops firing after initial press
- Hold analog stick → callback stops firing  
- Result: **Arduino timeout (500ms) stops the motors!**

## The Solution: Timer-Based Publishing

### After (Timer + Callback):
```python
def joy_callback(self, msg):
    # Just store the latest state
    self.last_joy_msg = msg

def timer_callback(self, event):
    # Publish at 20 Hz continuously
    # Uses the last stored joystick state
    self.command_pub.publish(cmd)
```

**Benefit**: Commands publish **continuously at 20Hz** regardless of whether joystick changes!

## How It Works

1. **Joystick Callback** - Stores joystick state when it changes
2. **Timer Callback** - Runs every 50ms (20Hz) and:
   - Reads last stored joystick state
   - Calculates drive speeds
   - Calculates servo speed  
   - Publishes command

3. **Result**: Even if you hold trigger at constant value, Arduino receives commands 20 times per second → No timeout!

## Configuration

Edit the publish rate in `drive_servo_control.py` line 35:
```python
self.publish_rate = 20  # Hz (20 = 50ms interval)
# Higher = more responsive but more CPU usage
# Lower = less CPU but may timeout
```

**Recommended**: 10-50 Hz (Arduino timeout is 500ms, so minimum ~3 Hz)

## Why 20 Hz?

- **Fast enough**: Well above Arduino timeout threshold (500ms)
- **Smooth control**: 50ms updates feel responsive
- **Not wasteful**: Doesn't spam unnecessary messages
- **Standard**: Common rate for robot control systems

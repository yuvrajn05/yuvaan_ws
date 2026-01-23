#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import unified_control

class UnifiedControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('unified_control_node', anonymous=True)
        
        # Publisher for unified commands
        self.command_pub = rospy.Publisher('unified_command', unified_control, queue_size=10)
        
        # Subscriber to joystick
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # Drive mode management
        self.current_mode_index = 0
        self.modes = [1, 2, 3, 4]  # 1=slow, 2=medium, 3=fast, 4=max
        self.mode_names = ["SLOW", "MEDIUM", "FAST", "MAX"]
        
        # Servo speeds (microseconds)
        self.servo_dpad = 1500      # D-pad controlled servo
        self.servo_stick_x = 1500   # Right stick X servo
        self.servo_stick_y = 1500   # Right stick Y servo
        
        # D-pad servo parameters
        self.dpad_speed = 250  # Speed offset for D-pad servo
        
        # Button debouncing
        self.last_mode_change_time = 0
        self.debounce_delay = 0.3
        
        # Store last joystick state for continuous publishing
        self.last_joy_msg = None
        
        # Timer for continuous publishing (20 Hz)
        self.publish_rate = 20
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        
        rospy.loginfo("=" * 70)
        rospy.loginfo("UNIFIED CONTROL NODE STARTED - Drive + 3 Servos")
        rospy.loginfo("=" * 70)
        rospy.loginfo("DRIVE CONTROLS:")
        rospy.loginfo("  RT/LT → Forward/Reverse")
        rospy.loginfo("  Left Stick X → Steering")
        rospy.loginfo("  Button A → Cycle Modes (Slow/Medium/Fast/Max)")
        rospy.loginfo("")
        rospy.loginfo("SERVO 1 (D-pad - Hold-to-Move):")
        rospy.loginfo("  D-pad Left → CCW rotation")
        rospy.loginfo("  D-pad Right → CW rotation")
        rospy.loginfo("  D-pad Release → STOP")
        rospy.loginfo("")
        rospy.loginfo("SERVO 2 & 3 (Right Stick - Proportional):")
        rospy.loginfo("  Right Stick X → Servo 2 (horizontal)")
        rospy.loginfo("  Right Stick Y → Servo 3 (vertical)")
        rospy.loginfo("  Stick position = servo speed")
        rospy.loginfo("")
        rospy.loginfo(f"Publishing at {self.publish_rate} Hz (continuous)")
        rospy.loginfo("=" * 70)
    
    def joy_callback(self, msg):
        """Store the latest joystick state"""
        self.last_joy_msg = msg
    
    def timer_callback(self, event):
        """Called continuously at 20 Hz"""
        if self.last_joy_msg is None:
            return
        
        msg = self.last_joy_msg
        
        # ===== JOYSTICK MAPPING =====
        LT = -msg.axes[2]           # Left trigger
        RT = -msg.axes[5]           # Right trigger
        L_Analog_X = -msg.axes[0]   # Left stick X (turning)
        R_Analog_X = msg.axes[3]    # Right stick X
        R_Analog_Y = msg.axes[4]    # Right stick Y
        D_X = msg.axes[6]           # D-pad X
        A = msg.buttons[0]          # A button
        
        # ===== DRIVE MODE MANAGEMENT =====
        current_time = rospy.get_time()
        if A == 1 and (current_time - self.last_mode_change_time) > self.debounce_delay:
            self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
            self.last_mode_change_time = current_time
            rospy.loginfo(f"Mode: {self.mode_names[self.current_mode_index]}")
        
        mode = self.modes[self.current_mode_index]
        
        # ===== DRIVE SPEED CALCULATION =====
        if mode == 1:   # Slow
            vel_linear_x = int(64 * (RT - LT) / 2)
            vel_angular_z = int(64 * L_Analog_X)
        elif mode == 2: # Medium
            vel_linear_x = int(127 * (RT - LT) / 2)
            vel_angular_z = int(127 * L_Analog_X)
        elif mode == 3: # Fast
            vel_linear_x = int(191 * (RT - LT) / 2)
            vel_angular_z = int(191 * L_Analog_X)
        else:           # Max
            vel_linear_x = int(255 * (RT - LT) / 2)
            vel_angular_z = int(255 * L_Analog_X)
        
        # ===== D-PAD SERVO (Hold-to-Move) =====
        if D_X < 0:  # Left
            self.servo_dpad = 1500 - self.dpad_speed
        elif D_X > 0:  # Right
            self.servo_dpad = 1500 + self.dpad_speed
        else:  # Released
            self.servo_dpad = 1500
        
        # ===== RIGHT STICK SERVOS (Proportional) =====
        deadzone = 0.1
        
        # Servo X (horizontal)
        if abs(R_Analog_X) < deadzone:
            self.servo_stick_x = 1500
        else:
            self.servo_stick_x = int(1500 + (500 * R_Analog_X))
        
        # Servo Y (vertical)
        if abs(R_Analog_Y) < deadzone:
            self.servo_stick_y = 1500
        else:
            self.servo_stick_y = int(1500 + (500 * R_Analog_Y))
        
        # Constrain all servos
        self.servo_dpad = max(1000, min(2000, self.servo_dpad))
        self.servo_stick_x = max(1000, min(2000, self.servo_stick_x))
        self.servo_stick_y = max(1000, min(2000, self.servo_stick_y))
        
        # ===== PUBLISH UNIFIED MESSAGE =====
        cmd = unified_control()
        cmd.mode = mode
        cmd.vel_linear_x = vel_linear_x
        cmd.vel_angular_z = vel_angular_z
        cmd.servo_dpad = self.servo_dpad
        cmd.servo_stick_x = self.servo_stick_x
        cmd.servo_stick_y = self.servo_stick_y
        
        self.command_pub.publish(cmd)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = UnifiedControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

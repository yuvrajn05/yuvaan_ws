#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import drive_servo

class DriveServoControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('drive_servo_control_node', anonymous=True)
        
        # Publisher for combined drive and servo commands
        self.command_pub = rospy.Publisher('drive_servo_command', drive_servo, queue_size=10)
        
        # Subscriber to joystick
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # Drive mode management
        self.current_mode_index = 0
        self.modes = [1, 2, 3, 4]  # 1=slow, 2=medium, 3=fast, 4=max
        self.mode_names = ["SLOW", "MEDIUM", "FAST", "MAX"]
        
        # Servo control (using microseconds for precision)
        self.servo_speed = 1500    # Current servo speed in microseconds (1500 = stop)
        self.step_size_us = 100 # Microsecond step size per D-pad press
        
        # Button debouncing
        self.last_mode_change_time = 0
        self.last_dpad_state = 0   # Track D-pad state for edge detection
        self.debounce_delay = 0.3  # 300ms debounce
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Drive + 360° Servo Control Node Started")
        rospy.loginfo("=" * 60)
        rospy.loginfo("DRIVE CONTROLS:")
        rospy.loginfo("  Triggers (RT/LT) → Forward/Backward")
        rospy.loginfo("  Left Analog X → Turning")
        rospy.loginfo("  Button A → Cycle Speed Modes (Slow/Med/Fast/Max)")
        rospy.loginfo("")
        rospy.loginfo("SERVO CONTROLS:")
        rospy.loginfo("  D-pad Left → CCW rotation")
        rospy.loginfo("  D-pad Right → CW rotation")
        rospy.loginfo("  D-pad Release → Stop")
        rospy.loginfo("=" * 60)
    
    def joy_callback(self, msg):
        """
        Unified callback for joystick input controlling both drive and servo
        """
        # ===== JOYSTICK MAPPING =====
        LT = -msg.axes[2]           # Left trigger
        RT = -msg.axes[5]           # Right trigger
        L_Analog_X = -msg.axes[0]   # Left stick X (turning)
        D_X = msg.axes[6]           # D-pad X axis
        A = msg.buttons[0]          # A button (mode change)
        
        # ===== DRIVE MODE MANAGEMENT =====
        current_time = rospy.get_time()
        if A == 1 and (current_time - self.last_mode_change_time) > self.debounce_delay:
            self.current_mode_index = (self.current_mode_index + 1) % len(self.modes)
            self.last_mode_change_time = current_time
            rospy.loginfo(f"Mode changed to: {self.mode_names[self.current_mode_index]}")
        
        mode = self.modes[self.current_mode_index]
        
        # ===== DRIVE SPEED CALCULATION =====
        vel_linear_x = 0
        vel_angular_z = 0
        
        if mode == 1:   # Slow mode
            vel_linear_x = int(64 * (RT - LT) / 2)
            vel_angular_z = int(64 * L_Analog_X)
        elif mode == 2: # Medium mode
            vel_linear_x = int(127 * (RT - LT) / 2)
            vel_angular_z = int(127 * L_Analog_X)
        elif mode == 3: # Fast mode
            vel_linear_x = int(191 * (RT - LT) / 2)
            vel_angular_z = int(191 * L_Analog_X)
        elif mode == 4: # Max mode
            vel_linear_x = int(255 * (RT - LT) / 2)
            vel_angular_z = int(255 * L_Analog_X)
        
        # ===== SERVO SPEED CALCULATION (INCREMENTAL) =====
        # Incremental step-based control (edge detection on D-pad)
        if D_X < 0 and self.last_dpad_state >= 0:  # D-pad Left pressed (new press)
            # Decrease speed (more CCW)
            self.servo_speed -= self.step_size_us
            rospy.loginfo(f"Servo CCW → {self.servo_speed}μs")
        elif D_X > 0 and self.last_dpad_state <= 0:  # D-pad Right pressed (new press)
            # Increase speed (more CW)
            self.servo_speed += self.step_size_us
            rospy.loginfo(f"Servo CW → {self.servo_speed}μs")
        
        # Update D-pad state for next iteration
        self.last_dpad_state = D_X
        
        # Constrain servo speed to valid microsecond range
        self.servo_speed = max(1000, min(2000, self.servo_speed))
        
        # ===== CREATE AND PUBLISH MESSAGE =====
        cmd = drive_servo()
        cmd.mode = mode
        cmd.vel_linear_x = vel_linear_x
        cmd.vel_angular_z = vel_angular_z
        cmd.servo_speed = self.servo_speed
        
        self.command_pub.publish(cmd)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = DriveServoControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

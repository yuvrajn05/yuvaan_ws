#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import dual_servo

class DualServoControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('dual_servo_control_node', anonymous=True)
        
        # Publisher for dual servo commands
        self.command_pub = rospy.Publisher('dual_servo_command', dual_servo, queue_size=10)
        
        # Subscriber to joystick
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # Servo control parameters (for continuous rotation servos)
        self.servo1_speed = 1500  # Servo 1 (Right stick X)
        self.servo2_speed = 1500  # Servo 2 (Right stick Y)
        
        # Store last joystick state for continuous publishing
        self.last_joy_msg = None
        
        # Timer for continuous publishing (20 Hz)
        self.publish_rate = 20  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Dual Servo Control Node Started")
        rospy.loginfo("=" * 60)
        rospy.loginfo("SERVO 1 (Right Stick X - Horizontal):")
        rospy.loginfo("  Push Left → CCW (proportional speed)")
        rospy.loginfo("  Push Right → CW (proportional speed)")
        rospy.loginfo("  Center → STOP")
        rospy.loginfo("")
        rospy.loginfo("SERVO 2 (Right Stick Y - Vertical):")
        rospy.loginfo("  Push Up → CW (proportional speed)")
        rospy.loginfo("  Push Down → CCW (proportional speed)")
        rospy.loginfo("  Center → STOP")
        rospy.loginfo("")
        rospy.loginfo("Speed Range: 1000-2000μs (1500=stop)")
        rospy.loginfo("Control: PROPORTIONAL (stick position = speed)")
        rospy.loginfo(f"Publishing at {self.publish_rate} Hz (continuous)")
        rospy.loginfo("=" * 60)
    
    def joy_callback(self, msg):
        """Store the latest joystick state"""
        self.last_joy_msg = msg
    
    def timer_callback(self, event):
        """
        Called continuously at 20 Hz
        Controls both servos based on right analog stick
        """
        if self.last_joy_msg is None:
            return  # No joystick data yet
        
        msg = self.last_joy_msg
        
        # Right analog stick mapping
        R_Analog_X = msg.axes[3]  # Right stick X (left/right) -1.0 to +1.0
        R_Analog_Y = msg.axes[4]  # Right stick Y (up/down) -1.0 to +1.0
        
        # Deadzone threshold to prevent drift
        deadzone = 0.1
        
        # ===== SERVO 1 CONTROL (Right Stick X) - PROPORTIONAL =====
        if abs(R_Analog_X) < deadzone:
            # Stick centered - STOP
            self.servo1_speed = 1500
        else:
            # Map stick value (-1.0 to +1.0) to servo speed (1000 to 2000μs)
            # -1.0 (full left) → 1000μs (full CCW)
            # +1.0 (full right) → 2000μs (full CW)
            self.servo1_speed = int(1500 + (500 * R_Analog_X))
        
        # ===== SERVO 2 CONTROL (Right Stick Y) - PROPORTIONAL =====
        if abs(R_Analog_Y) < deadzone:
            # Stick centered - STOP
            self.servo2_speed = 1500
        else:
            # Map stick value (-1.0 to +1.0) to servo speed (1000 to 2000μs)
            # -1.0 (full down) → 1000μs (full CCW)
            # +1.0 (full up) → 2000μs (full CW)
            self.servo2_speed = int(1500 + (500 * R_Analog_Y))
        
        # Constrain to valid microsecond range
        self.servo1_speed = max(1000, min(2000, self.servo1_speed))
        self.servo2_speed = max(1000, min(2000, self.servo2_speed))
        
        # ===== CREATE AND PUBLISH MESSAGE =====
        cmd = dual_servo()
        cmd.servo1_speed = self.servo1_speed
        cmd.servo2_speed = self.servo2_speed
        
        self.command_pub.publish(cmd)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = DualServoControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

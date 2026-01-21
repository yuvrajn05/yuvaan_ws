#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16

class ServoJoyControl:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('servo_joy_control', anonymous=True)
        
        # Publisher for servo position
        self.servo_pub = rospy.Publisher('servo_command', Int16, queue_size=10)
        
        # Subscriber to joystick
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # Servo speed control (for continuous rotation servo)
        # 90 = stop, <90 = CCW, >90 = CW
        self.servo_speed = 90  # Start at stop
        
        # Speed parameters
        self.max_speed = 90  # Max offset from center (0-90)
        self.acceleration = 5  # Speed change per update
        self.rate = rospy.Rate(50)  # 50 Hz update rate
        
        rospy.loginfo("360° Servo Joy Control Node Started")
        rospy.loginfo("Use D-pad X axis to control servo:")
        rospy.loginfo("  D-pad Left = CCW rotation")
        rospy.loginfo("  D-pad Right = CW rotation")
        rospy.loginfo("  Release = Stop")
    
    def joy_callback(self, msg):
        """
        Callback function for joystick input
        D-pad X axis is typically msg.axes[6]
        For 360° servo: 90 = stop, <90 = CCW, >90 = CW
        """
        # Get D-pad X value (typically axes[6])
        dpad_x = msg.axes[6]
        
        # Map D-pad input to servo speed
        if dpad_x < 0:  # D-pad Left - CCW rotation
            # Map to 0-89 range (full CCW speed at 0)
            self.servo_speed = int(90 + (self.max_speed * dpad_x))
            rospy.loginfo(f"Rotating CCW → Speed: {self.servo_speed}")
        elif dpad_x > 0:  # D-pad Right - CW rotation
            # Map to 91-180 range (full CW speed at 180)
            self.servo_speed = int(90 + (self.max_speed * dpad_x))
            rospy.loginfo(f"Rotating CW → Speed: {self.servo_speed}")
        else:  # D-pad released - Stop
            self.servo_speed = 90
            rospy.loginfo("Stopped")
        
        # Constrain to valid range (0-180)
        self.servo_speed = max(0, min(180, self.servo_speed))
        
        # Publish servo command
        self.servo_pub.publish(self.servo_speed)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = ServoJoyControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass

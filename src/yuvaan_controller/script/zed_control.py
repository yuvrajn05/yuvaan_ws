#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from yuvaan_controller.msg import yuvaan

def joy_callback(msg):

    speed = int(255*msg.data)

    motorspeed = yuvaan()

    motorspeed.vel_linear_x = speed
    motorspeed.vel_angular_z = speed
    motorspeed.base = speed
    motorspeed.gripper = speed
    motorspeed.ra_middle = speed
    motorspeed.ra_bottom = speed
    motorspeed.mode = speed

    # Publish motor control command
    motor_command_pub.publish(motorspeed)

if __name__ == '__main__':
    rospy.init_node('zed_control_node')
    joy_sub = rospy.Subscriber('direction', Int32, joy_callback)
    motor_command_pub = rospy.Publisher('motor_command', yuvaan, queue_size=10)
    rospy.spin()
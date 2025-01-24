#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import yuvaan

def joy_callback(msg):
    
    y_axis1 = -msg.axes[5]
    y_axis2 = -msg.axes[2]

    y_velocity_linear = int(255*(y_axis1 - y_axis2)/2)

    z_axis = msg.axes[0]

    motor_speed_a = int(255*z_axis)

    linear_y = int(255 * (y_axis1 - y_axis2) / 2)
    angular_z = int(-255 * msg.axes[0])
    base = int(255 * (msg.buttons[5] - msg.buttons[4]) / 2)
    RA_middle = int(255*msg.axes[4])
    RA_bottom = int(255*(msg.buttons[5]-msg.buttons[4]))
    gripper = int(255 * (msg.buttons[2] - msg.buttons[3]))
    point = 5

    flag = 0
    f = 0

    if msg.buttons[1] == 1:
            if not f:
                flag = not flag
                f = 1
    else:
        f = 0

    mode = 0 if flag else 1

    if mode :
        RA_bottom = linear_y
        linear_y = int(255*(msg.axes[1]))

    values = [linear_y, angular_z, base, gripper, point, mode]

    motorspeed = yuvaan()

    motorspeed.vel_linear_x = linear_y
    motorspeed.vel_angular_z = angular_z
    motorspeed.base = base
    motorspeed.gripper = gripper
    motorspeed.ra_middle = RA_middle
    motorspeed.ra_bottom = RA_bottom
    motorspeed.mode = mode

    # Publish motor control command
    motor_command_pub.publish(motorspeed)

if __name__ == '__main__':
    rospy.init_node('motor_control_node')
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    motor_command_pub = rospy.Publisher('motor_command', yuvaan, queue_size=10)
    rospy.spin()
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import yuvaan

import math

def joy_callback(msg):
    global current_mode_index  # Use the global variable

    LT = -msg.axes[2]
    RT = -msg.axes[5]
    LB = msg.buttons[4]
    RB = msg.buttons[5]
    L_Analog_Y = msg.axes[1]
    L_Analog_X = -msg.axes[0]
    R_Analog_Y = msg.axes[4]
    R_Analog_X = msg.axes[3]
    D_Y = msg.axes[7]
    D_X = msg.axes[6]
    Y = msg.buttons[3]
    X = msg.buttons[2]
    B = msg.buttons[1]
    A = msg.buttons[0]
    START = msg.buttons[7]
    BACK = msg.buttons[6]
    LOGITECH = msg.buttons[8]

    omega_max = 255 #rad/sec
    theta_max = math.pi/4 #rad
 
    b = 0.45 #in meter
    l = 1 #in meter

    omega = int(255 * (RT - LT) / 2)
    theta = int(255 * L_Analog_X)

    # thetaR = (math.pi/2 - math.atan(1/math.tan(theta)-(b/l)))
    # thetaL = (math.pi/2 - math.atan(1/math.tan(theta)+(b/l)))

    # omegaR = ((omega*(math.sin(theta)))/(math.sin(thetaR)))
    # omegaL = ((omega*(math.sin(theta)))/(math.sin(thetaL)))

    motorspeed = yuvaan()
    motorspeed.vel_linear_x = omega
    motorspeed.vel_angular_z = theta
    motorspeed.ra_1 = omega
    motorspeed.ra_2 = omega
    motorspeed.ba_1 = theta
    motorspeed.ba_2 = theta



    # Publish motor control command
    motor_command_pub.publish(motorspeed)

if __name__ == '__main__':
    rospy.init_node('motor_control_node')
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    motor_command_pub = rospy.Publisher('omega_pub', yuvaan, queue_size=10)
    rospy.spin()

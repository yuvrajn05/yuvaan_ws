#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import drive, mani

# ---------------- DRIVE MODES ----------------
current_mode_index = 0
modes = [1, 2, 3, 4]

# ---------------- MANIPULATOR MODES ----------------
yaw_mode_index = 0
yaw_modes = [30, 60, 80]

roll_mode_index = 0
roll_modes = [30, 60, 80]


def joy_callback(msg):
    global current_mode_index
    global yaw_mode_index
    global roll_mode_index

    # ---------------- AXES ----------------
    LT = -msg.axes[2]
    RT = -msg.axes[5]
    L_Analog_X = -msg.axes[0]
    L_Analog_Y = msg.axes[1]
    R_Analog_X = msg.axes[3]
    R_Analog_Y = msg.axes[4]
    D_X = msg.axes[6]
    D_Y = msg.axes[7]

    # ---------------- BUTTONS ----------------
    A = msg.buttons[0]
    B = msg.buttons[1]
    LB = msg.buttons[4]
    RB = msg.buttons[5]
    BACK = msg.buttons[6]

    # ---------------- MODE SWITCHING ----------------
    if BACK == 1:
        current_mode_index = (current_mode_index + 1) % len(modes)

    mode = modes[current_mode_index]

    if A == 1:
        yaw_mode_index = (yaw_mode_index + 1) % len(yaw_modes)

    if B == 1:
        roll_mode_index = (roll_mode_index + 1) % len(roll_modes)

    yaw_mode = yaw_modes[yaw_mode_index]
    roll_mode = roll_modes[roll_mode_index]

    # ---------------- DRIVE COMPUTATION ----------------
    vel_linear_x = 0
    vel_angular_z = 0

    if mode == 1:
        vel_linear_x = int(97 * (RT - LT) / 2)
        vel_angular_z = int(97 * L_Analog_X)

    elif mode == 2:
        vel_linear_x = int(127 * (RT - LT) / 2)
        vel_angular_z = int(127 * L_Analog_X)

    elif mode == 3:
        vel_linear_x = int(97 * D_Y)
        vel_angular_z = int(97 * D_X)

    elif mode == 4:
        vel_linear_x = int(255 * (RT - LT) / 2)
        vel_angular_z = int(255 * L_Analog_X)

    # ---------------- MANIPULATOR COMPUTATION ----------------
    BASE = SHOULDER = ELBOW = ROLL = PITCH = GRIPPER = 0

    if mode == 3:
        BASE = int(255 * L_Analog_X)
        SHOULDER = int(255 * L_Analog_Y)
        ELBOW = int(255 * R_Analog_Y)
        ROLL = int(-roll_mode * R_Analog_X)
        PITCH = int(yaw_mode * (RT - LT) / 2)
        GRIPPER = int(127 * (RB - LB))

    # ---------------- PUBLISH DRIVE ----------------
    drive_msg = drive()
    drive_msg.mode = mode
    drive_msg.vel_linear_x = vel_linear_x
    drive_msg.vel_angular_z = vel_angular_z
    drive_pub.publish(drive_msg)

    # ---------------- PUBLISH MANIPULATOR ----------------
    mani_msg = mani()
    mani_msg.roll_mode = roll_mode
    mani_msg.yaw_mode = yaw_mode
    mani_msg.ra_1 = BASE
    mani_msg.ra_2 = SHOULDER
    mani_msg.ra_3 = ELBOW
    mani_msg.ra_4 = ROLL
    mani_msg.ra_5 = PITCH
    mani_msg.ra_6 = GRIPPER
    mani_pub.publish(mani_msg)


if __name__ == '__main__':
    rospy.init_node('teleop_drive_mani_split')

    drive_pub = rospy.Publisher(
        'drive_motor_command', drive, queue_size=10
    )
    mani_pub = rospy.Publisher(
        'mani_motor_command', mani, queue_size=10
    )

    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)

    rospy.loginfo("Teleop split node started | Manipulator active ONLY in mode 3")
    rospy.spin()

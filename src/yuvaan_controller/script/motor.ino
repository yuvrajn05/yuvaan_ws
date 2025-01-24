#include <MotorControl.h>
#include <Encoder.h>
#include <ros.h>
#include <yuvaan_controller/yuvaan.h> 
#include "CytronMotorDriver.h"

CytronMD RA_base_motor(PWM_DIR, 3, 4); // PWM 2 = Pin 9, DIR 2 = Pin 8.
CytronMD RA_shoulder_motor(PWM_DIR, 5, 6);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD RA_elbow_motor(PWM_DIR, 7, 8);  // PWM 1 = Pin 5, DIR 1 = Pin 6.
CytronMD RA_roll_motor(PWM_DIR, 9, 10);  // PWM 1 = Pin 5, DIR 1 = Pin 6.
CytronMD RA_yaw_motor(PWM_DIR, 11, 12);  // PWM 1 = Pin 5, DIR 1 = Pin 6.
CytronMD RA_gripper_motor(PWM_DIR, 13, 14);  // PWM 1 = Pin 5, DIR 1 = Pin 6.

ros::NodeHandle nh;

void joyCallback(const yuvaan_controller::yuvaan& msg) {
 
  int ra_1 = msg.ra_1;
  int ra_2 = msg.ra_2;
  int ra_3 = msg.ra_3;
	int ra_4 = msg.ra_4;
	int ra_5 = msg.ra_5;
	int ra_6 = msg.ra_6;

  RA_base_motor.setSpeed(ra_1);
  RA_shoulder_motor.setSpeed(ra_2);
	RA_elbow_motor.setSpeed(ra_3);
	RA_roll_motor.setSpeed(ra_4);
	RA_yaw_motor.setSpeed(ra_5);
	RA_gripper_motor.setSpeed(ra_6);

}

ros::Subscriber<yuvaan_controller::yuvaan> joy_sub("motor_command", joyCallback);

void setup() {

  nh.initNode();
  nh.subscribe(joy_sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
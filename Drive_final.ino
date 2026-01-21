#include <ros.h>
#include <yuvaan_controller/drive.h> 
#include "CytronMotorDriver.h"

CytronMD L_motor(PWM_DIR, 3, 2); 
CytronMD R_motor(PWM_DIR, 5, 4);

ros::NodeHandle nh;
unsigned long last_command_time = 0;  // Track last message received time
const unsigned long TIMEOUT_MS = 500; // Timeout duration in milliseconds

void joyCallback(const yuvaan_controller::drive& msg) {
  last_command_time = millis(); // Update last command time

  int left_speed = constrain(msg.vel_linear_x + msg.vel_angular_z, -255, 255);
  int right_speed = constrain(msg.vel_linear_x - msg.vel_angular_z, -255, 255);

  L_motor.setSpeed(left_speed);
  R_motor.setSpeed(right_speed);
}

ros::Subscriber<yuvaan_controller::drive> joy_sub("drive_motor_command", joyCallback);

void setup() {
  nh.initNode();
  nh.subscribe(joy_sub);
}

void loop() {
  nh.spinOnce();

  // Check for timeout
  if (millis() - last_command_time > TIMEOUT_MS) {
    // Stop all motors if no command received within timeout period
    L_motor.setSpeed(0);
    R_motor.setSpeed(0);
  }

  delay(5); // Reduce loop delay for better responsiveness
}

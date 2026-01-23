/*
 * Unified Controller for ROS
 * Controls: 2x Drive Motors + 3x Servos
 * 
 * Hardware:
 * - Arduino Uno
 * - 2x Cytron Motor Drivers (differential drive)
 * - 3x 360° Continuous Rotation Servos
 * 
 * Pin Configuration:
 * MOTORS:
 *   Left Motor: PWM=3, DIR=2
 *   Right Motor: PWM=5, DIR=4
 * SERVOS:
 *   D-pad Servo: Pin 9
 *   Right Stick X Servo: Pin 10
 *   Right Stick Y Servo: Pin 11
 * 
 * ROS Topic: /unified_command (yuvaan_controller/unified_control)
 */

#include <ros.h>
#include <Servo.h>
#include <yuvaan_controller/unified_control.h>
#include "CytronMotorDriver.h"

// ===== MOTOR SETUP =====
CytronMD L_motor(PWM_DIR, 3, 2);  // Left motor
CytronMD R_motor(PWM_DIR, 5, 4);  // Right motor

// ===== SERVO SETUP =====
Servo servo_dpad;      // D-pad controlled
Servo servo_stick_x;   // Right stick X
Servo servo_stick_y;   // Right stick Y

const int SERVO_DPAD_PIN = 9;
const int SERVO_STICK_X_PIN = 10;
const int SERVO_STICK_Y_PIN = 11;

// ===== ROS SETUP =====
ros::NodeHandle nh;

// ===== SAFETY TIMEOUT =====
unsigned long last_command_time = 0;
const unsigned long TIMEOUT_MS = 500;

// ===== CALLBACK FUNCTION =====
void unifiedCallback(const yuvaan_controller::unified_control& msg) {
  // Update watchdog timer
  last_command_time = millis();
  
  // --- DRIVE CONTROL ---
  int left_speed = constrain(msg.vel_linear_x + msg.vel_angular_z, -255, 255);
  int right_speed = constrain(msg.vel_linear_x - msg.vel_angular_z, -255, 255);
  
  L_motor.setSpeed(left_speed);
  R_motor.setSpeed(right_speed);
  
  // --- SERVO CONTROL ---
  int dpad_servo = constrain(msg.servo_dpad, 1000, 2000);
  int stick_x_servo = constrain(msg.servo_stick_x, 1000, 2000);
  int stick_y_servo = constrain(msg.servo_stick_y, 1000, 2000);
  
  servo_dpad.writeMicroseconds(dpad_servo);
  servo_stick_x.writeMicroseconds(stick_x_servo);
  servo_stick_y.writeMicroseconds(stick_y_servo);
}

// ===== ROS SUBSCRIBER =====
ros::Subscriber<yuvaan_controller::unified_control> sub("unified_command", &unifiedCallback);

// ===== SETUP =====
void setup() {
  // Initialize ROS
  nh.initNode();
  nh.subscribe(sub);
  
  // Initialize servos
  servo_dpad.attach(SERVO_DPAD_PIN);
  servo_stick_x.attach(SERVO_STICK_X_PIN);
  servo_stick_y.attach(SERVO_STICK_Y_PIN);
  
  // Start everything at stop/neutral
  servo_dpad.writeMicroseconds(1500);
  servo_stick_x.writeMicroseconds(1500);
  servo_stick_y.writeMicroseconds(1500);
  
  delay(500);
  
  nh.loginfo("Unified Controller Ready! (Drive + 3 Servos)");
}

// ===== MAIN LOOP =====
void loop() {
  nh.spinOnce();
  
  // Safety timeout - stop everything
  if (millis() - last_command_time > TIMEOUT_MS) {
    L_motor.setSpeed(0);
    R_motor.setSpeed(0);
    servo_dpad.writeMicroseconds(1500);
    servo_stick_x.writeMicroseconds(1500);
    servo_stick_y.writeMicroseconds(1500);
  }
  
  delay(5);
}

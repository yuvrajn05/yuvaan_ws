/*
 * Unified Drive + 360° Servo Controller for ROS
 * 
 * Hardware:
 * - Arduino Uno
 * - 2x Cytron Motor Drivers (for differential drive)
 * - Robokits RKI 1201 360° Continuous Rotation Servo
 * 
 * Pin Configuration:
 * - Left Motor: PWM=3, DIR=2
 * - Right Motor: PWM=5, DIR=4
 * - Servo: Pin 9
 * 
 * ROS Topic: /drive_servo_command (yuvaan_controller/drive_servo)
 */

#include <ros.h>
#include <Servo.h>
#include <yuvaan_controller/drive_servo.h>
#include "CytronMotorDriver.h"

// ===== MOTOR SETUP =====
CytronMD L_motor(PWM_DIR, 3, 2);  // Left motor
CytronMD R_motor(PWM_DIR, 5, 4);  // Right motor

// ===== SERVO SETUP =====
Servo rotation_servo;
const int SERVO_PIN = 9;

// ===== ROS SETUP =====
ros::NodeHandle nh;

// ===== SAFETY TIMEOUT =====
unsigned long last_command_time = 0;
const unsigned long TIMEOUT_MS = 500;  // 500ms timeout

// ===== CALLBACK FUNCTION =====
void driveServoCallback(const yuvaan_controller::drive_servo& msg) {
  // Update watchdog timer
  last_command_time = millis();
  
  // --- DRIVE CONTROL ---
  // Differential drive: mix linear and angular velocities
  int left_speed = constrain(msg.vel_linear_x + msg.vel_angular_z, -255, 255);
  int right_speed = constrain(msg.vel_linear_x - msg.vel_angular_z, -255, 255);
  
  L_motor.setSpeed(left_speed);
  R_motor.setSpeed(right_speed);
  
  // --- SERVO CONTROL ---
  // Use microseconds for precise speed control
  int servo_speed = constrain(msg.servo_speed, 1000, 2000);
  rotation_servo.writeMicroseconds(servo_speed);
  
  // Optional: Debug logging
  #ifdef DEBUG_MODE
  char log_msg[80];
  sprintf(log_msg, "Drive L:%d R:%d | Servo:%dus", left_speed, right_speed, servo_speed);
  nh.loginfo(log_msg);
  #endif
}

// ===== ROS SUBSCRIBER =====
ros::Subscriber<yuvaan_controller::drive_servo> sub("drive_servo_command", &driveServoCallback);

// ===== SETUP =====
void setup() {
  // Initialize ROS
  nh.initNode();
  nh.subscribe(sub);
  
  // Initialize servo
  rotation_servo.attach(SERVO_PIN);
  rotation_servo.writeMicroseconds(1500);  // Start at stop position
  
  // Small delay for initialization
  delay(500);
  
  nh.loginfo("Drive + Servo Controller Ready!");
}

// ===== MAIN LOOP =====
void loop() {
  // Handle ROS communication
  nh.spinOnce();
  
  // Safety timeout - stop everything if no commands received
  if (millis() - last_command_time > TIMEOUT_MS) {
    L_motor.setSpeed(0);
    R_motor.setSpeed(0);
    rotation_servo.writeMicroseconds(1500);  // Stop servo
  }
  
  // Small delay for stability
  delay(5);
}

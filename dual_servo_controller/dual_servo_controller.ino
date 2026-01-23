/*
 * Dual Servo Controller for ROS
 * Controls 2x 360° Continuous Rotation Servos
 * 
 * Hardware:
 * - Arduino Uno
 * - 2x 360° Continuous Rotation Servos
 * 
 * Pin Configuration:
 * - Servo 1: Pin 9 (Right Stick X - horizontal)
 * - Servo 2: Pin 10 (Right Stick Y - vertical)
 * 
 * ROS Topic: /dual_servo_command (yuvaan_controller/dual_servo)
 */

#include <ros.h>
#include <Servo.h>
#include <yuvaan_controller/dual_servo.h>

// ===== SERVO SETUP =====
Servo servo1;  // Right stick X (horizontal)
Servo servo2;  // Right stick Y (vertical)

const int SERVO1_PIN = 9;
const int SERVO2_PIN = 10;

// ===== ROS SETUP =====
ros::NodeHandle nh;

// ===== SAFETY TIMEOUT =====
unsigned long last_command_time = 0;
const unsigned long TIMEOUT_MS = 500;  // 500ms timeout

// ===== CALLBACK FUNCTION =====
void dualServoCallback(const yuvaan_controller::dual_servo& msg) {
  // Update watchdog timer
  last_command_time = millis();
  
  // --- SERVO 1 CONTROL ---
  int servo1_speed = constrain(msg.servo1_speed, 1000, 2000);
  servo1.writeMicroseconds(servo1_speed);
  
  // --- SERVO 2 CONTROL ---
  int servo2_speed = constrain(msg.servo2_speed, 1000, 2000);
  servo2.writeMicroseconds(servo2_speed);
  
  // Optional: Debug logging
  #ifdef DEBUG_MODE
  char log_msg[60];
  sprintf(log_msg, "S1:%dus S2:%dus", servo1_speed, servo2_speed);
  nh.loginfo(log_msg);
  #endif
}

// ===== ROS SUBSCRIBER =====
ros::Subscriber<yuvaan_controller::dual_servo> sub("dual_servo_command", &dualServoCallback);

// ===== SETUP =====
void setup() {
  // Initialize ROS
  nh.initNode();
  nh.subscribe(sub);
  
  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Start at stop position
  servo1.writeMicroseconds(1500);
  servo2.writeMicroseconds(1500);
  
  // Small delay for initialization
  delay(500);
  
  nh.loginfo("Dual Servo Controller Ready!");
}

// ===== MAIN LOOP =====
void loop() {
  // Handle ROS communication
  nh.spinOnce();
  
  // Safety timeout - stop both servos if no commands received
  if (millis() - last_command_time > TIMEOUT_MS) {
    servo1.writeMicroseconds(1500);  // Stop servo 1
    servo2.writeMicroseconds(1500);  // Stop servo 2
  }
  
  // Small delay for stability
  delay(5);
}

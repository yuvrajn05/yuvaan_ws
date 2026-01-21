/*
 * Arduino 360° Continuous Rotation Servo Controller for ROS
 * Controls Robokits RKI 1201 metal gear 360° rotation servo
 * Receives velocity commands via ROS (rosserial)
 * 
 * Hardware:
 * - Arduino Uno
 * - Robokits RKI 1201 360° Continuous Rotation Servo
 * - Servo connected to Pin 9
 * 
 * ROS Topic: /servo_command (std_msgs/Int16)
 * 
 * Servo Control:
 * - Value 90  = STOP
 * - Value 0-89  = CCW rotation (0 = full speed CCW)
 * - Value 91-180 = CW rotation (180 = full speed CW)
 */

#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>

// Create servo object
Servo myServo;

// Pin definition
const int SERVO_PIN = 9;

// Current servo speed (90 = stop)
int currentSpeed = 90;

// ROS node handle
ros::NodeHandle nh;

// Callback function for servo command
void servoCallback(const std_msgs::Int16& cmd_msg) {
  // Get commanded speed (0-180, where 90 = stop)
  int targetSpeed = cmd_msg.data;
  
  // Constrain to valid servo range (0-180)
  targetSpeed = constrain(targetSpeed, 0, 180);
  
  // Update speed
  currentSpeed = targetSpeed;
  
  // Control servo rotation
  myServo.write(currentSpeed);
  
  // Optional: Log to ROS (for debugging)
  char log_msg[50];
  if (targetSpeed < 90) {
    sprintf(log_msg, "Servo CCW speed: %d", 90 - targetSpeed);
  } else if (targetSpeed > 90) {
    sprintf(log_msg, "Servo CW speed: %d", targetSpeed - 90);
  } else {
    sprintf(log_msg, "Servo STOPPED");
  }
  nh.loginfo(log_msg);
}

// ROS Subscriber
ros::Subscriber<std_msgs::Int16> sub("servo_command", &servoCallback);

void setup() {
  // Initialize ROS node
  nh.initNode();
  nh.subscribe(sub);
  
  // Attach servo to pin
  myServo.attach(SERVO_PIN);
  
  // Set initial speed to STOP (90 = stopped)
  myServo.write(currentSpeed);
  
  // Small delay for initialization
  delay(500);
  
  nh.loginfo("Arduino 360° Servo Controller Ready");
}

void loop() {
  // Handle ROS communication
  nh.spinOnce();
  
  // Small delay for stability
  delay(10);
}

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from yuvaan_controller.msg import unified_control, mani

class MergedController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('merged_control_node', anonymous=True)
        
        # Publishers for both modes
        self.unified_pub = rospy.Publisher('unified_command', unified_control, queue_size=10)
        self.mani_pub = rospy.Publisher('mani_motor_command', mani, queue_size=10)
        
        # Subscriber to joystick
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        # Mode management (0 = unified, 1 = mani)
        self.control_mode = 0  # Start with unified mode
        self.mode_names = ["UNIFIED", "MANI"]
        
        # === UNIFIED MODE STATE ===
        self.current_drive_mode_index = 0
        self.drive_modes = [1, 2, 3, 4]  # 1=slow, 2=medium, 3=fast, 4=max
        self.drive_mode_names = ["SLOW", "MEDIUM", "FAST", "MAX"]
        
        # Servo speeds (microseconds)
        self.servo_dpad = 1500
        self.servo_stick_x = 1500
        self.servo_stick_y = 1500
        self.dpad_speed = 250
        self.servo_hold_mode = False  # False = auto-center, True = hold position
        
        # === MANI MODE STATE ===
        self.yaw_mode_index = 0
        self.yaw_modes = [64, 127, 191, 255]  # 25%, 50%, 75%, 100%
        self.yaw_mode_names = ["25%", "50%", "75%", "100%"]
        self.roll_mode_index = 0
        self.roll_modes = [64, 127, 191, 255]  # 25%, 50%, 75%, 100%
        self.roll_mode_names = ["25%", "50%", "75%", "100%"]
        self.gripper_mode_index = 0
        self.gripper_modes = [64, 127, 191, 255]  # 25%, 50%, 75%, 100%
        self.gripper_mode_names = ["25%", "50%", "75%", "100%"]
        
        # Button debouncing
        self.last_mode_switch_time = 0
        self.last_drive_mode_change_time = 0
        self.last_servo_hold_change_time = 0
        self.last_yaw_mode_change_time = 0
        self.last_roll_mode_change_time = 0
        self.last_gripper_mode_change_time = 0
        self.debounce_delay = 0.3
        
        # Store last joystick state
        self.last_joy_msg = None
        
        # Timer for continuous publishing (20 Hz)
        self.publish_rate = 20
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        
        self.print_startup_message()
    
    def print_startup_message(self):
        rospy.loginfo("=" * 70)
        rospy.loginfo("MERGED CONTROL NODE STARTED")
        rospy.loginfo("=" * 70)
        rospy.loginfo("MODE SWITCHING:")
        rospy.loginfo("  BACK Button → Switch between UNIFIED and MANI modes")
        rospy.loginfo("")
        rospy.loginfo("UNIFIED MODE (Drive + 3 Servos):")
        rospy.loginfo("  RT/LT → Forward/Reverse")
        rospy.loginfo("  Left Stick X → Steering")
        rospy.loginfo("  Button A → Cycle Drive Modes (Slow/Medium/Fast/Max)")
        rospy.loginfo("  Button B → Toggle Servo Hold Mode (Auto-Center/Hold Position)")
        rospy.loginfo("  D-pad Left/Right → Servo 1 (Hold-to-Move)")
        rospy.loginfo("  Right Stick X → Servo 2 (Proportional)")
        rospy.loginfo("  Right Stick Y → Servo 3 (Proportional)")
        rospy.loginfo("")
        rospy.loginfo("MANI MODE (Manipulator Control):")
        rospy.loginfo("  Left Stick X → BASE")
        rospy.loginfo("  Left Stick Y → SHOULDER")
        rospy.loginfo("  Right Stick Y → ELBOW")
        rospy.loginfo("  Right Stick X → ROLL")
        rospy.loginfo("  RT/LT → PITCH")
        rospy.loginfo("  RB/LB → GRIPPER")
        rospy.loginfo("  Button Y → Cycle Yaw Modes (25%/50%/75%/100%)")
        rospy.loginfo("  Button B → Cycle Roll Modes (25%/50%/75%/100%)")
        rospy.loginfo("  Button X → Cycle Gripper Modes (25%/50%/75%/100%)")
        rospy.loginfo("")
        rospy.loginfo(f"Current Mode: {self.mode_names[self.control_mode]}")
        rospy.loginfo(f"Publishing at {self.publish_rate} Hz (continuous for both modes)")
        rospy.loginfo("=" * 70)
    
    def joy_callback(self, msg):
        """Store the latest joystick state"""
        self.last_joy_msg = msg
    
    def handle_mode_switching(self, BACK):
        """Handle switching between unified and mani modes"""
        current_time = rospy.get_time()
        if BACK == 1 and (current_time - self.last_mode_switch_time) > self.debounce_delay:
            self.control_mode = (self.control_mode + 1) % 2
            self.last_mode_switch_time = current_time
            rospy.loginfo("=" * 70)
            rospy.loginfo(f"SWITCHED TO: {self.mode_names[self.control_mode]} MODE")
            rospy.loginfo("=" * 70)
    
    def process_unified_mode(self, msg):
        """Process and publish unified control commands"""
        # Joystick mapping
        LT = -msg.axes[2]
        RT = -msg.axes[5]
        L_Analog_X = -msg.axes[0]
        R_Analog_X = msg.axes[3]
        R_Analog_Y = msg.axes[4]
        D_X = msg.axes[6]
        A = msg.buttons[0]
        B = msg.buttons[1]
        
        # Drive mode management
        current_time = rospy.get_time()
        if A == 1 and (current_time - self.last_drive_mode_change_time) > self.debounce_delay:
            self.current_drive_mode_index = (self.current_drive_mode_index + 1) % len(self.drive_modes)
            self.last_drive_mode_change_time = current_time
            rospy.loginfo(f"Drive Mode: {self.drive_mode_names[self.current_drive_mode_index]}")
        
        # Servo hold mode management
        if B == 1 and (current_time - self.last_servo_hold_change_time) > self.debounce_delay:
            self.servo_hold_mode = not self.servo_hold_mode
            self.last_servo_hold_change_time = current_time
            mode_str = "HOLD POSITION" if self.servo_hold_mode else "AUTO-CENTER"
            rospy.loginfo(f"Servo Mode: {mode_str}")
        
        mode = self.drive_modes[self.current_drive_mode_index]
        
        # Drive speed calculation
        if mode == 1:   # Slow
            vel_linear_x = int(64 * (RT - LT) / 2)
            vel_angular_z = int(64 * L_Analog_X)
        elif mode == 2: # Medium
            vel_linear_x = int(127 * (RT - LT) / 2)
            vel_angular_z = int(127 * L_Analog_X)
        elif mode == 3: # Fast
            vel_linear_x = int(191 * (RT - LT) / 2)
            vel_angular_z = int(191 * L_Analog_X)
        else:           # Max
            vel_linear_x = int(255 * (RT - LT) / 2)
            vel_angular_z = int(255 * L_Analog_X)
        
        # D-pad servo (Hold-to-Move) - REVERSED
        if D_X < 0:  # Left
            self.servo_dpad = 1500 + self.dpad_speed
        elif D_X > 0:  # Right
            self.servo_dpad = 1500 - self.dpad_speed
        else:  # Released
            self.servo_dpad = 1500
        
        # Right stick servos (Proportional)
        deadzone = 0.1
        
        if abs(R_Analog_X) < deadzone:
            if not self.servo_hold_mode:
                self.servo_stick_x = 1500  # Auto-center when released
            # else: hold last position
        else:
            self.servo_stick_x = int(1500 - (500 * R_Analog_X))  # REVERSED
        
        if abs(R_Analog_Y) < deadzone:
            if not self.servo_hold_mode:
                self.servo_stick_y = 1500  # Auto-center when released
            # else: hold last position
        else:
            self.servo_stick_y = int(1500 + (500 * R_Analog_Y))
        
        # Constrain all servos
        self.servo_dpad = max(1000, min(2000, self.servo_dpad))
        self.servo_stick_x = max(1000, min(2000, self.servo_stick_x))
        self.servo_stick_y = max(1000, min(2000, self.servo_stick_y))
        
        # Publish unified message
        cmd = unified_control()
        cmd.mode = mode
        cmd.vel_linear_x = vel_linear_x
        cmd.vel_angular_z = vel_angular_z
        cmd.servo_dpad = self.servo_dpad
        cmd.servo_stick_x = self.servo_stick_x
        cmd.servo_stick_y = self.servo_stick_y
        
        self.unified_pub.publish(cmd)
    
    def process_mani_mode(self, msg):
        """Process and publish mani control commands"""
        # Joystick mapping
        LT = -msg.axes[2]
        RT = -msg.axes[5]
        LB = msg.buttons[4]
        RB = msg.buttons[5]
        L_Analog_Y = msg.axes[1]
        L_Analog_X = -msg.axes[0]
        R_Analog_Y = msg.axes[4]
        R_Analog_X = msg.axes[3]
        Y = msg.buttons[3]  # Y button for yaw mode
        B = msg.buttons[1]  # B button for roll mode
        X = msg.buttons[2]  # X button for gripper mode
        
        # Yaw mode management
        current_time = rospy.get_time()
        if Y == 1 and (current_time - self.last_yaw_mode_change_time) > self.debounce_delay:
            self.yaw_mode_index = (self.yaw_mode_index + 1) % len(self.yaw_modes)
            self.last_yaw_mode_change_time = current_time
            rospy.loginfo(f"Yaw Mode: {self.yaw_mode_names[self.yaw_mode_index]} ({self.yaw_modes[self.yaw_mode_index]})")
        
        yaw_mode = self.yaw_modes[self.yaw_mode_index]
        
        # Roll mode management
        if B == 1 and (current_time - self.last_roll_mode_change_time) > self.debounce_delay:
            self.roll_mode_index = (self.roll_mode_index + 1) % len(self.roll_modes)
            self.last_roll_mode_change_time = current_time
            rospy.loginfo(f"Roll Mode: {self.roll_mode_names[self.roll_mode_index]} ({self.roll_modes[self.roll_mode_index]})")
        
        roll_mode = self.roll_modes[self.roll_mode_index]
        
        # Gripper mode management
        if X == 1 and (current_time - self.last_gripper_mode_change_time) > self.debounce_delay:
            self.gripper_mode_index = (self.gripper_mode_index + 1) % len(self.gripper_modes)
            self.last_gripper_mode_change_time = current_time
            rospy.loginfo(f"Gripper Mode: {self.gripper_mode_names[self.gripper_mode_index]} ({self.gripper_modes[self.gripper_mode_index]})")
        
        gripper_mode = self.gripper_modes[self.gripper_mode_index]
        
        # Calculate motor commands
        BASE = int(255 * L_Analog_X)
        SHOULDER = int(255 * L_Analog_Y)
        ELBOW = int(255 * R_Analog_Y)
        ROLL = int(-roll_mode * R_Analog_X)
        PITCH = int(yaw_mode * (RT - LT) / 2)
        GRIPPER = int(gripper_mode * (RB - LB))
        
        # Publish mani message
        motorspeed = mani()
        motorspeed.roll_mode = roll_mode
        motorspeed.yaw_mode = yaw_mode
        motorspeed.ra_1 = BASE
        motorspeed.ra_2 = SHOULDER
        motorspeed.ra_3 = ELBOW
        motorspeed.ra_4 = ROLL
        motorspeed.ra_5 = PITCH
        motorspeed.ra_6 = GRIPPER
        
        self.mani_pub.publish(motorspeed)
    
    def timer_callback(self, event):
        """Called continuously at 20 Hz"""
        if self.last_joy_msg is None:
            return
        
        msg = self.last_joy_msg
        
        # BACK button for mode switching
        BACK = msg.buttons[6]
        self.handle_mode_switching(BACK)
        
        # Process based on current mode
        if self.control_mode == 0:
            # UNIFIED MODE - only publish unified commands
            self.process_unified_mode(msg)
        else:
            # MANI MODE - only publish mani commands
            self.process_mani_mode(msg)
    
    def run(self):
        """Keep the node running"""
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MergedController()
        controller.run()
    except rospy.ROSInterruptException:
        pass

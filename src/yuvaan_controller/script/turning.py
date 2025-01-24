#!/usr/bin/env python3

import pyrealsense2 as rs
import pyzed.sl as sl
import numpy as np
import cv2
import math
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Float32
from cv_bridge import CvBridge

MATCH_THRESHOLD = 0.8
DEPTH_THRESHOLD = 2.0
TURN_THRESHOLD = 90.0

class IntegratedDetectionNode:
    def __init__(self):
        rospy.init_node('integrated_detection_node')
        
        # RealSense Pipeline for IMU
        self.rs_pipeline = rs.pipeline()
        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.accel)
        rs_config.enable_stream(rs.stream.gyro)
        self.rs_profile = self.rs_pipeline.start(rs_config)

        # ZED Camera and OpenCV Setup
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.coordinate_units = sl.UNIT.METER
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        
        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            rospy.logerr("Error opening ZED camera")
        
        # OpenCV Camera
        self.cap = cv2.VideoCapture(0)
        
        # Arrow Detection Setup
        right_arrow_path = rospy.get_param('~right_arrow_path', "/home/jetson/Downloads/right.png")
        left_arrow_path = rospy.get_param('~left_arrow_path', "/home/jetson/Downloads/left.png")
        
        self.right_arrow = cv2.imread(right_arrow_path, cv2.IMREAD_GRAYSCALE)
        self.left_arrow = cv2.imread(left_arrow_path, cv2.IMREAD_GRAYSCALE)
        
        # Publishers
        self.direction_pub = rospy.Publisher('arrow_direction', Int8, queue_size=10)
        self.depth_pub = rospy.Publisher('arrow_depth', Float32, queue_size=10)
        
        self.bridge = CvBridge()
        
        # Turn Detection Variables
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.prev_time = time.time()
        self.reference_yaw_degs = 0.0
        self.is_turning = False

    def complementary_filter_update(self, ax, ay, az, gx, gy, gz, dt, alpha=0.98):
        # Gyro integration
        roll_gyro  = self.roll  + gx * dt
        pitch_gyro = self.pitch + gy * dt
        yaw_gyro   = self.yaw   + gz * dt

        # Normalize accelerometer
        accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
        if accel_norm > 1e-6:
            axn = ax / accel_norm
            ayn = ay / accel_norm
            azn = az / accel_norm

            # Approximate pitch/roll from accel
            pitch_acc = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))
            roll_acc  = math.atan2(ayn, azn)

            # Complementary filter mix
            self.roll  = alpha * roll_gyro  + (1 - alpha) * roll_acc
            self.pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
            # Typically yaw is integrated only
            self.yaw   = yaw_gyro
        else:
            # If accel data is invalid, just trust gyro
            self.roll  = roll_gyro
            self.pitch = pitch_gyro
            self.yaw   = yaw_gyro

        return self.yaw

    def detect_turn(self):
        try:
            # Poll new frames for IMU
            frames = self.rs_pipeline.poll_for_frames()
            if not frames:
                return False

            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame  = frames.first_or_default(rs.stream.gyro)
            if not accel_frame or not gyro_frame:
                return False

            # Extract motion data
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            gyro_data  = gyro_frame.as_motion_frame().get_motion_data()
            
            # Compute delta time
            current_time = time.time()
            dt = current_time - self.prev_time
            self.prev_time = current_time

            # Update yaw
            yaw = self.complementary_filter_update(
                accel_data.x, accel_data.y, accel_data.z,
                gyro_data.x, gyro_data.y, gyro_data.z,
                dt, alpha=0.98
            )

            # Convert yaw to degrees
            yaw_degs = math.degrees(yaw)

            # Compare current yaw with reference to detect 90° turns
            delta = yaw_degs - self.reference_yaw_degs

            # Normalize the difference to (-180, 180]
            if delta > 180:
                delta -= 360
            elif delta <= -180:
                delta += 360

            # Check thresholds
            if abs(delta) >= TURN_THRESHOLD:
                if not self.is_turning:
                    self.is_turning = True
                    direction = "Left" if delta > 0 else "Right"
                    rospy.loginfo(f"Detected {direction} 90° turn")
                    self.reference_yaw_degs += 90 if delta > 0 else -90
                    return True
        
        except Exception as e:
            rospy.logerr(f"Error in turn detection: {e}")
        
        return False

    def log_edge_detection(self):
        rospy.loginfo("Performing edge detection...")

    def log_contour_detection(self):
        rospy.loginfo("Detecting contours...")

    def log_template_matching(self):
        rospy.loginfo("Performing template matching...")

    def edge_detection(self, image):
        self.log_edge_detection()
        edges = cv2.Canny(image, 50, 150)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        return edges

    def to_grayscale_and_blur(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        return blurred

    def detect_contours(self, image):
        self.log_contour_detection()
        processed = self.edge_detection(self.to_grayscale_and_blur(image))
        contours, _ = cv2.findContours(processed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def identify_arrow_tip(self, points, hull_indices):
        remaining_indices = [i for i in range(len(points)) if i not in hull_indices]

        for i in range(2):
            j = (remaining_indices[i] + 2) % len(points)
            if (points[j] == points[(remaining_indices[i - 1] - 2 + len(points)) % len(points)]).all():
                return points[j]
        return (-1, -1)

    def determine_direction(self, approx, tip):
        left_points = sum(1 for pt in approx if pt[0][0] < tip[0])
        right_points = sum(1 for pt in approx if pt[0][0] > tip[0])

        if left_points > right_points and left_points > 4:
            return -1  # Left
        if right_points > left_points and right_points > 4:
            return 1   # Right
        return 0       # None

    def calculate_angle(self, p1, p2):
        return math.degrees(math.atan2(p1[1] - p2[1], p1[0] - p2[0]))

    def process_frame(self, frame):
        contours = self.detect_contours(frame)
        for contour in contours:
            approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
            hull_indices = cv2.convexHull(approx, returnPoints=False)

            if (len(hull_indices) > 4 and len(hull_indices) < 6 and 
                len(hull_indices) + 2 == len(approx) and len(approx) > 6):
                
                tip = self.identify_arrow_tip(approx[:, 0], hull_indices.flatten())
                if tip[0] != -1 and tip[1] != -1:
                    direction = self.determine_direction(approx, tip)
                    if direction != 0:
                        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 3)
                        cv2.circle(frame, tip, 3, (0, 0, 255), -1)
                        return tip, direction
        return None, None

    def match_and_annotate(self, frame, template_img, color, label):
        self.log_template_matching()
        gray_frame = self.to_grayscale_and_blur(frame)
        best_value = -1
        best_location = (-1, -1)
        best_scale = -1

        for scale in np.arange(0.1, 0.5, 0.027):
            resized_template = cv2.resize(template_img, None, fx=scale, fy=scale)
            result = cv2.matchTemplate(gray_frame, resized_template, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, max_loc = cv2.minMaxLoc(result)

            if max_val > best_value and max_val > MATCH_THRESHOLD:
                best_value = max_val
                best_location = max_loc
                best_scale = scale

        if best_location != (-1, -1):
            w = int(template_img.shape[1] * best_scale)
            h = int(template_img.shape[0] * best_scale)
            top_left = best_location
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(frame, top_left, bottom_right, color, 2)

            frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
            angle = self.calculate_angle(top_left, frame_center)
            
            rospy.loginfo(f"{label} arrow detected at angle: {angle:.2f}")

    def get_frame_depth(self, runtime_params, frame_center):
        depth_map = sl.Mat()
        if self.zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            z_depth = depth_map.get_value(int(frame_center[0]), int(frame_center[1]))[1]
            return z_depth
        return None

    def process_detection(self):
        # Check for turning first
        if self.detect_turn():
            # If turning, don't process arrow detection
            return

        # Reset turning flag if no turn is in progress
        self.is_turning = False

        # Capture frame from OpenCV camera
        ret, frame = self.cap.read()
        if not ret or frame is None:
            rospy.logerr("Error capturing frame")
            return

        # Process frame for arrow detection
        frame_center, direction = self.process_frame(frame)

        if direction is not None and direction != 0:
            # Get depth using ZED camera
            runtime_params = sl.RuntimeParameters()
            z_depth = self.get_frame_depth(runtime_params, frame_center)
            
            # Publish only when depth is less than threshold
            if z_depth is not None and z_depth < DEPTH_THRESHOLD:
                # Publish Direction
                direction_msg = Int8()
                direction_msg.data = direction
                self.direction_pub.publish(direction_msg)
                
                # Publish Depth
                depth_msg = Float32()
                depth_msg.data = z_depth
                self.depth_pub.publish(depth_msg)
                
                rospy.loginfo(f"Depth at frame center: {z_depth:.2f} meters")

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            self.process_detection()
            rate.sleep()

    def __del__(self):
        # Cleanup
        self.rs_pipeline.stop()
        self.zed.close()
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = IntegratedDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def camera_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    # Set up publisher to the 'camera/image_raw' topic
    pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    # Initialize OpenCV and CvBridge
    print("debug1/")
    cap = cv2.VideoCapture(0)
    print("debug2")  # Use 0 for default camera
    bridge = CvBridge()
    
    if not cap.isOpened():
        rospy.logerr("Could not open the camera.")
        return
    
    rate = rospy.Rate(10)  # 10 Hz publishing rate
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to capture image")
            continue
        # Convert the OpenCV image to a ROS Image message
        image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # Publish the image message
        pub.publish(image_message)
        rate.sleep()
    
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass

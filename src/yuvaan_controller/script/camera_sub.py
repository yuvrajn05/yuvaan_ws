#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def image_callback(msg):
    # Convert the ROS Image message to OpenCV format
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # Display the image
    cv2.imshow("Camera Feed", frame)
    cv2.waitKey(1)  # Necessary for OpenCV to process the image

def camera_subscriber():
    rospy.init_node('camera_subscriber', anonymous=True)
    # Subscribe to the 'camera/image_raw' topic
    rospy.Subscriber('camera/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        camera_subscriber()
    except rospy.ROSInterruptException:
        pass
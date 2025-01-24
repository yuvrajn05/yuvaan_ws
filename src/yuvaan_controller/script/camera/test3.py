#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from flask import Flask, Response
import threading

app = Flask(__name__)
bridge = CvBridge()
frame = None  # Global variable to store the latest camera frame

def image_callback(msg):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")  # Update the global frame

def camera_subscriber():
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber('camera/image_raw', Image, image_callback)
    rospy.spin()

# Flask route for the MJPEG stream
@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Function to generate frames for MJPEG stream
def generate_frames():
    global frame
    while True:
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)  # Convert frame to JPEG
            frame_data = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')

if __name__ == '__main__':
    # Run camera subscriber in a separate thread
    threading.Thread(target=camera_subscriber).start()
    # Run the Flask app to provide the video feed
    app.run(host='0.0.0.0', port=5002)  # Use a different port than Flask SocketIO server

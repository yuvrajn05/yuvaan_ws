#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import numpy as np
from ultralytics import YOLO
import cv2
import time
import pyzed.sl as sl
import tensorflow as tf

# Load your models
model1 = YOLO("bestv8_1.pt")
filepath = "modelvgg.h5"
model2 = tf.keras.models.load_model(filepath)

def depth():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    zed.open(init_params)

    depth_ar = []
    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()
    depth_map = sl.Mat()

    try:
        while True:
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)

                img = image.get_data()[:, :, :3]
                results = model1(img)
                for result in results:
                    cord = result.boxes.xyxy
                    if len(cord) != 0:
                        x1, y1, x2, y2 = cord[0][:4]
                        (x, y) = (x2 + x1) / 2, (y2 + y1) / 2
                        z_depth = depth_map.get_value(int(x), int(y))[1]
                        depth_ar.append(z_depth)
                        if z_depth <= 2.1:
                            return z_depth

    finally:
        zed.close()
    return None

def detect():
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    zed.open(init_params)

    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()
    direction_count = {"left": 0, "right": 0}

    try:
        start_time = time.time()
        while time.time() - start_time < 5:  # Run for 5 seconds
            if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image, sl.VIEW.LEFT)
                img = image.get_data()[:, :, :3]
                img = cv2.resize(img, (224, 224))
                img = np.expand_dims(img, axis=0)

                output = model2.predict(img)
                dir = array2dir(output)
                if dir in direction_count:
                    direction_count[dir] += 1

    finally:
        zed.close()

    return max(direction_count, key=direction_count.get, default="none")

def array2dir(array):
    # none_prob = 0.1
    left_prob= array[0][0]
    right_prob = array[0][1]
    if left_prob > right_prob :
        return "left"
    elif right_prob > left_prob:
        return "right"
    return "none"

def main():
    rospy.init_node('depth_direction_node')
    pub = rospy.Publisher('direction', Int32, queue_size=10)

    while not rospy.is_shutdown():
        depth_value = depth()
        if depth_value is not None and depth_value <= 2.1:
            direction = detect()
            if direction == "left":
                print("left")
                pub.publish(-1)
            elif direction == "right":
                print("right")
                pub.publish(1)
            else:
                print("none")
                pub.publish(0)

if __name__ == '__main__':
    main()

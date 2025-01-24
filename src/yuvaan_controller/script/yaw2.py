import pyrealsense2 as rs
import numpy as np
import time
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable IMU streams
config.enable_stream(rs.stream.gyro)
config.enable_stream(rs.stream.accel)

# Start streaming
pipeline.start(config)
madgwick = Madgwick()  # Initialize Madgwick filter

# Quaternion for orientation tracking
q = np.array([1.0, 0.0, 0.0, 0.0])  

def get_imu_data():
    """Fetches the latest IMU data from the RealSense D435i."""
    frames = pipeline.wait_for_frames()
    gyro = frames.first_or_default(rs.stream.gyro)
    accel = frames.first_or_default(rs.stream.accel)

    if gyro and accel:
        gyro_data = gyro.as_motion_frame().get_motion_data()
        accel_data = accel.as_motion_frame().get_motion_data()
        return np.array([accel_data.x, accel_data.y, accel_data.z]), np.array([gyro_data.x, gyro_data.y, gyro_data.z])
    return None, None

try:
    prev_time = time.time()

    while True:
        accel, gyro = get_imu_data()
        if accel is None or gyro is None:
            continue

        # Calculate time delta
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        # Convert gyroscope from deg/s to rad/s
        gyro = np.radians(gyro)

        # Update orientation using Madgwick filter
        q = madgwick.updateIMU(q, gyr=gyro, acc=accel)

        # Convert quaternion to roll, pitch, yaw
        r = R.from_quat(q)  
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)

        print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}")

except KeyboardInterrupt:
    print("Stopping...")
finally:
    pipeline.stop()

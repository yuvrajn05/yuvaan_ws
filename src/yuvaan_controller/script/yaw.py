import pyrealsense2 as rs
import numpy as np
import time
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# 1) Setup pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.accel)
config.enable_stream(rs.stream.gyro)
profile = pipeline.start(config)

# 2) Complementary filter function
def complementary_filter_update(ax, ay, az, gx, gy, gz, dt, 
                                roll, pitch, yaw, alpha=0.98):
    # Gyro integration
    roll_gyro  = roll  + gx * dt
    pitch_gyro = pitch + gy * dt
    yaw_gyro   = yaw   + gz * dt

    # Accelerometer-based roll, pitch
    accel_norm = math.sqrt(ax*ax + ay*ay + az*az)
    if accel_norm > 1e-6:
        axn, ayn, azn = ax / accel_norm, ay / accel_norm, az / accel_norm
        
        pitch_acc = math.atan2(-axn, math.sqrt(ayn*ayn + azn*azn))
        roll_acc  = math.atan2(ayn, azn)
        
        roll  = alpha * roll_gyro  + (1.0 - alpha) * roll_acc
        pitch = alpha * pitch_gyro + (1.0 - alpha) * pitch_acc
        yaw   = yaw_gyro
    else:
        roll  = roll_gyro
        pitch = pitch_gyro
        yaw   = yaw_gyro
    
    return roll, pitch, yaw

# 3) Euler to Rotation matrix
def euler_to_rotation_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    Rz = np.array([[ cy, -sy,  0 ],
                   [ sy,  cy,  0 ],
                   [  0,   0,  1 ]])

    Ry = np.array([[ cp,  0, sp ],
                   [  0,  1,  0 ],
                   [-sp,  0, cp ]])

    Rx = np.array([[ 1,  0,  0 ],
                   [ 0, cr, -sr ],
                   [ 0, sr,  cr ]])

    R = Rz @ Ry @ Rx
    return R

# 4) Plot setup
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('RealSense D435i Orientation')

# Rotating axes (for the live orientation)
rot_lines = []
for i in range(3):
    line, = ax.plot([], [], [], lw=3)
    rot_lines.append(line)

# Reference axes
colors = ['r', 'g', 'b']
origin = np.array([0, 0, 0])
ref_axes = np.eye(3)*0.5
for i in range(3):
    ax.plot([origin[0], ref_axes[i,0]],
            [origin[1], ref_axes[i,1]],
            [origin[2], ref_axes[i,2]], 
            color=colors[i], linestyle='--', lw=1)

# 5) Global orientation variables
roll, pitch, yaw = 0.0, 0.0, 0.0
prev_time = time.time()

def update_plot(_):
    global roll, pitch, yaw, prev_time

    frames = pipeline.poll_for_frames()
    if not frames:
        return rot_lines

    accel_frame = frames.first_or_default(rs.stream.accel)
    gyro_frame  = frames.first_or_default(rs.stream.gyro)

    if accel_frame and gyro_frame:
        accel_data = accel_frame.as_motion_frame().get_motion_data()
        gyro_data  = gyro_frame.as_motion_frame().get_motion_data()

        axn = accel_data.x
        ayn = accel_data.y
        azn = accel_data.z
        gxn = gyro_data.x
        gyn = gyro_data.y
        gzn = gyro_data.z
        
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        
        roll, pitch, yaw = complementary_filter_update(
            axn, ayn, azn, 
            gxn, gyn, gzn, 
            dt, roll, pitch, yaw, alpha=0.98
        )
        
        R = euler_to_rotation_matrix(roll, pitch, yaw)
        local_axes = np.eye(3)*0.5
        rotated_axes = R @ local_axes
        
        for i in range(3):
            x_vals = [0, rotated_axes[0, i]]
            y_vals = [0, rotated_axes[1, i]]
            z_vals = [0, rotated_axes[2, i]]
            rot_lines[i].set_data(x_vals, y_vals)
            rot_lines[i].set_3d_properties(z_vals)
            rot_lines[i].set_color(colors[i])

    return rot_lines

ani = FuncAnimation(fig, update_plot, interval=50, blit=False)
plt.show()

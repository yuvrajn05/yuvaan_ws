# Drive and Servo Control Message
# Combines differential drive control with continuous rotation servo control

# Drive control
int32 vel_linear_x    # Linear velocity (-255 to 255)
int32 vel_angular_z   # Angular velocity (-255 to 255)
int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)

# 360° Servo control (microseconds for precision)
int32 servo_speed     # Servo speed in microseconds (1000-2000, 1500=stop)

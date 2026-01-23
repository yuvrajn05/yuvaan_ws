# Unified Control Message
# Combines drive control + 3 servos (1 D-pad + 2 analog stick)

# Drive control
int32 vel_linear_x    # Linear velocity (-255 to 255)
int32 vel_angular_z   # Angular velocity (-255 to 255)
int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)

# Servo control (all in microseconds, 1500=stop)
int32 servo_dpad      # D-pad controlled servo (1000-2000μs)
int32 servo_stick_x   # Right stick X servo (1000-2000μs)
int32 servo_stick_y   # Right stick Y servo (1000-2000μs)

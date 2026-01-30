#!/bin/bash
# Check hardware connections for Yuvaan robot

echo "=========================================="
echo "  Yuvaan Hardware Connection Check"
echo "=========================================="
echo ""

# Check joystick
echo "🕹️  Joystick:"
if [ -e /dev/input/js0 ]; then
    echo "   ✅ /dev/input/js0 - Found"
    ls -la /dev/input/js0 | awk '{print "      Permissions: " $1 " " $3 ":" $4}'
else
    echo "   ❌ /dev/input/js0 - Not found"
fi
echo ""

# Check serial devices
echo "🔌 Serial Devices:"
if ls /dev/ttyUSB* >/dev/null 2>&1; then
    for device in /dev/ttyUSB*; do
        echo "   ✅ $device - Found"
        ls -la $device | awk '{print "      Permissions: " $1 " " $3 ":" $4}'
        
        # Try to identify device
        if command -v udevadm &> /dev/null; then
            vendor=$(udevadm info -q property -n $device | grep ID_VENDOR_ID | cut -d'=' -f2)
            model=$(udevadm info -q property -n $device | grep ID_MODEL_ID | cut -d'=' -f2)
            if [ ! -z "$vendor" ]; then
                echo "      Device info: Vendor=$vendor, Model=$model"
            fi
        fi
    done
else
    echo "   ❌ No /dev/ttyUSB* devices found"
    echo "      Connect Arduino via USB cable"
fi
echo ""

# Check user groups
echo "👤 User Groups:"
if groups | grep -q dialout; then
    echo "   ✅ User is in 'dialout' group (can access serial ports)"
else
    echo "   ⚠️  User NOT in 'dialout' group"
    echo "      Run: sudo usermod -a -G dialout $USER"
    echo "      Then logout and login again"
fi
echo ""

# Check ROS
echo "🤖 ROS Environment:"
if [ ! -z "$ROS_DISTRO" ]; then
    echo "   ✅ ROS_DISTRO = $ROS_DISTRO"
    echo "   ✅ ROS_MASTER_URI = $ROS_MASTER_URI"
else
    echo "   ⚠️  ROS not sourced"
    echo "      Run: source /opt/ros/noetic/setup.bash"
fi
echo ""

# Suggest which group to use
echo "📋 Recommended Group:"
has_js=$([ -e /dev/input/js0 ] && echo "yes" || echo "no")
has_usb0=$([ -e /dev/ttyUSB0 ] && echo "yes" || echo "no")
has_usb1=$([ -e /dev/ttyUSB1 ] && echo "yes" || echo "no")

if [ "$has_usb0" = "yes" ] && [ "$has_usb1" = "yes" ]; then
    echo "   🎯 Use: drive_mani (both Arduinos detected)"
elif [ "$has_usb0" = "yes" ]; then
    echo "   🎯 Use: drive_only (one Arduino detected)"
else
    echo "   🎯 Use: no_hardware (no Arduinos detected)"
fi
echo ""
echo "=========================================="

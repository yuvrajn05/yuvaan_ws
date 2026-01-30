#!/bin/bash
# SSH Remote Mode Testing Script

echo "=========================================="
echo "  Yuvaan SSH Remote Mode Test"
echo "=========================================="
echo ""

WS_DIR="/home/neem/yuvaan_ws"
cd "$WS_DIR"

echo "📋 Test Plan:"
echo "  1. Verify Jetson connectivity"
echo "  2. Test SSH connection"
echo "  3. Start minimal group in remote mode"
echo "  4. Verify components run on Jetson"
echo "  5. Monitor and stop components"
echo ""

# Test 1: Network
echo "Test 1: Checking Jetson connectivity..."
if ping -c 2 192.168.2.101 > /dev/null 2>&1; then
    echo "  ✅ Jetson reachable at 192.168.2.101"
else
    echo "  ❌ Cannot reach Jetson"
    echo "  Fix: Check if Jetson is powered on and connected"
    exit 1
fi
echo ""

# Test 2: SSH
echo "Test 2: Testing SSH connection..."
if ssh -o ConnectTimeout=5 jetson@192.168.2.101 "echo SSH_OK" 2>/dev/null | grep -q "SSH_OK"; then
    echo "  ✅ SSH connection working"
else
    echo "  ⚠️  SSH connection failed"
    echo "  Running SSH setup..."
    ./yuvaan_start.sh --setup-ssh
fi
echo ""

# Test 3: Check Jetson ROS
echo "Test 3: Checking if ROS is available on Jetson..."
ROS_CHECK=$(ssh jetson@192.168.2.101 "source /opt/ros/noetic/setup.bash 2>&1 && echo ROS_OK" 2>&1 | tail -1)
if [[ "$ROS_CHECK" == "ROS_OK" ]]; then
    echo "  ✅ ROS available on Jetson"
else
    echo "  ⚠️  ROS may not be installed on Jetson"
    echo "  Output: $ROS_CHECK"
fi
echo ""

echo "=========================================="
echo " Ready to Test Remote Mode!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Run: ./yuvaan_start.sh --monitor"
echo "     (Remote mode is default)"
echo ""
echo "  2. From the menu:"
echo "     Option 5 → Start Group"
echo "     Choose 'no_hardware' (option 1)"
echo ""
echo "  3. In another terminal:"
echo "     tmux attach -t yuvaan_auto_monitor"
echo ""
echo "  4. Verify components on Jetson:"
echo "     ssh jetson@192.168.2.101"
echo "     ps aux | grep ros"
echo ""
echo "=========================================="

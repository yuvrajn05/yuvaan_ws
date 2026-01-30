#!/usr/bin/env python3
"""
Quick test script to verify remote execution works
"""
import sys
import time
sys.path.append('/home/neem/yuvaan_ws/src/yuvaan_controller/script')

from ssh_utils import SSHManager

print("=" * 50)
print("  Remote Execution Test")
print("=" * 50)
print()

# Initialize SSH manager
ssh = SSHManager(
    host="192.168.2.101",
    user="jetson",
    password="jetson"
)
ssh.workspace_path = "/home/jetson/yuvaan_ws"  # Set workspace path for ROS sourcing

print("Test 1: Verify ROS environment sourcing")
print("-" * 50)
returncode, stdout, stderr = ssh.execute_command("which roscore")
if returncode == 0:
    print(f"✅ roscore found at: {stdout.strip()}")
else:
    print(f"❌ roscore not found")
    print(f"   Error: {stderr}")

returncode, stdout, stderr = ssh.execute_command("echo $ROS_DISTRO")
if returncode == 0 and stdout.strip():
    print(f"✅ ROS_DISTRO: {stdout.strip()}")
else:
    print(f"⚠️  ROS_DISTRO not set")

print()
print("Test 2: Check if roscore is already running")
print("-" * 50)
returncode, stdout, stderr = ssh.execute_command("pgrep -f roscore")
if returncode == 0 and stdout.strip():
    print(f"⚠️  roscore already running (PID: {stdout.strip()})")
    print("   Stopping existing roscore...")
    ssh.execute_command("pkill -f roscore")
    time.sleep(2)
    print("   Stopped.")
else:
    print("✅ No roscore running (clean state)")

print()
print("Test 3: Start roscore in background")
print("-" * 50)
returncode, stdout, stderr = ssh.execute_command("roscore", background=True)
if returncode == 0:
    print("✅ roscore start command sent")
    print("   Waiting 3 seconds for startup...")
    time.sleep(3)
else:
    print(f"❌ Failed to start roscore")
    print(f"   Error: {stderr}")

print()
print("Test 4: Verify roscore is running")
print("-" * 50)
returncode, stdout, stderr = ssh.execute_command("pgrep -f roscore")
if returncode == 0 and stdout.strip():
    print(f"✅ roscore is running (PID: {stdout.strip()})")
else:
    print(f"❌ roscore is NOT running")
    if stderr:
        print(f"   Error: {stderr}")

print()
print("Test 5: Check ROS topics")
print("-" * 50)
returncode, stdout, stderr = ssh.execute_command("rostopic list")
if returncode == 0:
    print(f"✅ ROS topics available:")
    for line in stdout.strip().split('\n'):
        print(f"   - {line}")
else:
    print(f"❌ Failed to list topics")
    print(f"   Error: {stderr}")

print()
print("Test 6: Cleanup - Stop roscore")
print("-" * 50)
ssh.execute_command("pkill -f roscore")
time.sleep(2)
returncode, stdout, stderr = ssh.execute_command("pgrep -f roscore")
if returncode != 0 or not stdout.strip():
    print("✅ roscore stopped successfully")
else:
    print(f"⚠️  roscore still running (PID: {stdout.strip()})")

print()
print("=" * 50)
print("  Test Complete!")
print("=" * 50)

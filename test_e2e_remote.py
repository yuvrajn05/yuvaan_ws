#!/usr/bin/env python3
"""
End-to-end test for remote component startup and detection
"""
import sys
import time
import subprocess
sys.path.append('/home/neem/yuvaan_ws/src/yuvaan_controller/script')

from ssh_utils import SSHManager
from process_manager import ProcessManager, ManagedProcess
from pathlib import Path

print("="*60)
print("  End-to-End Remote Component Test")
print("="*60)
print()

# Setup
ssh = SSHManager('192.168.2.101', 'jetson', 'jetson')
ssh.workspace_path = "/home/jetson/yuvaan_ws"
log_dir = Path("/tmp/test_logs")
log_dir.mkdir(exist_ok=True)

pm = ProcessManager(log_dir=log_dir, ssh_manager=ssh)

# Clean up any existing processes
print("Cleaning up existing processes...")
ssh.execute_system_command("pkill -9 -f roscore")
ssh.execute_system_command("pkill -9 -f merged_control")
time.sleep(2)

# Test 1: Add roscore component
print("\nTest 1: Adding roscore component")
pm.add_process(
    name="roscore",
    command="roscore",
    location="jetson",
    dependencies=[]
)
print("✅ roscore added")

# Test 2: Start roscore
print("\nTest 2: Starting roscore")
success = pm.start_process("roscore")
if success:
    print("✅ roscore started successfully")
else:
    print("❌ roscore failed to start")
    
# Test 3: Check if running
print("\nTest 3: Checking if roscore is running")
is_running = pm.processes["roscore"].is_running()
print(f"  is_running() returned: {is_running}")

# Direct check
returncode, stdout, stderr = ssh.execute_system_command("pgrep roscore")
print(f"  Direct pgrep: returncode={returncode}, found PIDs: {stdout.strip()}")

if is_running:
    print("✅ roscore detected as running")
else:
    print("❌ roscore NOT detected (but may actually be running)")

# Test 4: Add merged_control
print("\nTest 4: Adding merged_control component")
pm.add_process(
    name="merged_control",
    command="rosrun yuvaan_controller merged_control.py",
    location="jetson",
    dependencies=["roscore"]
)
print("✅ merged_control added")

# Test 5: Start merged_control
print("\nTest 5: Starting merged_control")
success2 = pm.start_process("merged_control", startup_delay=1)
if success2:
    print("✅ merged_control started successfully")
else:
    print("❌ merged_control failed to start")

# Test 6: Check if running  
print("\nTest6: Checking if merged_control is running")
is_running2 = pm.processes["merged_control"].is_running()
print(f"  is_running() returned: {is_running2}")

# Direct check
returncode2, stdout2, stderr2 = ssh.execute_system_command("pgrep -af merged_control")
print(f"  Direct pgrep: returncode={returncode2}")
if stdout2:
    print(f"  Found processes:\n{stdout2}")

if is_running2:
    print("✅ merged_control detected as running")
else:
    print("❌ merged_control NOT detected")

# Cleanup
print("\nCleaning up...")
pm.stop_all()
time.sleep(2)

print("\n" + "="*60)
print("  Test Complete")
print("="*60)

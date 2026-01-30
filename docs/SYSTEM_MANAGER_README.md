# Yuvaan Robot System Manager

A comprehensive system management solution for the Yuvaan robot that provides component lifecycle control, health monitoring, logging, and rollback capabilities.

## Features

- **Component Management**: Start/stop/restart individual components or groups
- **Progressive System Groups**: Upgrade from simple testing to full system
- **Health Monitoring**: Continuous ping monitoring and process health checks
- **Comprehensive Logging**: All component output logged with rotation
- **State Management**: Save/restore system states and rollback to previous configurations
- **Multi-Environment**: Run on Jetson (remote) or laptop (local testing)
- **Automatic Dependencies**: Automatically resolves and starts component dependencies
- **Interactive Menu**: User-friendly text interface for all operations

## System Groups

The system supports progressive groups that can be upgraded incrementally:

1. **no_hardware** - Testing without hardware (simulation/echo only)
   - roscore, merged_control, joy_node, unified_echo

2. **drive_only** - Drive motors only (no manipulator)  
   - Adds: serial_node_main

3. **drive_mani** - Drive motors + Manipulator
   - Adds: mani_echo, serial_node_mani

4. **full_system** - Complete system with camera
   - Adds: video_stream

5. **monitoring** - Monitoring and debugging setup
   - ping_monitor, roscore, merged_control, unified_echo, mani_echo

## Installation

### Prerequisites

```bash
# Update package list
sudo apt update

# Install Python dependencies
pip3 install pyyaml colorama

# Install sshpass (for initial SSH key setup)
sudo apt install -y sshpass
```

### Setup

1. The configuration file is located at `config/yuvaan_config.yaml`
2. Review and modify network settings if needed:
   ```yaml
   network:
     jetson_ip: "192.168.2.101"
     jetson_user: "jetson"
     jetson_password: "jetson"
   ```

3. Make scripts executable (already done):
   ```bash
   chmod +x src/yuvaan_controller/script/yuvaan_manager.py
   ```

## Quick Start

### Interactive Mode (Recommended)

```bash
# Start the system manager
python3 src/yuvaan_controller/script/yuvaan_manager.py

# Or in local mode (run everything on laptop)
python3 src/yuvaan_controller/script/yuvaan_manager.py --mode local
```

On first run, the system will:
1. Check connectivity to Jetson
2. Setup SSH keys for passwordless authentication  
3. Display the interactive menu

### Command Line Mode

```bash
# Setup SSH keys only
python3 src/yuvaan_controller/script/yuvaan_manager.py --setup-ssh

# Show system status
python3 src/yuvaan_controller/script/yuvaan_manager.py --status

# Start a specific group
python3 src/yuvaan_controller/script/yuvaan_manager.py --start-group no_hardware

# Stop all components
python3 src/yuvaan_controller/script/yuvaan_manager.py --stop-all
```

### Using ROS Launch Files

```bash
# Start a group using ROS launch
roslaunch yuvaan_controller yuvaan_system.launch group:=no_hardware

# Start drive only system
roslaunch yuvaan_controller yuvaan_system.launch group:=drive_only

# Start full system
roslaunch yuvaan_controller yuvaan_system.launch group:=full_system
```

## Interactive Menu Guide

### System Control
- **Show System Status**: View running components, uptime, and connectivity
- **Start Component**: Start individual components
- **Stop Component**: Stop individual components (respects dependencies)
- **Restart Component**: Restart a component

### Group Management
- **Start Group**: Start all components in a predefined group
- **Stop Current Group**: Stop all components in active group
- **Upgrade to Next Group**: Incrementally add components (e.g., no_hardware → drive_only)

### Monitoring
- **View Component Logs**: View recent output from components
- **Monitor Ping**: Continuous connectivity monitoring to Jetson

### State Management
- **Save Current State as Preset**: Save current configuration for quick recall
- **Load Preset**: Restore a saved preset
- **Rollback to Previous State**: Undo recent changes

## Configuration File

The `config/yuvaan_config.yaml` file defines:

- Network settings (Jetson IP, credentials)
- Logging configuration
- Component definitions (commands, dependencies, health checks)
- System groups
- Execution modes

Example component definition:
```yaml
- name: "merged_control"
  description: "Unified and Manipulator Control Script"
  location: "jetson"
  command: "rosrun yuvaan_controller merged_control.py"
  type: "required"
  dependencies: ["roscore"]
  auto_restart: true
  startup_delay: 2
```

## Logging

All logs are stored in `~/yuvaan_logs/`:

```
~/yuvaan_logs/
├── yuvaan_manager.log          # Main system log
├── state/                      # State history
│   ├── current_state.json
│   ├── history/
│   └── presets.json
├── roscore_stdout.log         # Component logs
├── roscore_stderr.log
├── merged_control_stdout.log
└── ...
```

## Typical Workflow

### Starting System for Testing (No Hardware)

1. Run system manager: `python3 src/yuvaan_controller/script/yuvaan_manager.py`
2. Select option 5 (Start Group)
3. Select `no_hardware`
4. Components start automatically with dependency resolution
5. View status with option 1

### Upgrading to Full System

1. From `no_hardware` group, select option 7 (Upgrade to Next Group)
2. System upgrades to `drive_only` (adds serial_node_main)
3. Select option 7 again to upgrade to `drive_mani`
4. Select option 7 again to upgrade to `full_system`

### Monitoring and Debugging

1. Select option 8 to view component logs
2. Select option 9 to monitor Jetson connectivity
3. Use option 1 to check component status and uptime

### Saving and Restoring States

1. Configure your desired state
2. Select option 10 to save as preset (e.g., "testing_setup")
3. Later, select option 11 to quickly restore this configuration
4. Use option 12 to rollback if something goes wrong

## Troubleshooting

### SSH Connection Issues

```bash
# Test SSH connection
ssh jetson@192.168.2.101

# Setup SSH keys manually if automatic setup fails
ssh-keygen -t rsa -b 4096
ssh-copy-id jetson@192.168.2.101

# Test passwordless SSH
ssh -o BatchMode=yes jetson@192.168.2.101 echo "test"
```

### Component Won't Start

1. Check logs: View logs in interactive menu (option 8)
2. Verify dependencies: Ensure roscore and dependencies are running
3. Check device paths: For serial nodes, verify /dev/ttyUSB0 and /dev/ttyUSB1 exist
4. Test manually: Run the command directly to see errors

### Network Connectivity

```bash
# Check if Jetson is reachable
ping 192.168.2.101

# Check ROS network configuration on both machines
echo $ROS_MASTER_URI
echo $ROS_IP

# Set ROS network (on laptop)
export ROS_MASTER_URI=http://192.168.2.101:11311
export ROS_IP=$(hostname -I | awk '{print $1}')
```

### Serial Device Not Found

```bash
# On Jetson, list USB devices
ls -la /dev/ttyUSB*

# Check device permissions
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1

# Add user to dialout group (permanent fix)
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## Advanced Usage

### Creating Custom Groups

Edit `config/yuvaan_config.yaml`:

```yaml
groups:
  my_custom_group:
    description: "My custom configuration"
    components:
      - "roscore"
      - "merged_control"
      - "my_component"
```

### Adding New Components

Edit `config/yuvaan_config.yaml`:

```yaml
components:
  - name: "my_component"
    description: "My custom component"
    location: "jetson"  # or "laptop"
    command: "rosrun my_package my_node"
    type: "optional"  # or "required"
    dependencies: ["roscore"]
    auto_restart: false
    startup_delay: 1
```

### Running in Local Mode

Useful for testing when Jetson is not available:

```bash
python3 src/yuvaan_controller/script/yuvaan_manager.py --mode local
```

All components will run on the laptop instead of Jetson.

## Architecture

```
yuvaan_manager.py (Main Controller)
    ├── ssh_utils.py (SSH Connection Management)
    ├── process_manager.py (Process Lifecycle)
    └── state_manager.py (State Persistence)
```

### Key Components

- **YuvaanSystemManager**: Main orchestrator
- **SSHManager**: Handles SSH connections and remote execution
- **ProcessManager**: Manages local and remote processes
- **StateManager**: Handles state persistence and rollback

## Files Overview

```
yuvaan_ws/
├── config/
│   └── yuvaan_config.yaml           # Main configuration
├── src/yuvaan_controller/
│   ├── script/
│   │   ├── yuvaan_manager.py        # Main manager script
│   │   ├── ssh_utils.py             # SSH utilities
│   │   ├── process_manager.py        # Process management
│   │   ├── state_manager.py         # State management
│   │   └── commands.txt             # Original command reference
│   └── launch/
│       └── yuvaan_system.launch     # ROS launch file
├── docs/
│   └── SYSTEM_MANAGER_README.md     # This file
└── .agent/workflows/
    └── startup.md                    # Original workflow documentation
```

## Safety Features

- **Dependency Resolution**: Automatically starts dependencies in correct order
- **Graceful Shutdown**: Components stopped in reverse dependency order
- **State Rollback**: Can undo changes if something goes wrong
- **Automatic Restart**: Critical components restart on failure
- **Comprehensive Logging**: All output captured for debugging

## Tips

1. **First Time Setup**: Run `--setup-ssh` first to configure authentication
2. **Start Small**: Begin with `no_hardware` group for testing
3. **Save Presets**: Save working configurations as presets for quick access
4. **Monitor Logs**: Regularly check logs when debugging issues
5. **Use Rollback**: Don't hesitate to rollback if something breaks

## Support

For issues or questions:
1. Check logs in `~/yuvaan_logs/`
2. Review component status in interactive menu
3. Test individual components manually
4. Check the troubleshooting section above

---

**Version**: 1.0  
**Last Updated**: 2026-01-28

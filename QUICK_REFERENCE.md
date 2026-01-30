# Yuvaan Robot System - Quick Reference

## Quick Start Commands

```bash
# Start system (interactive mode)
./yuvaan_start.sh

# Setup SSH keys (first time)
./yuvaan_start.sh --setup-ssh

# Show status
./yuvaan_start.sh --status

# Run in local mode (all on laptop)
./yuvaan_start.sh --local

# Start specific group  
python3 src/yuvaan_controller/script/yuvaan_manager.py --start-group no_hardware

# View logs in real-time
./view_logs.sh

# Using ROS launch
roslaunch yuvaan_controller yuvaan_system.launch group:=drive_only
```

## System Groups

| Group | Components | Use Case |
|-------|-----------|----------|
| **no_hardware** | roscore, merged_control, joy_node, unified_echo | Testing without hardware |
| **drive_only** | no_hardware + serial_node_main | Drive motors only |
| **drive_mani** | drive_only + mani_echo, serial_node_mani | Drive + manipulator |
| **full_system** | drive_mani + video_stream | Complete system |
| **monitoring** | Core + all echo nodes | Debugging setup |

## Interactive Menu

```
1. Show System Status          8. View Component Logs
2. Start Component              9. Monitor Ping
3. Stop Component              10. Save Preset
4. Restart Component           11. Load Preset
5. Start Group                 12. Rollback
6. Stop Current Group           0. Exit
7. Upgrade to Next Group
```

## Component Startup Order

1. `roscore` (always first)
2. `merged_control` (depends on roscore)
3. `joy_node` (depends on roscore)
4. `serial_node_main`, `serial_node_mani` (depend on merged_control)
5. `video_stream` (independent)
6. Echo nodes for monitoring

## Log Locations

- Main log: `~/yuvaan_logs/yuvaan_manager.log`
- Component logs: `~/yuvaan_logs/<component>_stdout.log`
- State files: `~/yuvaan_logs/state/`

## Common Tasks

### Start Testing Environment
```bash
./yuvaan_start.sh
# Select: 5 → no_hardware
```

### Progressively Enable Hardware
```bash
# Start with no_hardware (option 5)
# Then repeatedly select option 7 to upgrade:
#   no_hardware → drive_only → drive_mani → full_system
```

### View Logs
```bash
# In interactive menu: option 8
# Or directly:
tail -f ~/yuvaan_logs/merged_control_stdout.log
```

### Save Current Configuration
```bash
# In interactive menu: option 10
# Name it (e.g., "testing_setup")
# Later load with option 11
```

## Troubleshooting

**Can't connect to Jetson**
```bash
ping 192.168.2.101
./yuvaan_start.sh --setup-ssh
```

**Component won't start**
- Check logs (option 8)
- Verify dependencies are running
- Test manually: `rosrun yuvaan_controller merged_control.py`

**Serial device not found**
```bash
# On Jetson:
ls -la /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0 /dev/ttyUSB1
```

## Files

- Config: `config/yuvaan_config.yaml`
- Manager: `src/yuvaan_controller/script/yuvaan_manager.py`
- Launch: `src/yuvaan_controller/launch/yuvaan_system.launch`
- Docs: `docs/SYSTEM_MANAGER_README.md`

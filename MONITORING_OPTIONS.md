# Yuvaan Monitoring Options

Three ways to monitor your robot components in real-time!

## 🎯 Option 1: Auto-Monitoring (NEW! - Recommended)

**Monitoring panes open automatically as you start each component!**

```bash
./yuvaan_start.sh --local --monitor
```

**How it works:**
1. Start the system with `--monitor` flag
2. Start components from the interactive menu
3. Each component automatically gets a monitoring pane
4. View all monitors: `tmux attach -t yuvaan_auto_monitor`

**Perfect for:**
- Interactive testing
- Progressive component startup
- Debugging specific components

📖 [Full Guide](docs/AUTO_MONITORING.md)

---

## 📊 Option 2: Monitor All Components

**Pre-configured monitoring for ALL 10 components**

```bash
./monitor_all.sh
```

**Shows:**
- roscore, merged_control, joy_node
- joy_echo, unified_echo, mani_echo  
- serial_node_main, serial_node_mani
- video_stream, ping_monitor

**Perfect for:**
- Full system overview
- When all components are already running
- Quick debugging session

---

## 🎨 Option 3: Custom Monitor

**Select only the components you want to watch**

```bash
./monitor_custom.sh
```

**Interactive selection:**
- Choose which components to monitor
- Create a custom tmux session
- Saves screen space

**Perfect for:**
- Focused debugging
- Limited screen space
- Specific subsystem testing

---

## Quick Comparison

| Method | When to Use | Auto-Create | Customizable |
|--------|-------------|-------------|--------------|
| **Auto-Monitor** | Interactive testing | ✅ Yes | Matches what you start |
| **Monitor All** | Full system | ❌ Manual | No - shows all 10 |
| **Custom Monitor** | Specific components | ❌ Manual | ✅ You choose |

## Examples

### Full Testing Session

```bash
# Option 1: Auto-monitoring
./yuvaan_start.sh --local --monitor
# Start components, monitors appear automatically
# In another terminal: tmux attach -t yuvaan_auto_monitor
```

### Quick System Check

```bash
# Option 2: Monitor all
# Terminal 1: Start system
./yuvaan_start.sh --local

# Terminal 2: Monitor everything
./monitor_all.sh
```

### Debug Specific Issue

```bash
# Option 3: Custom monitor
./monitor_custom.sh
# Select: merged_control, unified_echo, serial_node_main
```

---

**Try auto-monitoring now:**
```bash
./yuvaan_start.sh --local --monitor
```

See [docs/AUTO_MONITORING.md](docs/AUTO_MONITORING.md) for complete guide!

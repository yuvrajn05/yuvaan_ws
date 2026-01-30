# Auto-Monitoring Feature

## What is Auto-Monitoring?

When you start components, the system **automatically creates monitoring panes** in a tmux session showing live output from each component!

## How to Use

### Start with Auto-Monitoring

```bash
# Local mode with auto-monitoring
./yuvaan_start.sh --local --monitor

# Remote mode with auto-monitoring  
./yuvaan_start.sh --monitor
```

### What Happens

1. **System starts** - Interactive menu appears
2. **Monitoring session created** - A detached tmux session is ready
3. **Start components** - As each component starts, a monitoring pane opens
4. **View monitors** - Attach to the tmux session to see all outputs

### View the Monitoring Session

After starting components, view all monitors in another terminal:

```bash
tmux attach -t yuvaan_auto_monitor
```

**Controls:**
- `Ctrl+B` then `Arrow Keys` - Navigate between panes
- `Ctrl+B` then `Z` - Zoom current pane (fullscreen)
- `Ctrl+B` then `[` - Scroll mode (press `q` to exit)
- `Ctrl+B` then `D` - Detach (monitoring keeps running)

### Complete Workflow

**Terminal 1: System Manager**
```bash
cd ~/yuvaan_ws
./yuvaan_start.sh --local --monitor

# You'll see:
# ======================================================================
#   Auto-Monitoring Enabled
# ======================================================================
# 
# Monitoring session created: yuvaan_auto_monitor
# 
# As you start components, a monitoring pane will open for each one.
# To view the monitoring session, run:
#   tmux attach -t yuvaan_auto_monitor

# Now use the interactive menu:
# Option 5 - Start Group
# Choose: no_hardware
# Components start and monitoring panes are created!
```

**Terminal 2: View Monitors**
```bash
tmux attach -t yuvaan_auto_monitor

# You'll see a tiled layout with:
# ┌──────────────┬──────────────┬──────────────┐
# │  ROSCORE     │ MERGED CTL   │  JOY NODE    │
# ├──────────────┼──────────────┼──────────────┤
# │  JOY ECHO    │ UNIFIED ECHO │              │
# └──────────────┴──────────────┴──────────────┘
```

## How It Works

1. **Session Creation** - When you start with `--monitor`, a detached tmux session is created
2. **Auto-Pane Creation** - Each time you start a component, a new pane is added showing `tail -f` of that component's log
3. **Auto-Layout** - Panes are automatically arranged in a tiled layout
4. **Persistent** - The tmux session persists even if you detach

## Advantages

✅ **Automatic** - No need to manually open monitoring scripts  
✅ **Real-time** - See output as soon as components start  
✅ **All-in-one** - Single tmux session shows everything  
✅ **Flexible** - View monitoring whenever you want  
✅ **Multi-tasking** - Manager in one  terminal, monitors in another

## Examples

### Start and Immediately Monitor

```bash
# Terminal 1
./yuvaan_start.sh --local --monitor
# Start group, but don't attach yet

# Terminal 2 (at the same time)
watch -n 1 'tmux list-panes -t yuvaan_auto_monitor 2>/dev/null | wc -l'
# Watch the pane count increase as you start components!

# Then attach when ready
tmux attach -t yuvaan_auto_monitor
```

### Progressive Group Upgrade with Monitoring

```bash
./yuvaan_start.sh --local --monitor

# In menu:
# 1. Start no_hardware (option 5, then 1)
#    → 5 panes created
# 2. Detach (Ctrl+B D) and attach to monitor
# 3. In another terminal, attach to manager again
# 4. Upgrade group (option 7)
#    → New panes appear for added components!
```

## Comparison with Manual Monitoring

| Feature | Auto-Monitor | Manual (monitor_all.sh) |
|---------|--------------|------------------------|
| Setup | `--monitor` flag | Run separate script |
| Timing | Panes appear as components start | Shows all at once |
| Flexibility | Dynamic - adapts to what you start | Static - shows all components |
| Use case | Interactive testing | Full system monitoring |

## Tips

1. **Start monitoring first** - Run with `--monitor`, then attach to tmux session in another terminal
2. **Fullscreen a pane** - Press `Ctrl+B` then `Z` to zoom the active pane
3. **Detach and reattach** - Monitoring keeps running when you detach
4. **Kill session** - `tmux kill-session -t yuvaan_auto_monitor` to close all panes

## When to Use

**Use Auto-Monitor when:**
- Testing incrementally (start components one by one)
- Debugging specific components
- Progressively upgrading through groups
- Want monitoring to match what's actually running

**Use monitor_all.sh when:**
- Want to see all possible components
- System is already running
- Need a quick overview
- Testing with full system

---

**Try it now:**
```bash
./yuvaan_start.sh --local --monitor
```

Then in another terminal:
```bash
tmux attach -t yuvaan_auto_monitor
```

Start components and watch them appear! 🚀

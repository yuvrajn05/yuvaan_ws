# Dynamic Monitoring - Auto Open/Close Panes

## ✨ New Behavior

Monitoring panes now **automatically open** when components start and **automatically close** when components stop!

## 🎯 How It Works

### When You Start a Component:
- ✅ Monitoring pane opens automatically
- ✅ Shows live tail of that component's log
- ✅ Arranged in tiled layout with other panes

### When You Stop a Component:
- ✅ Monitoring pane closes automatically
- ✅ Other panes rebalance to fill the space
- ✅ Clean and organized view

### When You Restart a Component:
- ✅ Old pane closes
- ✅ New pane opens with fresh log stream
- ✅ Seamless transition

## 📋 Examples

### Example 1: Start and Stop Components

**Terminal 1: System Manager**
```bash
./yuvaan_start.sh --local --monitor

# Start group
# Option 5 → no_hardware
# 5 panes appear: roscore, merged_control, joy_node, joy_echo, unified_echo

# Stop joy_echo
# Option 3 → joy_echo
# joy_echo pane closes automatically!

# Start it again
# Option 2 → joy_echo
# joy_echo pane reopens!
```

**Terminal 2: Watch the Magic**
```bash
tmux attach -t yuvaan_auto_monitor

# You'll see panes appear and disappear as you start/stop components!
```

### Example 2: Progressive Upgrade with Visual Feedback

```bash
# Start with no_hardware (5 panes)
./yuvaan_start.sh --local --monitor
# Option 5 → no_hardware

# Attach to monitor in another terminal
tmux attach -t yuvaan_auto_monitor

# Back in manager, upgrade to drive_only
# Option 7 (Upgrade to Next Group)
# Watch serial_node_main pane appear!

# Upgrade to drive_mani
# Option 7
# Watch mani_echo and serial_node_mani panes appear!
```

### Example 3: Testing Individual Components

```bash
# Start components one by one
./yuvaan_start.sh --local --monitor

# Start roscore
# Option 2 → roscore
# 1 pane appears

# Start merged_control
# Option 2 → merged_control
# 2 panes now

# Start joy_echo
# Option 2 → joy_echo
# 3 panes now

# Stop joy_echo to test something
# Option 3 → joy_echo
# Back to 2 panes

# Restart it
# Option 4 → joy_echo
# 3 panes again!
```

## 🔍 What Each Action Does

| Action | Monitoring Panes |
|--------|------------------|
| **Start Component** | Opens new pane for that component |
| **Stop Component** | Closes pane for that component |
| **Restart Component** | Closes old pane, opens new pane |
| **Start Group** | Opens panes for all components in group |
| **Stop Group** | Closes all panes for that group |
| **Upgrade Group** | Opens panes for newly added components only |

## 🎨 Visual Example

**Starting no_hardware group:**
```
Before: Just placeholder pane
┌──────────────────────────┐
│ Waiting for components...│
└──────────────────────────┘

After starting no_hardware:
┌────────┬────────┬────────┐
│ROSCORE │MERGED  │JOY_NODE│
│        │CONTROL │        │
├────────┼────────┼────────┤
│JOY_ECHO│UNIFIED │        │
│        │ECHO    │        │
└────────┴────────┴────────┘

After stopping joy_echo:
┌────────┬────────┬────────┐
│ROSCORE │MERGED  │JOY_NODE│
│        │CONTROL │        │
├────────┴────────┼────────┤
│UNIFIED_ECHO     │        │
│                 │        │
└─────────────────┴────────┘

After restarting joy_echo:
┌────────┬────────┬────────┐
│ROSCORE │MERGED  │JOY_NODE│
│        │CONTROL │        │
├────────┼────────┼────────┤
│JOY_ECHO│UNIFIED │        │
│        │ECHO    │        │
└────────┴────────┴────────┘
```

## 💡 Benefits

✅ **Visual Feedback** - See what's actually running  
✅ **Clean Interface** - No dead panes from stopped components  
✅ **Easy Debugging** - Focus only on active components  
✅ **Automatic** - No manual pane management needed  

## 🚀 Try It Now

**Test the dynamic behavior:**

```bash
# Terminal 1
./yuvaan_start.sh --local --monitor

# Terminal 2
tmux attach -t yuvaan_auto_monitor

# Now in Terminal 1:
# 1. Start a component (Option 2) → Watch pane appear
# 2. Stop that component (Option 3) → Watch pane disappear
# 3. Restart it (Option 4) → Watch pane reappear
# 4. Start a group (Option 5) → Watch multiple panes appear
# 5. Stop the group (Option 6) → Watch all panes disappear
```

---

**The monitoring now perfectly mirrors your running components!** 🎉

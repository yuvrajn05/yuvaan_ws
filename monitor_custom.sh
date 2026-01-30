#!/bin/bash
# Custom monitor - select which components to watch

SESSION_NAME="yuvaan_custom"
LOG_DIR="$HOME/yuvaan_logs"

echo "=========================================="
echo "  Yuvaan Custom Monitor"
echo "=========================================="
echo ""
echo "Select components to monitor:"
echo ""

# List all components
components=(
    "roscore:ROS Master"
    "merged_control:Main Controller"
    "joy_node:Joystick Input"
    "joy_echo:Joystick Echo"
    "unified_echo:Unified Commands Echo"
    "mani_echo:Manipulator Echo"
    "serial_node_main:Main Serial (Drive)"
    "serial_node_mani:Manipulator Serial"
    "video_stream:Camera Stream"
    "ping_monitor:Network Ping"
)

selected=()

for i in "${!components[@]}"; do
    IFS=':' read -r name desc <<< "${components[$i]}"
    read -p "$(($i + 1)). $desc [$name] (y/N): " choice
    if [[ "$choice" =~ ^[Yy]$ ]]; then
        selected+=("$name")
    fi
done

if [ ${#selected[@]} -eq 0 ]; then
    echo "No components selected. Exiting."
    exit 0
fi

echo ""
echo "Starting monitor for: ${selected[*]}"
echo ""

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create new session
tmux new-session -d -s $SESSION_NAME -n "Custom"

# Add first component
first="${selected[0]}"
tmux send-keys -t $SESSION_NAME "echo '=== ${first^^} ===' && tail -f $LOG_DIR/${first}_stdout.log $LOG_DIR/${first}_stderr.log 2>/dev/null || echo 'Waiting for $first...'" C-m

# Add remaining components
for comp in "${selected[@]:1}"; do
    tmux split-window -t $SESSION_NAME
    tmux send-keys -t $SESSION_NAME "echo '=== ${comp^^} ===' && tail -f $LOG_DIR/${comp}_stdout.log $LOG_DIR/${comp}_stderr.log 2>/dev/null || echo 'Waiting for $comp...'" C-m
    tmux select-layout -t $SESSION_NAME tiled
done

echo "✅ Monitor created with ${#selected[@]} components"
echo ""
echo "Attaching to tmux session..."
tmux attach-session -t $SESSION_NAME

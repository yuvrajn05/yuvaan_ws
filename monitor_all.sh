#!/bin/bash
# Launch Yuvaan components in a tmux session with split panes
# Each pane shows live output from a different component

SESSION_NAME="yuvaan_monitor"
LOG_DIR="$HOME/yuvaan_logs"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "tmux not found. Installing..."
    sudo apt update && sudo apt install -y tmux
fi

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

echo "=========================================="
echo "  Starting Yuvaan Multi-Terminal Monitor"
echo "  Showing ALL Components"
echo "=========================================="
echo ""
echo "Creating tmux session with all component outputs..."
echo ""

# Create new session
tmux new-session -d -s $SESSION_NAME -n "Yuvaan"

# Create a 3x4 grid layout
# Row 1
tmux send-keys -t $SESSION_NAME "echo '=== ROSCORE ===' && tail -f $LOG_DIR/roscore_stdout.log 2>/dev/null || echo 'Waiting for roscore...'" C-m

tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== MERGED CONTROL ===' && tail -f $LOG_DIR/merged_control_stdout.log 2>/dev/null || echo 'Waiting for merged_control...'" C-m

tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== JOY NODE ===' && tail -f $LOG_DIR/joy_node_stdout.log 2>/dev/null || echo 'Waiting for joy_node...'" C-m

# Row 2
tmux select-pane -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== JOY ECHO ===' && tail -f $LOG_DIR/joy_echo_stdout.log 2>/dev/null || echo 'Waiting for joy_echo... (Move joystick!)'" C-m

tmux select-pane -t $SESSION_NAME:0.2
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== UNIFIED ECHO ===' && tail -f $LOG_DIR/unified_echo_stdout.log 2>/dev/null || echo 'Waiting for unified_echo...'" C-m

tmux select-pane -t $SESSION_NAME:0.4
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== MANI ECHO ===' && tail -f $LOG_DIR/mani_echo_stdout.log 2>/dev/null || echo 'Waiting for mani_echo...'" C-m

# Row 3
tmux select-pane -t $SESSION_NAME:0.1
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== SERIAL MAIN ===' && tail -f $LOG_DIR/serial_node_main_stdout.log $LOG_DIR/serial_node_main_stderr.log 2>/dev/null || echo 'Waiting for serial_node_main...'" C-m

tmux select-pane -t $SESSION_NAME:0.4
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== SERIAL MANI ===' && tail -f $LOG_DIR/serial_node_mani_stdout.log $LOG_DIR/serial_node_mani_stderr.log 2>/dev/null || echo 'Waiting for serial_node_mani...'" C-m

tmux select-pane -t $SESSION_NAME:0.7
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== VIDEO STREAM ===' && tail -f $LOG_DIR/video_stream_stdout.log 2>/dev/null || echo 'Waiting for video_stream...'" C-m

# Row 4
tmux select-pane -t $SESSION_NAME:0.2
tmux split-window -v -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "echo '=== PING MONITOR ===' && tail -f $LOG_DIR/ping_monitor_stdout.log 2>/dev/null || echo 'Waiting for ping_monitor...'" C-m

# Balance the layout to make it look nice
tmux select-layout -t $SESSION_NAME tiled

# Attach to the session
echo "✅ Created monitoring session with ALL 10 components!"
echo ""
echo "Layout (3x4 grid):"
echo "  Row 1: roscore | merged_control | joy_node"
echo "  Row 2: joy_echo | unified_echo | mani_echo"
echo "  Row 3: serial_main | serial_mani | video_stream"
echo "  Row 4: ping_monitor | (empty) | (empty)"
echo ""
echo "Controls:"
echo "  Ctrl+B then Arrow Keys - Navigate between panes"
echo "  Ctrl+B then D - Detach from session (keeps running)"
echo "  Ctrl+B then [ - Scroll mode (q to exit)"
echo "  Ctrl+B then Z - Zoom current pane (toggle fullscreen)"
echo "  'tmux attach -t $SESSION_NAME' - Reattach to session"
echo "  'tmux kill-session -t $SESSION_NAME' - Close all panes"
echo ""
echo "Press Enter to attach..."
read

tmux attach-session -t $SESSION_NAME

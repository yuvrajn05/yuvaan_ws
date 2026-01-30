#!/bin/bash
# Launch Yuvaan components in separate gnome-terminal windows
# Arranges them in a grid layout

LOG_DIR="$HOME/yuvaan_logs"

echo "=========================================="
echo "  Starting Yuvaan Multi-Window Monitor"
echo "=========================================="
echo ""

# Check if gnome-terminal is available
if ! command -v gnome-terminal &> /dev/null; then
    echo "gnome-terminal not found!"
    echo "Use monitor_all.sh (tmux version) instead"
    exit 1
fi

# Get screen dimensions for positioning
SCREEN_WIDTH=$(xdpyinfo | grep dimensions | awk '{print $2}' | cut -d'x' -f1)
SCREEN_HEIGHT=$(xdpyinfo | grep dimensions | awk '{print $2}' | cut -d'x' -f2)

# Calculate window size (3 columns, 2 rows)
WIN_WIDTH=$((SCREEN_WIDTH / 3))
WIN_HEIGHT=$((SCREEN_HEIGHT / 2))

echo "Opening component monitors in separate windows..."
echo "Arranging in 3x2 grid..."
echo ""

# Row 1, Column 1 - roscore
gnome-terminal --geometry=80x20+0+0 \
  --title="ROSCORE" \
  -- bash -c "echo '=== ROSCORE ==='; tail -f $LOG_DIR/roscore_stdout.log 2>/dev/null || (echo 'Waiting for roscore...'; sleep infinity)" &

sleep 0.3

# Row 1, Column 2 - merged_control  
gnome-terminal --geometry=80x20+$WIN_WIDTH+0 \
  --title="MERGED CONTROL" \
  -- bash -c "echo '=== MERGED CONTROL ==='; tail -f $LOG_DIR/merged_control_stdout.log 2>/dev/null || (echo 'Waiting for merged_control...'; sleep infinity)" &

sleep 0.3

# Row 1, Column 3 - joy_node
gnome-terminal --geometry=80x20+$((WIN_WIDTH * 2))+0 \
  --title="JOY NODE" \
  -- bash -c "echo '=== JOY NODE ==='; tail -f $LOG_DIR/joy_node_stdout.log 2>/dev/null || (echo 'Waiting for joy_node...'; sleep infinity)" &

sleep 0.3

# Row 2, Column 1 - joy_echo
gnome-terminal --geometry=80x20+0+$WIN_HEIGHT \
  --title="JOY ECHO" \
  -- bash -c "echo '=== JOY ECHO ==='; tail -f $LOG_DIR/joy_echo_stdout.log 2>/dev/null || (echo 'Waiting for joy_echo...'; sleep infinity)" &

sleep 0.3

# Row 2, Column 2 - unified_echo
gnome-terminal --geometry=80x20+$WIN_WIDTH+$WIN_HEIGHT \
  --title="UNIFIED ECHO" \
  -- bash -c "echo '=== UNIFIED ECHO ==='; tail -f $LOG_DIR/unified_echo_stdout.log 2>/dev/null || (echo 'Waiting for unified_echo...'; sleep infinity)" &

sleep 0.3

# Row 2, Column 3 - serial_node_main
gnome-terminal --geometry=80x20+$((WIN_WIDTH * 2))+$WIN_HEIGHT \
  --title="SERIAL NODE MAIN" \
  -- bash -c "echo '=== SERIAL NODE MAIN ==='; tail -f $LOG_DIR/serial_node_main_stdout.log $LOG_DIR/serial_node_main_stderr.log 2>/dev/null || (echo 'Waiting for serial_node_main...'; sleep infinity)" &

echo ""
echo "✅ Monitoring windows opened!"
echo ""
echo "Each window shows live output from a component."
echo "Close individual windows or use Ctrl+C here to exit."

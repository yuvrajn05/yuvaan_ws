#!/bin/bash
# Stream logs from running components in real-time

LOG_DIR="$HOME/yuvaan_logs"

# Check if log directory exists
if [ ! -d "$LOG_DIR" ]; then
    echo "Error: Log directory not found at $LOG_DIR"
    echo "Have you started the system yet?"
    exit 1
fi

echo "=========================================="
echo "  Yuvaan Component Log Viewer"
echo "=========================================="
echo ""

# Find all stdout log files
LOG_FILES=("$LOG_DIR"/*_stdout.log)

if [ ${#LOG_FILES[@]} -eq 0 ] || [ ! -f "${LOG_FILES[0]}" ]; then
    echo "No component logs found."
    echo "Start some components first!"
    exit 0
fi

echo "Available component logs:"
echo ""

# List log files
i=1
for logfile in "${LOG_FILES[@]}"; do
    if [ -f "$logfile" ]; then
        basename=$(basename "$logfile" _stdout.log)
        # Check if process is running
        if pgrep -f "$basename" > /dev/null 2>&1; then
            status="● Running"
        else
            status="○ Stopped"
        fi
        echo "  $i. $status - $basename"
        ((i++))
    fi
done

echo ""
echo "Options:"
echo "  0. View all logs merged"
echo ""
read -p "Enter choice (or press Enter for all): " choice

# Default to all if empty
if [ -z "$choice" ]; then
    choice=0
fi

if [ "$choice" = "0" ]; then
    echo ""
    echo "=========================================="
    echo "  Streaming ALL component logs"
    echo "  Press Ctrl+C to stop"
    echo "=========================================="
    echo ""
    tail -f "$LOG_DIR"/*_stdout.log
else
    # Get the selected file
    selected_file="${LOG_FILES[$((choice-1))]}"
    
    if [ -f "$selected_file" ]; then
        component=$(basename "$selected_file" _stdout.log)
        echo ""
        echo "=========================================="
        echo "  Streaming: $component"
        echo "  Press Ctrl+C to stop"
        echo "=========================================="
        echo ""
        tail -f "$selected_file"
    else
        echo "Invalid selection"
        exit 1
    fi
fi

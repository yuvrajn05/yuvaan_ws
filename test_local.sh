#!/bin/bash
# Local Testing Demo Script for Yuvaan System Manager

echo "=========================================="
echo "  Yuvaan System - Local Mode Test"
echo "=========================================="
echo ""
echo "This script demonstrates the system manager in local mode."
echo "All components will run on this laptop (no Jetson needed)."
echo ""

WS_DIR="/home/neem/yuvaan_ws"
cd "$WS_DIR"

echo "Step 1: Checking system status..."
echo "-----------------------------------"
./yuvaan_start.sh --local --status
echo ""

echo "=========================================="
echo ""
echo "To test interactively, run:"
echo "  ./yuvaan_start.sh --local"
echo ""
echo "Then from the menu:"
echo "  1. Select option 5 (Start Group)"
echo "  2. Choose 'no_hardware' (option 1)"
echo "  3. Watch components start"
echo "  4. Select option 1 to see them running"
echo ""
echo "Note: Components will run locally but may fail if:"
echo "  - ROS is not installed"
echo "  - yuvaan_controller package not found"
echo "  - merged_control.py has errors"
echo ""
echo "This is normal for testing the manager itself!"
echo "=========================================="

#!/bin/bash
# Quick start script for Yuvaan Robot System Manager

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  Yuvaan Robot System Manager"
echo "=========================================="
echo ""

# Get workspace directory
# If script is run from yuvaan_ws/, SCRIPT_DIR will be yuvaan_ws
# Otherwise find it from the actual script location
if [ -f "config/yuvaan_config.yaml" ]; then
    # Running from yuvaan_ws directory
    WS_DIR="$(pwd)"
else
    # Try to find it from script location
    SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    WS_DIR="$SCRIPT_DIR"
fi

echo -e "${YELLOW}Workspace: $WS_DIR${NC}"
echo ""

# Check if config exists
CONFIG_FILE="$WS_DIR/config/yuvaan_config.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Configuration file not found at $CONFIG_FILE"
    exit 1
fi

# Parse command line arguments
MODE="remote"
ACTION="interactive"
AUTO_MONITOR=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --local)
            MODE="local"
            shift
            ;;
        --monitor)
            AUTO_MONITOR="--auto-monitor"
            shift
            ;;
        --setup-ssh)
            ACTION="setup-ssh"
            shift
            ;;
        --status)
            ACTION="status"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --local       Run in local mode (all components on laptop)"
            echo "  --monitor     Auto-open monitoring panes for components"
            echo "  --setup-ssh   Setup SSH keys and exit"
            echo "  --status      Show system status and exit"
            echo "  --help        Show this help message"
            echo ""
            echo "Default: Interactive mode in remote mode (Jetson)"
            echo ""
            echo "Examples:"
            echo "  $0 --local --monitor    # Local mode with auto-monitoring"
            echo "  $0 --monitor            # Remote mode with auto-monitoring"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Build python command
PYTHON_CMD="python3 $WS_DIR/src/yuvaan_controller/script/yuvaan_manager.py"
PYTHON_CMD="$PYTHON_CMD --config $CONFIG_FILE --mode $MODE $AUTO_MONITOR"

case $ACTION in
    setup-ssh)
        PYTHON_CMD="$PYTHON_CMD --setup-ssh"
        ;;
    status)
        PYTHON_CMD="$PYTHON_CMD --status"
        ;;
esac

# Run the manager
echo -e "${GREEN}Starting Yuvaan System Manager...${NC}"
echo ""
$PYTHON_CMD

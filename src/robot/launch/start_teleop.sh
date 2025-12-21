#!/bin/bash
#
# Start all robot control nodes in one terminal
# Usage: ./start_teleop.sh
#

echo "🤖 Starting Mobile Manipulator Teleop..."
echo ""

# Trap Ctrl+C to kill all background processes
cleanup() {
    echo ""
    echo "🛑 Shutting down all nodes..."
    kill $(jobs -p) 2>/dev/null
    wait
    echo "👋 Goodbye!"
    exit 0
}
trap cleanup SIGINT SIGTERM

# Source ROS2 workspace (check multiple locations)
SETUP_FOUND=false

# Check common locations
for SETUP_PATH in \
    ~/Artemis6/src/install/setup.bash \
    ~/Artemis6/src/install/setup.zsh \
    ../install/setup.bash \
    ../install/setup.zsh \
    ../../install/setup.bash \
    ../../install/setup.zsh \
    ./install/setup.bash \
    ./install/setup.zsh
do
    if [ -f "$SETUP_PATH" ]; then
        echo "📦 Sourcing $SETUP_PATH"
        source "$SETUP_PATH"
        SETUP_FOUND=true
        break
    fi
done

if [ "$SETUP_FOUND" = false ]; then
    echo "🚨 No setup.bash or setup.zsh found!"
    echo "   Run from ~/Artemis6/src or make sure you've built with 'colcon build'"
    exit 1
fi

# Start base_controller in background
echo "🚗 Starting base_controller..."
ros2 run robot base_controller &
BASE_PID=$!
sleep 1

# Start arm_controller in background  
echo "🦾 Starting arm_controller..."
ros2 run robot arm_controller &
ARM_PID=$!
sleep 1

echo ""
echo "✅ Controllers started! Now starting keyboard teleop..."
echo ""
sleep 1

# Start keyboard_control in foreground (needs terminal access)
ros2 run robot keyboard_control

# When keyboard_control exits, clean up
cleanup


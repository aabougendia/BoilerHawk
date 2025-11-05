#!/bin/bash
# Build script for control package

echo "Building control package..."

# Navigate to workspace root
cd ~/BoilerHawk

# Clean previous build (optional)
# rm -rf build/control install/control log/control

# Build the control package
colcon build --packages-select control --symlink-install

# Source the setup file
source install/setup.bash

echo "Build complete!"
echo ""
echo "To test, run:"
echo "  Terminal 1: cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter --console --map"
echo "  Terminal 2: ros2 launch control sitl_control_test.launch.py"
echo "  Terminal 3 (MAVProxy): mode GUIDED, then arm throttle"

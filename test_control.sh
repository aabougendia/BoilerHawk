#!/bin/bash

echo "=== Building Control Package ==="
cd /home/zizo2004/BoilerHawk
colcon build --packages-select control --symlink-install

echo ""
echo "=== Build Complete! ==="
echo ""
echo "Now run these commands in separate terminals:"
echo ""
echo "Terminal 1 - ArduPilot SITL:"
echo "  cd ~/ardupilot/Tools/autotest"
echo "  ./sim_vehicle.py -v ArduCopter --console --map"
echo ""
echo "Terminal 2 - Control System (after SITL shows STABILIZE>):"
echo "  cd ~/BoilerHawk"
echo "  source install/setup.bash"
echo "  ros2 launch control sitl_control_test.launch.py pattern:=hover"
echo ""
echo "Terminal 1 again - Arm the drone (in MAVProxy console):"
echo "  mode GUIDED"
echo "  arm throttle"
echo ""

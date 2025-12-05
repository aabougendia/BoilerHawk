#!/bin/bash
# Launch ArduPilot SITL for Iris

# Set environment
source ~/.bashrc
cd ~/ardupilot

echo "========================================="
echo "Launching ArduPilot SITL for Iris"
echo "========================================="
echo ""
echo "Make sure Gazebo is already running!"
echo ""

# Launch SITL
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

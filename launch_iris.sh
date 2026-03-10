#!/bin/bash
# Launch Iris drone with ArduPilot in Gazebo

# Set environment variables
source ~/.bashrc
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}

echo "========================================="
echo "Launching Gazebo with Iris quadcopter"
echo "========================================="
echo ""
echo "After Gazebo loads, open another terminal and run:"
echo "  cd ~/ardupilot"
echo "  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console"
echo ""

# Launch Gazebo
gz sim iris_runway.sdf

#!/bin/bash
# Launch depth camera simulation with all components

cd ~/boilerHawk_ws
source install/setup.bash
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:~/boilerHawk_ws/src/sim_models/models
ros2 launch sim_models depth_camera_sim.launch.py

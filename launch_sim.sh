#!/bin/bash
# ============================================================
# BoilerHawk — One-command full simulation launcher
# ============================================================
#
# Starts everything in a single terminal:
#   Gazebo  →  ArduPilot SITL  →  MAVROS  →  Perception  →  Planning  →  Control
#
# Usage:
#   ./launch_sim.sh                  # default goal (3, 3)
#   ./launch_sim.sh goal_x:=5.0 goal_y:=5.0   # custom goal
#
# To arm & takeoff (in a second terminal):
#   source ~/BoilerHawk/BoilerHawk/install/setup.bash
#   ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
#   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
#   ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 2.0}"
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source the ROS 2 workspace
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "ERROR: install/setup.bash not found. Run 'colcon build' first."
    exit 1
fi

echo "========================================="
echo "  BoilerHawk — Full Simulation Launch"
echo "========================================="
echo ""
echo "This will start:"
echo "  • Gazebo Harmonic  (outdoor world + iris drone)"
echo "  • ArduPilot SITL   (2 s delay)"
echo "  • MAVROS            (20 s delay — waits for SITL TCP port)"
echo "  • Perception + Planning + Control nodes"
echo "  • RViz              (5 s delay)"
echo ""
echo "Ctrl+C to stop everything."
echo "========================================="
echo ""

ros2 launch sim_models stage123_control.launch.py "$@"

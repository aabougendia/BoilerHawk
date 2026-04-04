#!/bin/bash
# ============================================================
# BoilerHawk — One-command full simulation launcher
# ============================================================
#
# Starts everything in a single terminal:
#   Gazebo  →  ArduPilot SITL  →  MAVROS  →  Perception  →  Planning  →  Control
#
# Usage:
#   ./launch_sim.sh                  # default outdoor world
#   ./launch_sim.sh delivery         # package delivery simulation
#   ./launch_sim.sh goal_x:=5.0 goal_y:=5.0   # custom goal (outdoor)
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

# ── Source ROS 2 Jazzy ───────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash

# ── Build workspace ──────────────────────────────────────────────────────
echo "[setup] Building workspace..."
colcon build --packages-select sim_models control planning perception mission_manager sensors_interface --symlink-install 2>&1 | tail -5 || true
echo "[setup] Build complete"

# ── Source the workspace overlay ─────────────────────────────────────────
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo "ERROR: install/setup.bash not found. Build failed?"
    exit 1
fi

# ── Restart ROS 2 daemon (avoid stale context) ──────────────────────────
ros2 daemon stop  2>/dev/null || true
ros2 daemon start 2>/dev/null || true

echo "========================================="
echo "  BoilerHawk — Full Simulation Launch"
echo "========================================="
echo ""

# Check for delivery mode
if [ "$1" = "delivery" ]; then
    shift  # remove 'delivery' from args
    echo "  MODE: Package Delivery"
    echo ""
    echo "This will start:"
    echo "  • Gazebo Harmonic  (delivery world + iris drone + package)"
    echo "  • ArduPilot SITL   (2 s delay)"
    echo "  • MAVROS            (25 s delay — waits for SITL TCP port)"
    echo "  • Perception + Planning + Control nodes"
    echo "  • Mission Manager  (package_delivery strategy)"
    echo "  • Delivery Manager (attach/detach bridge)"
    echo "  • RViz              (5 s delay)"
    echo ""
    echo "Ctrl+C to stop everything."
    echo "========================================="
    echo ""
    ros2 launch sim_models delivery_sim.launch.py "$@"
elif [ "$1" = "rescue" ]; then
    shift  # remove 'rescue' from args
    echo "  MODE: Search & Rescue"
    echo ""
    echo "This will start:"
    echo "  • Gazebo Harmonic  (rescue world + iris drone + victims)"
    echo "  • ArduPilot SITL   (2 s delay)"
    echo "  • MAVROS            (25 s delay — waits for SITL TCP port)"
    echo "  • Perception + Planning + Control nodes"
    echo "  • Human Detector   (YOLOv8n on depth camera)"
    echo "  • Mission Manager  (search_and_rescue strategy)"
    echo "  • RViz              (5 s delay)"
    echo ""
    echo "Ctrl+C to stop everything."
    echo "========================================="
    echo ""
    ros2 launch sim_models rescue_sim.launch.py "$@"
else
    echo "  MODE: Default (outdoor world)"
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
fi

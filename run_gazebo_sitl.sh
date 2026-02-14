#!/bin/bash
# ========================================================================
#  BoilerHawk – Gazebo SITL Runner
#  Launches Gazebo Harmonic + ROS 2 bridges + MAVROS,
#  then starts ArduPilot SITL in a separate process.
# ========================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── colours ──────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}================================================${NC}"
echo -e "${BLUE}  BoilerHawk – Gazebo + ArduPilot SITL Runner   ${NC}"
echo -e "${BLUE}================================================${NC}"

# ── cleanup on exit ──────────────────────────────────────────────────────
cleanup() {
    echo -e "\n${YELLOW}Shutting down all processes...${NC}"
    [[ -n "$SITL_PID"   ]] && kill "$SITL_PID"   2>/dev/null || true
    [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
    pkill -f "arducopter"    2>/dev/null || true
    pkill -f "mavproxy"      2>/dev/null || true
    pkill -f "gz sim"        2>/dev/null || true
    echo -e "${GREEN}Cleanup complete.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# ── prerequisite checks ─────────────────────────────────────────────────
ARDUPILOT_DIR="${ARDUPILOT_HOME:-$HOME/ardupilot}"
GZ_WS_DIR="${GZ_WS:-$HOME/gz_ws}"
ARDUPILOT_GZ_DIR="$GZ_WS_DIR/src/ardupilot_gazebo"

if [[ ! -d "$ARDUPILOT_DIR" ]]; then
    echo -e "${RED}ERROR: ArduPilot not found at $ARDUPILOT_DIR${NC}"
    echo "  Set ARDUPILOT_HOME or install ArduPilot at ~/ardupilot"
    exit 1
fi

if [[ ! -d "$ARDUPILOT_GZ_DIR" ]]; then
    echo -e "${RED}ERROR: ardupilot_gazebo not found at $ARDUPILOT_GZ_DIR${NC}"
    echo "  Clone https://github.com/ArduPilot/ardupilot_gazebo into ~/gz_ws/src/"
    echo "  and build with:  cd ~/gz_ws && colcon build"
    exit 1
fi

# ── environment variables ────────────────────────────────────────────────
echo -e "${YELLOW}Setting environment variables...${NC}"

# ArduPilotPlugin shared library
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}$GZ_WS_DIR/install/ardupilot_gazebo/lib/ardupilot_gazebo"

# Model search paths (local models + ardupilot_gazebo models)
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$SCRIPT_DIR/src/sim_models/models:$ARDUPILOT_GZ_DIR/models:$ARDUPILOT_GZ_DIR/worlds"

echo "  GZ_SIM_SYSTEM_PLUGIN_PATH = $GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "  GZ_SIM_RESOURCE_PATH      = $GZ_SIM_RESOURCE_PATH"

# ── source ROS 2 + workspace ────────────────────────────────────────────
echo -e "${YELLOW}Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# Build sim_models (+ control & planning if present)
echo -e "${YELLOW}Building workspace...${NC}"
colcon build --packages-select sim_models 2>/dev/null || true
colcon build --packages-select control planning 2>/dev/null || true
source install/setup.bash

echo -e "${GREEN}Build complete!${NC}"

# ── 1. Launch Gazebo + bridges + MAVROS via ROS 2 launch ────────────────
echo -e "\n${BLUE}[1/2] Launching Gazebo + bridges + MAVROS...${NC}"

FCU_URL="${FCU_URL:-tcp://127.0.0.1:5762}"
ros2 launch sim_models full_gazebo_sitl.launch.py \
    fcu_url:="$FCU_URL" \
    rviz:=true \
    mavros:=true &
LAUNCH_PID=$!

echo -e "${YELLOW}Waiting for Gazebo to start (10 s)...${NC}"
sleep 10

# ── 2. Start ArduPilot SITL ─────────────────────────────────────────────
echo -e "\n${BLUE}[2/2] Starting ArduPilot SITL...${NC}"

SIM_VEHICLE="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
if [[ ! -f "$SIM_VEHICLE" ]]; then
    echo -e "${RED}ERROR: sim_vehicle.py not found at $SIM_VEHICLE${NC}"
    exit 1
fi

# Launch ArduCopter with Gazebo-iris JSON backend
cd "$ARDUPILOT_DIR"
python3 "$SIM_VEHICLE" \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    --map --console \
    --no-rebuild \
    -I0 &
SITL_PID=$!

cd "$SCRIPT_DIR"

# Wait for SITL + MAVROS connection
echo -e "${YELLOW}Waiting for SITL + MAVROS handshake (25 s)...${NC}"
sleep 25

# ── Ready! ───────────────────────────────────────────────────────────────
echo ""
echo -e "${GREEN}================================================${NC}"
echo -e "${GREEN}   BoilerHawk Gazebo SITL is running!           ${NC}"
echo -e "${GREEN}================================================${NC}"
echo ""
echo -e "${CYAN}Quick reference:${NC}"
echo "  ros2 topic list                               # all active topics"
echo "  ros2 topic echo /mavros/state                 # MAVROS connection"
echo "  ros2 topic echo /mavros/local_position/pose   # drone pose"
echo "  ros2 topic echo /depth_camera/image --no-arr  # camera stream"
echo ""
echo -e "${CYAN}To arm & fly (in another terminal):${NC}"
echo "  source /opt/ros/jazzy/setup.bash && source install/setup.bash"
echo "  ros2 launch control full_system.launch.py     # planning + control"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop everything.${NC}"
echo ""

# Keep script running
wait "$LAUNCH_PID"

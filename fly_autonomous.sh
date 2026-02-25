#!/bin/bash
# ========================================================================
#  BoilerHawk – Autonomous flight: real perception + obstacle avoidance
#
#  Launches:
#    1. Gazebo Harmonic with sensor bridges + RViz
#    2. ArduPilot SITL
#    3. Perception (depth -> occupancy) + Planning (avoid_only) + Control
#
#  Behavior: Drone flies with no fixed goal; goal is always ahead of the
#  drone so it keeps moving and avoids obstacles using the depth camera.
#
#  Usage:  ./fly_autonomous.sh
#  Stop:   Ctrl+C
# ========================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║   BoilerHawk – Autonomous (perception + avoid) ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════╝${NC}"

cleanup() {
    echo -e "\n${YELLOW}Shutting down all processes...${NC}"
    pkill -f "arducopter"       2>/dev/null || true
    pkill -f "sim_vehicle"      2>/dev/null || true
    pkill -f "mavproxy"         2>/dev/null || true
    pkill -f "gz sim"           2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "mavros_node"      2>/dev/null || true
    pkill -f "mavros_router"    2>/dev/null || true
    pkill -f "control_node"     2>/dev/null || true
    pkill -f "planning_node"    2>/dev/null || true
    pkill -f "perception_node"  2>/dev/null || true
    pkill -f "mock_perception"  2>/dev/null || true
    pkill -f "rviz2"            2>/dev/null || true
    sleep 1
    echo -e "${GREEN}Cleanup complete.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo -e "${YELLOW}[setup] Killing stale processes from previous runs...${NC}"
pkill -f "arducopter"       2>/dev/null || true
pkill -f "sim_vehicle"      2>/dev/null || true
pkill -f "mavproxy"         2>/dev/null || true
pkill -f "mavros_node"      2>/dev/null || true
pkill -f "mavros_router"    2>/dev/null || true
pkill -f "control_node"     2>/dev/null || true
pkill -f "planning_node"    2>/dev/null || true
pkill -f "perception_node"  2>/dev/null || true
pkill -f "mock_perception"  2>/dev/null || true
sleep 2

ARDUPILOT_DIR="${ARDUPILOT_HOME:-$HOME/ardupilot}"
GZ_WS_DIR="${GZ_WS:-$HOME/gz_ws}"
ARDUPILOT_GZ_DIR="$GZ_WS_DIR/src/ardupilot_gazebo"
ARDUCOPTER_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
SITL_EXTRA_PARM="$SCRIPT_DIR/src/sim_models/config/sitl_extra.parm"
SITL_DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm,$SITL_EXTRA_PARM"

for check in "$ARDUPILOT_DIR" "$ARDUPILOT_GZ_DIR"; do
    [[ -d "$check" ]] || { echo -e "${RED}ERROR: $check not found${NC}"; exit 1; }
done
[[ -x "$ARDUCOPTER_BIN" ]] || { echo -e "${RED}ERROR: arducopter not found at $ARDUCOPTER_BIN${NC}"; exit 1; }

echo -e "${YELLOW}[setup] Setting environment...${NC}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}$GZ_WS_DIR/install/ardupilot_gazebo/lib/ardupilot_gazebo"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$SCRIPT_DIR/src/sim_models/models:$ARDUPILOT_GZ_DIR/models:$ARDUPILOT_GZ_DIR/worlds"

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
ros2 daemon stop  2>/dev/null || true
ros2 daemon start 2>/dev/null || true

echo -e "${YELLOW}[setup] Building workspace (perception + planning + control + sim_models)...${NC}"
colcon build --packages-select sim_models perception planning control 2>&1 | tail -5 || true
source install/setup.bash
echo -e "${GREEN}[setup] Build complete${NC}"

LOG_DIR="/tmp/boilerhawk_logs"
mkdir -p "$LOG_DIR"

# STEP 1: Gazebo + sensor bridges + RViz
echo -e "\n${BLUE}[1/3] Starting Gazebo + sensor bridges + RViz...${NC}"
ros2 launch sim_models full_gazebo_sitl.launch.py \
    rviz:=true \
    mavros:=false \
    > "$LOG_DIR/gazebo_launch.log" 2>&1 &
GZ_PID=$!
echo -e "${YELLOW}       Waiting for Gazebo to initialize (15s)...${NC}"
sleep 15
if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: Gazebo launch failed!${NC}"
    tail -20 "$LOG_DIR/gazebo_launch.log"
    exit 1
fi
echo -e "${GREEN}       Gazebo is running (PID $GZ_PID) ✓${NC}"

# STEP 2: ArduPilot SITL
echo -e "\n${BLUE}[2/3] Starting ArduPilot SITL...${NC}"
(
    cd "$ARDUPILOT_DIR"
    exec "$ARDUCOPTER_BIN" \
        --model JSON \
        --speedup 1 \
        --slave 0 \
        --defaults "$SITL_DEFAULTS" \
        --sim-address=127.0.0.1 \
        -I0
) > "$LOG_DIR/sitl.log" 2>&1 &
SITL_PID=$!
echo -e "${YELLOW}       Waiting for SITL to bind port 5760 (10s)...${NC}"
sleep 10
if ! kill -0 "$SITL_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: ArduCopter SITL died!${NC}"
    cat "$LOG_DIR/sitl.log"
    exit 1
fi
echo -e "${GREEN}       SITL is running (PID $SITL_PID) ✓${NC}"

# STEP 3: Perception + Control (BendyRuler avoidance)
echo -e "\n${BLUE}[3/3] Launching perception + control (BendyRuler)...${NC}"
echo -e "${CYAN}       Depth -> occupancy -> OBSTACLE_DISTANCE -> ArduPilot BendyRuler${NC}"
ros2 launch control full_autonomous.launch.py avoid_only:=true \
    > "$LOG_DIR/control.log" 2>&1 &
CTRL_PID=$!
sleep 5
if ! kill -0 "$CTRL_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: Autonomous launch failed!${NC}"
    tail -30 "$LOG_DIR/control.log"
    exit 1
fi
echo -e "${GREEN}       Perception + Control (BendyRuler) running ✓${NC}"

echo ""
echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${GREEN}║   Drone flying: no fixed goal, avoid only    ║${NC}"
echo -e "${BOLD}${GREEN}║   Press Ctrl+C to stop everything            ║${NC}"
echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Logs:${NC}"
echo "  tail -f $LOG_DIR/sitl.log"
echo "  tail -f $LOG_DIR/control.log"
echo "  tail -f $LOG_DIR/gazebo_launch.log"
echo ""
wait "$GZ_PID"

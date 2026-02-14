#!/bin/bash
# ========================================================================
#  BoilerHawk – Full Automated Gazebo SITL Flight
#
#  Launches everything needed for autonomous drone flight:
#    1. Gazebo Harmonic with sensor bridges + RViz
#    2. ArduPilot SITL (arducopter binary directly)
#    3. MAVROS (via ros2 run, not launch – avoids topic crash)
#    4. Planning + Control nodes
#    5. Arms, sets GUIDED mode, takes off
#
#  Usage:  ./fly.sh
#  Stop:   Ctrl+C
# ========================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── colours ──────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║   BoilerHawk – Automated Gazebo Flight      ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════╝${NC}"

# ── cleanup on exit ──────────────────────────────────────────────────────
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
    pkill -f "mock_perception"  2>/dev/null || true
    pkill -f "rviz2"            2>/dev/null || true
    sleep 1
    echo -e "${GREEN}Cleanup complete.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── prerequisite checks ─────────────────────────────────────────────────
ARDUPILOT_DIR="${ARDUPILOT_HOME:-$HOME/ardupilot}"
GZ_WS_DIR="${GZ_WS:-$HOME/gz_ws}"
ARDUPILOT_GZ_DIR="$GZ_WS_DIR/src/ardupilot_gazebo"
ARDUCOPTER_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
SITL_EXTRA_PARM="$SCRIPT_DIR/src/sim_models/config/sitl_extra.parm"
SITL_DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm,$SITL_EXTRA_PARM"
FCU_URL="${FCU_URL:-tcp://127.0.0.1:5760}"

for check in "$ARDUPILOT_DIR" "$ARDUPILOT_GZ_DIR"; do
    [[ -d "$check" ]] || { echo -e "${RED}ERROR: $check not found${NC}"; exit 1; }
done
[[ -x "$ARDUCOPTER_BIN" ]] || { echo -e "${RED}ERROR: arducopter not found at $ARDUCOPTER_BIN${NC}"; exit 1; }

# ── environment variables ────────────────────────────────────────────────
echo -e "${YELLOW}[setup] Setting environment...${NC}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}$GZ_WS_DIR/install/ardupilot_gazebo/lib/ardupilot_gazebo"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$SCRIPT_DIR/src/sim_models/models:$ARDUPILOT_GZ_DIR/models:$ARDUPILOT_GZ_DIR/worlds"

# ── source ROS 2 + build ────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
ros2 daemon stop  2>/dev/null || true
ros2 daemon start 2>/dev/null || true

echo -e "${YELLOW}[setup] Building workspace...${NC}"
colcon build --packages-select sim_models control planning 2>&1 | tail -3 || true
source install/setup.bash
echo -e "${GREEN}[setup] Build complete${NC}"

MAVROS_CFG="$SCRIPT_DIR/install/sim_models/share/sim_models/config/mavros_apm_pluginlists.yaml"
[[ -f "$MAVROS_CFG" ]] || { echo -e "${RED}ERROR: MAVROS config not found at $MAVROS_CFG${NC}"; exit 1; }

# Log directory
LOG_DIR="/tmp/boilerhawk_logs"
mkdir -p "$LOG_DIR"

# ═════════════════════════════════════════════════════════════════════════
# STEP 1: Launch Gazebo + sensor bridges (+ optional RViz)
# ═════════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[1/5] Starting Gazebo + sensor bridges + RViz...${NC}"

ros2 launch sim_models full_gazebo_sitl.launch.py \
    fcu_url:="$FCU_URL" \
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

# ═════════════════════════════════════════════════════════════════════════
# STEP 2: Start ArduPilot SITL (directly, not via sim_vehicle.py)
# ═════════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[2/5] Starting ArduPilot SITL...${NC}"

# Run arducopter directly from its directory so it finds param files
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

# ═════════════════════════════════════════════════════════════════════════
# STEP 3: Start MAVROS (via ros2 run – avoids launch file crash bug)
# ═════════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[3/5] Starting MAVROS...${NC}"

ros2 run mavros mavros_node --ros-args \
    --params-file "$MAVROS_CFG" \
    -p fcu_url:="$FCU_URL" \
    -p fcu_protocol:="v2.0" \
    -p system_id:=1 \
    -p component_id:=1 \
    -p target_system_id:=1 \
    -p target_component_id:=1 \
    > "$LOG_DIR/mavros.log" 2>&1 &
MAVROS_PID=$!

echo -e "${YELLOW}       Waiting for MAVROS to connect (up to 90s)...${NC}"

MAX_WAIT=90
ELAPSED=0
CONNECTED=false
sleep 15
ELAPSED=15

while [[ $ELAPSED -lt $MAX_WAIT ]]; do
    # Check MAVROS is still alive
    if ! kill -0 "$MAVROS_PID" 2>/dev/null; then
        echo -e "${RED}ERROR: MAVROS crashed!${NC}"
        tail -30 "$LOG_DIR/mavros.log"
        exit 1
    fi
    # Check SITL is still alive
    if ! kill -0 "$SITL_PID" 2>/dev/null; then
        echo -e "${RED}ERROR: SITL died during MAVROS connect!${NC}"
        tail -20 "$LOG_DIR/sitl.log"
        exit 1
    fi
    # Check for heartbeat in log
    if grep -q "Got HEARTBEAT\|FCU: ArduPilot\|CON: Got HEARTBEAT" "$LOG_DIR/mavros.log" 2>/dev/null; then
        echo -e "${GREEN}       MAVROS connected to ArduPilot! ✓${NC}"
        CONNECTED=true
        break
    fi
    if grep -q "EKF.*origin set" "$LOG_DIR/mavros.log" 2>/dev/null; then
        echo -e "${GREEN}       MAVROS connected (EKF initialized)! ✓${NC}"
        CONNECTED=true
        break
    fi
    echo -e "${YELLOW}       Waiting... (${ELAPSED}s / ${MAX_WAIT}s)${NC}"
    sleep 5
    ELAPSED=$((ELAPSED + 5))
done

if [[ "$CONNECTED" != "true" ]]; then
    echo -e "${RED}ERROR: MAVROS did not connect within ${MAX_WAIT}s${NC}"
    echo -e "${RED}MAVROS log:${NC}"
    tail -30 "$LOG_DIR/mavros.log"
    echo -e "${RED}SITL log:${NC}"
    tail -20 "$LOG_DIR/sitl.log"
    exit 1
fi

# Give EKF time to fully settle
echo -e "${YELLOW}       Waiting for EKF to settle (15s)...${NC}"
sleep 15

# ═════════════════════════════════════════════════════════════════════════
# STEP 4: Launch planning + control nodes
# ═════════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[4/5] Launching planning + control nodes...${NC}"

ros2 launch control full_system.launch.py \
    > "$LOG_DIR/control.log" 2>&1 &
CTRL_PID=$!

sleep 5
if ! kill -0 "$CTRL_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: Control launch failed!${NC}"
    tail -20 "$LOG_DIR/control.log"
    exit 1
fi
echo -e "${GREEN}       Control + Planning nodes running ✓${NC}"

# ═════════════════════════════════════════════════════════════════════════
# STEP 5: Set GUIDED mode, Arm, and Takeoff
# ═════════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[5/5] Arming drone and commanding takeoff...${NC}"

# 5a. Set GUIDED mode
echo -e "${YELLOW}       Setting GUIDED mode...${NC}"
for i in $(seq 1 8); do
    RESULT=$(timeout 10 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}" 2>&1) || true
    if echo "$RESULT" | grep -qi "mode_sent.*true"; then
        echo -e "${GREEN}       GUIDED mode set! ✓${NC}"
        break
    fi
    sleep 3
done
sleep 2

# 5b. Arm
echo -e "${YELLOW}       Arming...${NC}"
for i in $(seq 1 15); do
    RESULT=$(timeout 10 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" 2>&1) || true
    if echo "$RESULT" | grep -qi "success.*true"; then
        echo -e "${GREEN}       Armed! ✓${NC}"
        break
    fi
    echo -e "${YELLOW}       Arm attempt $i — retrying in 5s...${NC}"
    sleep 5
done
sleep 2

# 5c. Takeoff
echo -e "${YELLOW}       Commanding takeoff to 2m...${NC}"
for i in $(seq 1 5); do
    RESULT=$(timeout 10 ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 2.0}" 2>&1) || true
    if echo "$RESULT" | grep -qi "success.*true"; then
        echo -e "${GREEN}       Takeoff commanded! ✓${NC}"
        break
    fi
    sleep 3
done

# ═════════════════════════════════════════════════════════════════════════
# DONE
# ═════════════════════════════════════════════════════════════════════════
echo ""
echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${GREEN}║   🚁  Drone should be flying in Gazebo!     ║${NC}"
echo -e "${BOLD}${GREEN}║   Press Ctrl+C to stop everything            ║${NC}"
echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Logs:${NC}"
echo "  tail -f $LOG_DIR/sitl.log"
echo "  tail -f $LOG_DIR/mavros.log"
echo "  tail -f $LOG_DIR/control.log"
echo ""

# Keep alive — wait for Gazebo
wait "$GZ_PID"

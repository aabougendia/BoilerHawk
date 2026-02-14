#!/bin/bash
# BoilerHawk Full SITL Test Script
# This script launches the complete SITL simulation with all components

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}   BoilerHawk Full SITL Test Runner    ${NC}"
echo -e "${BLUE}========================================${NC}"

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Shutting down all processes...${NC}"
    
    # Kill all background processes
    if [ ! -z "$SITL_PID" ]; then
        echo "Stopping SITL (PID: $SITL_PID)..."
        kill $SITL_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$MAVROS_PID" ]; then
        echo "Stopping MAVROS (PID: $MAVROS_PID)..."
        kill $MAVROS_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$SYSTEM_PID" ]; then
        echo "Stopping BoilerHawk system (PID: $SYSTEM_PID)..."
        kill $SYSTEM_PID 2>/dev/null || true
    fi
    
    # Kill any remaining related processes
    pkill -f "arducopter" 2>/dev/null || true
    pkill -f "mavproxy" 2>/dev/null || true
    pkill -f "mavros" 2>/dev/null || true
    
    echo -e "${GREEN}Cleanup complete.${NC}"
    exit 0
}

# Set trap to cleanup on exit
trap cleanup SIGINT SIGTERM EXIT

# Check for ArduPilot
ARDUPILOT_DIR="$HOME/ardupilot"
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo -e "${RED}ERROR: ArduPilot not found at $ARDUPILOT_DIR${NC}"
    echo "Please install ArduPilot first."
    exit 1
fi

# Source ROS 2
echo -e "${YELLOW}Sourcing ROS 2...${NC}"
source /opt/ros/jazzy/setup.bash

# Build workspace if needed
echo -e "${YELLOW}Building BoilerHawk workspace...${NC}"
colcon build --packages-select control planning
source install/setup.bash

echo -e "${GREEN}Build complete!${NC}"

# Step 1: Start SITL
echo -e "\n${BLUE}[1/5] Starting ArduPilot SITL...${NC}"

# Start ArduCopter SITL directly (not via sim_vehicle.py to avoid blocking)
SITL_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
if [ ! -f "$SITL_BIN" ]; then
    echo -e "${YELLOW}Building SITL first...${NC}"
    cd "$ARDUPILOT_DIR"
    ./waf configure --board sitl
    ./waf copter
fi

cd "$ARDUPILOT_DIR/ArduCopter"
$SITL_BIN --model + --speedup 1 --defaults ../Tools/autotest/default_params/copter.parm --sim-address=127.0.0.1 -I0 &
SITL_PID=$!

# Wait for SITL to start
echo -e "${YELLOW}Waiting for SITL to start (10 seconds)...${NC}"
sleep 10

# Skip MAVProxy - MAVROS will connect directly to SITL
# The SITL exposes tcp:127.0.0.1:5760 which MAVROS can connect to directly
echo -e "${YELLOW}Skipping MAVProxy (MAVROS connects directly to SITL)...${NC}"

cd "$SCRIPT_DIR"

# Wait a moment for SITL to be ready
sleep 5

# Step 2: Start MAVROS (connects directly to SITL via TCP)
echo -e "\n${BLUE}[2/5] Starting MAVROS...${NC}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:="tcp://127.0.0.1:5760" &
MAVROS_PID=$!

# Wait for MAVROS to connect and EKF to initialize
echo -e "${YELLOW}Waiting for MAVROS and EKF to initialize (30 seconds)...${NC}"
sleep 30

# Step 3: Start BoilerHawk system
echo -e "\n${BLUE}[3/5] Launching BoilerHawk full system...${NC}"
ros2 launch control full_system.launch.py &
SYSTEM_PID=$!

# Wait for system to initialize
echo -e "${YELLOW}Waiting for system to initialize (5 seconds)...${NC}"
sleep 5

# Step 4: Arm the drone (with retry and timeout)
echo -e "\n${BLUE}[4/5] Arming drone...${NC}"
for i in 1 2 3 4 5; do
    echo -e "${YELLOW}Arm attempt $i...${NC}"
    # Use timeout to avoid hanging forever
    RESULT=$(timeout 10 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" 2>&1) || true
    if echo "$RESULT" | grep -q "success=True"; then
        echo -e "${GREEN}Arming successful!${NC}"
        break
    else
        echo -e "${YELLOW}Arm failed or timed out, waiting 5 seconds...${NC}"
        sleep 5
    fi
done

sleep 2

# Step 5: Takeoff (with retry and timeout)
echo -e "\n${BLUE}[5/5] Commanding takeoff to 2 meters...${NC}"
for i in 1 2 3; do
    echo -e "${YELLOW}Takeoff attempt $i...${NC}"
    RESULT=$(timeout 10 ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 2.0}" 2>&1) || true
    if echo "$RESULT" | grep -q "success=True"; then
        echo -e "${GREEN}Takeoff successful!${NC}"
        break
    else
        echo -e "${YELLOW}Takeoff failed or timed out, waiting 3 seconds...${NC}"
        sleep 3
    fi
done

echo -e "\n${GREEN}========================================${NC}"
echo -e "${GREEN}   BoilerHawk SITL is now running!     ${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${YELLOW}Press Ctrl+C to stop all processes${NC}"
echo ""
echo "Useful commands in another terminal:"
echo "  ros2 topic echo /mavros/local_position/pose   # View drone position"
echo "  ros2 topic echo /control/setpoint             # View setpoints"
echo "  ros2 topic echo /planning/path                # View planned path"
echo ""

# Keep script running and wait for processes
wait $SYSTEM_PID

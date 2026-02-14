#!/bin/bash
# BoilerHawk SITL Test Logger
# Captures all relevant data during SITL testing

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR=~/BoilerHawk/logs/test_$TIMESTAMP

echo "📝 Creating log directory: $LOG_DIR"
mkdir -p $LOG_DIR

echo "🎬 Starting logging session: $TIMESTAMP"

# Start ROS bag recording in background
echo "Recording ROS topics..."
ros2 bag record \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/setpoint_position/local \
  /control/status \
  /local_path \
  /global_path \
  /path_markers \
  -o $LOG_DIR/rosbag &

ROSBAG_PID=$!
echo "ROS bag recording started (PID: $ROSBAG_PID)"

# Log initial state
echo "📊 Capturing initial system state..."
echo "=== MAVROS State ===" > $LOG_DIR/initial_state.txt
ros2 topic echo /mavros/state --once >> $LOG_DIR/initial_state.txt 2>&1

echo "=== Control Status ===" >> $LOG_DIR/initial_state.txt
ros2 topic echo /control/status --once >> $LOG_DIR/initial_state.txt 2>&1

echo "=== Setpoint Rate ===" >> $LOG_DIR/initial_state.txt
timeout 5 ros2 topic hz /mavros/setpoint_position/local >> $LOG_DIR/initial_state.txt 2>&1

# Monitor position continuously
echo "📍 Monitoring drone position (press Ctrl+C to stop)..."
echo "Timestamp,X,Y,Z" > $LOG_DIR/position_log.csv

while true; do
    TIMESTAMP=$(date +%s.%N)
    POS=$(ros2 topic echo /mavros/local_position/pose --once 2>/dev/null | grep -A3 "position:" | tail -3)
    X=$(echo "$POS" | grep "x:" | awk '{print $2}')
    Y=$(echo "$POS" | grep "y:" | awk '{print $2}')
    Z=$(echo "$POS" | grep "z:" | awk '{print $2}')
    
    if [ ! -z "$X" ]; then
        echo "$TIMESTAMP,$X,$Y,$Z" >> $LOG_DIR/position_log.csv
        echo "[$TIMESTAMP] Position: ($X, $Y, $Z)"
    fi
    
    sleep 0.5
done 2>&1 | tee $LOG_DIR/monitor.log

# Cleanup on exit
trap "echo 'Stopping rosbag...'; kill $ROSBAG_PID; echo 'Logs saved to: $LOG_DIR'" EXIT

# Quick Reference - Testing Control Node with SITL

## Setup Sequence

### Terminal 1: Start ArduPilot SITL
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

### Terminal 2: Build and Launch ROS2 System
```bash
cd ~/BoilerHawk
colcon build --packages-select control --symlink-install
source install/setup.bash
ros2 launch control sitl_control_test.launch.py pattern:=hover
```

## MAVProxy Commands (Terminal 1)

### Basic Flight Commands
```bash
# Check current status
status

# Change to GUIDED mode (required for control node)
mode GUIDED

# Arm the drone
arm throttle

# Disarm
disarm

# Emergency land
mode LAND

# Return to launch
mode RTL
```

### Monitor Position
```bash
# Watch position updates
watch GLOBAL_POSITION_INT
watch LOCAL_POSITION_NED

# Stop watching
unwatch GLOBAL_POSITION_INT
```

### Manual Position Commands (for testing without ROS)
```bash
# In GUIDED mode, go to position (North, East, Alt)
guided 10 5 20

# Set velocity (m/s North, East, Down)
guided_velocity 1 0 0
```

### Parameter Management
```bash
# View important parameters
param show WPNAV_SPEED      # Waypoint speed (cm/s)
param show WPNAV_ACCEL      # Waypoint acceleration
param show WPNAV_RADIUS     # Waypoint radius

# Modify parameters
param set WPNAV_SPEED 500   # 5 m/s
param set SIM_SPEEDUP 2     # 2x simulation speed

# Save parameters
param save
```

### Simulation Controls
```bash
# Speed up simulation (2x real-time)
param set SIM_SPEEDUP 2

# Add wind
param set SIM_WIND_SPD 5
param set SIM_WIND_DIR 45

# Disable GPS failsafe for testing
param set FS_GCS_ENABLE 0
```

## ROS2 Commands (Terminal 3)

### Monitor Topics
```bash
# List all topics
ros2 topic list

# Echo control status
ros2 topic echo /control/status

# Echo current setpoints
ros2 topic echo /planning/setpoint

# Monitor MAVROS state
ros2 topic echo /mavros/state

# Check topic rates
ros2 topic hz /planning/setpoint
ros2 topic hz /control/status
```

### Test Different Patterns
```bash
# Hover (default)
ros2 launch control sitl_control_test.launch.py pattern:=hover

# Square pattern
ros2 launch control sitl_control_test.launch.py pattern:=square

# Circle trajectory
ros2 launch control sitl_control_test.launch.py pattern:=circle

# Velocity control test
ros2 launch control sitl_control_test.launch.py pattern:=velocity_test
```

### Manual Setpoint Publishing
```bash
# Publish single position setpoint
ros2 topic pub /planning/setpoint control/msg/ControlSetpoint "{
  header: {stamp: {sec: 0, nanosec: 0}},
  mode: 1,
  position_target: {x: 5.0, y: 5.0, z: 3.0},
  yaw_target: 0.0,
  sequence: 0
}" --once

# Publish velocity setpoint
ros2 topic pub /planning/setpoint control/msg/ControlSetpoint "{
  header: {stamp: {sec: 0, nanosec: 0}},
  mode: 2,
  velocity_target: {x: 1.0, y: 0.0, z: 0.0},
  yaw_rate_target: 0.0,
  sequence: 0
}" --rate 10
```

## Testing Workflow

1. **Start SITL** (Terminal 1)
2. **Launch ROS2 system** (Terminal 2)
3. **Verify connection**: Check that `ros2 topic echo /mavros/state` shows `connected: true`
4. **Switch to GUIDED**: In MAVProxy, type `mode GUIDED`
5. **Arm**: In MAVProxy, type `arm throttle`
6. **Watch it fly**: Control node takes over, follows mock planning
7. **Monitor**: Watch status with `ros2 topic echo /control/status`
8. **Land**: Either let it complete pattern or type `mode LAND` in MAVProxy

## Troubleshooting

### MAVROS shows "connected: false"
```bash
# Check if SITL is running
ps aux | grep sim_vehicle

# Check network ports
netstat -an | grep 14550

# Restart MAVROS
# Ctrl+C in Terminal 2, then relaunch
```

### Drone not arming
```bash
# In MAVProxy, check pre-arm checks
arm check

# Common issues:
# - Not in GUIDED mode: mode GUIDED
# - GPS not locked: wait for "GPS: 3D FIX"
# - Battery too low: param set BATT_LOW_VOLT 0
```

### Setpoints not being sent
```bash
# Check if mock planning is publishing
ros2 topic hz /planning/setpoint

# Should show ~10 Hz

# Check control node status
ros2 topic echo /control/status
# Look at state field
```

### Tracking error too high
```bash
# In MAVProxy, increase speed limits
param set WPNAV_SPEED 1000  # 10 m/s
param set WPNAV_ACCEL 200   # 2 m/s²

# Or reduce pattern size in launch file
ros2 launch control sitl_control_test.launch.py pattern:=circle size:=3.0
```

## Visualization

### RViz Setup
```bash
ros2 run rviz2 rviz2
```

Add these topics:
- `/mavros/local_position/pose` → Pose (current position)
- `/mavros/setpoint_position/local` → Pose (target position)
- Fixed Frame: `map`

### PlotJuggler (Optional)
```bash
ros2 run plotjuggler plotjuggler
```

Stream topics:
- `/control/status/position_error`
- `/control/status/battery_remaining`
- `/mavros/local_position/pose/pose/position/z`

## Next Steps After Basic Testing

1. ✅ Verify hover works
2. ✅ Test square pattern
3. ✅ Test circle trajectory
4. ✅ Verify velocity control
5. ⬜ Add Gazebo simulation for sensors
6. ⬜ Integrate with real planning node
7. ⬜ Test hardware

## Safety Notes

- Always keep SITL terminal visible to monitor status
- Have emergency `mode LAND` or `disarm` ready
- Monitor battery in status messages
- Check for pre-arm errors before first flight
- Start with hover pattern for initial testing

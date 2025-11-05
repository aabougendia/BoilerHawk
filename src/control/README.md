# Control Package

Control node for autonomous drone interfacing between planning and ArduPilot via MAVROS.

## Overview

This package provides:
- **Control Node**: Main control logic that receives setpoints from planning and commands ArduPilot via MAVROS
- **Mock Planning Node**: Test node that generates simple trajectories for validation
- **Custom Messages**: ControlSetpoint, ControlStatus, MissionSegment

## Architecture

```
Planning → Control Node → MAVROS → ArduPilot SITL
            ↓
        Status Feedback
```

## Installation

### Prerequisites

```bash
# Install MAVROS
sudo apt install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras

# Install GeographicLib datasets
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Install ArduPilot SITL
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
git checkout Copter-4.5
./waf configure --board sitl
./waf copter
```

### Build Package

```bash
cd ~/BoilerHawk
colcon build --packages-select control
source install/setup.bash
```

## Usage

### 1. Start ArduPilot SITL

In a new terminal:
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

### 2. Launch Control System

In another terminal:
```bash
cd ~/BoilerHawk
source install/setup.bash

# Launch with default hover pattern
ros2 launch control sitl_control_test.launch.py

# Or with different patterns
ros2 launch control sitl_control_test.launch.py pattern:=square
ros2 launch control sitl_control_test.launch.py pattern:=circle
```

### 3. Arm and Test (in MAVProxy console)

```bash
# Switch to GUIDED mode
mode GUIDED

# Arm the drone
arm throttle

# The control node will now take over and follow the mock planning commands
```

## Test Patterns

The mock planning node supports several test patterns:

- **hover**: Hold position at (0, 0, altitude)
- **square**: Fly a square pattern
- **circle**: Circular trajectory
- **velocity_test**: Velocity control test (back and forth motion)

## Topics

### Subscribed
- `/mavros/state` - ArduPilot connection state
- `/mavros/local_position/pose` - Current position
- `/mavros/local_position/velocity_local` - Current velocity
- `/mavros/battery` - Battery status
- `/mavros/global_position/global` - GPS status
- `/planning/setpoint` - Setpoints from planning
- `/planning/mission_segment` - Mission segments (future use)

### Published
- `/mavros/setpoint_position/local` - Position commands to ArduPilot
- `/mavros/setpoint_velocity/cmd_vel` - Velocity commands to ArduPilot
- `/control/status` - Control status feedback

## Parameters

### Control Node
- `update_rate` (default: 20.0 Hz) - Control loop frequency
- `setpoint_timeout` (default: 1.0 s) - Timeout before switching to hover
- `max_velocity` (default: 5.0 m/s) - Maximum velocity limit
- `max_acceleration` (default: 2.0 m/s²) - Maximum acceleration limit
- `position_tolerance` (default: 0.5 m) - Position arrival tolerance
- `takeoff_altitude` (default: 2.0 m) - Takeoff altitude

### Mock Planning Node
- `pattern` (default: 'hover') - Test pattern to execute
- `publish_rate` (default: 10.0 Hz) - Setpoint publish rate
- `altitude` (default: 3.0 m) - Flight altitude
- `size` (default: 5.0 m) - Pattern size

## Monitoring

### View Control Status
```bash
ros2 topic echo /control/status
```

### View Current Setpoints
```bash
ros2 topic echo /planning/setpoint
```

### Visualize in RViz
```bash
ros2 run rviz2 rviz2
# Add topics:
# - /mavros/local_position/pose (Pose)
# - /mavros/setpoint_position/local (Pose)
```

## State Machine

The control node implements the following states:

1. **IDLE** - Waiting for commands, MAVROS not connected
2. **ARMING** - Arming the vehicle
3. **TAKING_OFF** - Executing takeoff sequence
4. **EXECUTING** - Following planning commands
5. **HOVERING** - Holding current position (fallback when no setpoints)
6. **LANDING** - Landing sequence
7. **EMERGENCY** - Emergency state (RTL)
8. **ERROR** - Error state

## Safety Features

- Setpoint timeout detection (auto-hover if planning stops sending)
- Velocity limiting
- Battery monitoring
- GPS quality checks
- Connection monitoring

## Development

### Run Nodes Individually

```bash
# Control node only
ros2 run control control_node.py

# Mock planning only
ros2 run control mock_planning_node.py --ros-args -p pattern:=circle
```

### Custom Setpoint Publisher

For manual testing, publish custom setpoints:
```bash
ros2 topic pub /planning/setpoint control/msg/ControlSetpoint "
header:
  stamp: {sec: 0, nanosec: 0}
mode: 1
position_target: {x: 5.0, y: 5.0, z: 3.0}
yaw_target: 0.0
sequence: 0
"
```

## Troubleshooting

### MAVROS not connecting
- Check ArduPilot SITL is running
- Verify ports: `netstat -an | grep 14550`
- Check MAVROS output for errors

### Control node not arming
- Ensure GUIDED mode is set in MAVProxy
- Check GPS lock: `gps status` in MAVProxy
- Check battery: `status` in MAVProxy

### Drone not moving
- Verify setpoints are being published: `ros2 topic hz /planning/setpoint`
- Check control status: `ros2 topic echo /control/status`
- Monitor MAVROS topics: `ros2 topic list | grep mavros`

## Next Steps

1. ✅ Basic position control in SITL
2. ✅ Velocity control support
3. ⬜ Integrate with real planning node
4. ⬜ Add trajectory tracking
5. ⬜ Implement mission segment handling
6. ⬜ Add geofencing
7. ⬜ Hardware testing

## Contributing

When adding features:
1. Update message definitions if needed
2. Maintain state machine consistency
3. Add safety checks
4. Test in SITL before hardware
5. Update this README

# Control Module

A ROS 2 control package for interfacing between the planning module and ArduPilot via MAVROS.

## Overview

This package implements a control system that:
- Receives waypoint paths from the perception/planning module
- Sends position setpoints to ArduPilot via MAVROS
- Handles flight mode management and arming
- Tracks waypoint progress and path following
- Provides continuous control authority in GUIDED mode

## Features

- **Path Following**: Sequential waypoint tracking from planning module
- **MAVROS Integration**: Position setpoint commands to ArduPilot
- **State Management**: Automatic mode switching and arming (configurable)
- **Waypoint Tracking**: Progress monitoring with distance-based advancement
- **Safety**: Configurable auto-arm and mode switching for safe testing
- **Status Publishing**: Real-time status updates for monitoring

## Package Structure

```
control/
├── control/
│   ├── __init__.py
│   └── control_node.py          # Main control node
├── config/
│   └── control_params.yaml      # Configuration parameters
├── launch/
│   ├── control.launch.py        # Launch control node only
│   └── full_system.launch.py    # Launch with perception
├── test/
│   └── test_control.py          # Unit tests
├── package.xml
├── setup.py
└── README.md
```

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- Python 3.8+
- MAVROS package for ROS 2
- Perception package (for path planning)

### Build Instructions

```bash
# Navigate to your ROS 2 workspace
cd ~/BoilerHawk

# Build the control package
colcon build --packages-select control

# Source the workspace
source install/setup.bash
```

## Usage

### Running with SITL

Complete SITL test setup:

```bash
# Terminal 1: Start ArduPilot SITL
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map

# Terminal 2: Launch MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555

# Terminal 3: Launch full system (perception + control)
ros2 launch control full_system.launch.py

# Terminal 4: Monitor status
ros2 topic echo /control/status
```

### Running Standalone

If you have planning/perception already running:

```bash
ros2 launch control control.launch.py
```

Or run the node directly:

```bash
ros2 run control control_node
```

## ROS 2 Interface

### Subscribed Topics

- `/local_path` (`nav_msgs/Path`)
  - Waypoint path from planning module
  - Control follows these waypoints sequentially

- `/mavros/state` (`mavros_msgs/State`)
  - Flight controller state (mode, armed, connected)

- `/mavros/local_position/pose` (`geometry_msgs/PoseStamped`)
  - Current drone position for waypoint tracking

### Published Topics

- `/mavros/setpoint_position/local` (`geometry_msgs/PoseStamped`)
  - Position setpoints sent to ArduPilot at 20Hz
  - Required for maintaining GUIDED mode control

- `/control/status` (`std_msgs/String`)
  - Human-readable status updates

### Service Clients

- `/mavros/cmd/arming` (`mavros_msgs/CommandBool`)
  - Arm/disarm the flight controller

- `/mavros/set_mode` (`mavros_msgs/SetMode`)
  - Change flight mode (e.g., to GUIDED)

### Parameters

- `waypoint_threshold` (double, default: 0.5)
  - Distance in meters to consider a waypoint reached

- `setpoint_rate` (double, default: 20.0)
  - Frequency (Hz) for publishing setpoints
  - Must be high enough to maintain GUIDED mode

- `auto_arm` (bool, default: false)
  - Automatically arm when ready
  - **WARNING**: Set to false for initial testing

- `auto_mode_switch` (bool, default: true)
  - Automatically switch to GUIDED mode when connected

- `target_altitude` (double, default: 2.0)
  - Target altitude in meters for all waypoints
  - Overrides Z component from planning path

## How It Works

### Path Following Algorithm

1. **Receive Path**: Subscribe to `/local_path` from planning
2. **Extract Waypoint**: Get the current target waypoint
3. **Send Setpoint**: Publish position setpoint at 20Hz
4. **Track Progress**: Monitor distance to current waypoint
5. **Advance**: When waypoint reached (< threshold), move to next
6. **Repeat**: Continue until path complete

### State Management

The control node monitors the MAVROS state and can automatically:
- Switch to GUIDED mode when connected
- Arm the drone when in GUIDED mode (if auto_arm enabled)
- Publish status updates for monitoring

### Continuous Setpoint Publishing

**CRITICAL**: The control node publishes setpoints at 20Hz continuously. This is required for ArduPilot's GUIDED mode to maintain control authority. If setpoints stop, the flight controller may:
- Switch to a failsafe mode
- Trigger RTL (Return to Launch)
- Enter a hover state

## Configuration

Edit `config/control_params.yaml` to customize behavior:

```yaml
control_node:
  ros__parameters:
    waypoint_threshold: 0.5      # Waypoint reached distance (m)
    setpoint_rate: 20.0          # Setpoint publishing rate (Hz)
    auto_arm: false              # Auto-arm (WARNING: false for testing)
    auto_mode_switch: true       # Auto switch to GUIDED
    target_altitude: 2.0         # Flight altitude (m)
```

## Safety Considerations

> [!WARNING]
> **Auto-Arming**: For initial testing, keep `auto_arm: false`. Arm manually via MAVProxy or QGroundControl after verifying the system is working correctly.

> [!IMPORTANT]
> **Continuous Control**: The setpoint publisher runs continuously at 20Hz. This is essential for GUIDED mode and cannot be interrupted without losing control.

> [!CAUTION]
> **Coordinate Frames**: The control node uses LOCAL position setpoints (NED frame) relative to the home position. Ensure your planning module outputs paths in the same frame.

## Testing

### Monitor Topics

```bash
# Check if receiving paths
ros2 topic hz /local_path

# Check setpoint publishing rate
ros2 topic hz /mavros/setpoint_position/local

# Monitor status
ros2 topic echo /control/status

# Check current position
ros2 topic echo /mavros/local_position/pose
```

### Verify MAVROS Connection

```bash
# Check MAVROS state
ros2 topic echo /mavros/state

# Should show:
# - connected: True
# - armed: (True/False)
# - mode: "GUIDED" (when active)
```

## Integration with Other Modules

### Perception/Planning Module
- Planning publishes `/local_path` with waypoints
- Control subscribes and follows the path

### MAVROS
- Control publishes `/mavros/setpoint_position/local`
- ArduPilot receives and executes commands

### System Architecture

```
Perception → /local_path → Control → /mavros/setpoint_position/local → ArduPilot
                                ↓
                         /control/status
```

## Troubleshooting

### Drone Not Moving
- Check if MAVROS is connected: `ros2 topic echo /mavros/state`
- Verify drone is armed and in GUIDED mode
- Check if setpoints are being published: `ros2 topic hz /mavros/setpoint_position/local`
- Verify path is being received: `ros2 topic echo /local_path`

### Mode Won't Switch to GUIDED
- Ensure MAVROS connection is active
- Check ArduPilot parameters (e.g., `GUIDED` mode enabled)
- Verify no GPS/EKF errors in ArduPilot

### Waypoints Not Advancing
- Check `waypoint_threshold` - may be too small
- Verify drone is actually moving (check `/mavros/local_position/pose`)
- Monitor distance calculation in logs

## Future Enhancements

- **Velocity Control**: Add velocity setpoint mode for smoother flight
- **Trajectory Smoothing**: Smooth transitions between waypoints
- **Obstacle Avoidance**: Emergency stop on obstacle detection
- **Recovery Behaviors**: Handle lost paths or MAVROS disconnection
- **Advanced Modes**: Support for velocity, acceleration control

## License

MIT License

## Authors

BoilerHawk Team

## Contact

For questions or issues: aabugendia@gmail.com

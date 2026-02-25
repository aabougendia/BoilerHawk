# Planning Module

A ROS 2 planning package for path planning and obstacle avoidance. Consumes occupancy grids from the perception module and current pose from localization (or FCU), and publishes waypoint paths to the control node.

## Overview

This package implements the planning layer for autonomous flight:
- **Inputs**: Occupancy grid from perception (depth camera → occupancy), current pose from localization / FCU
- **Outputs**: Global and local paths (waypoints) on `/global_path` and `/local_path` for the control node
- Uses current drone pose as **dynamic start** when available; goal from parameters (or future mission topic)
- Obstacle avoidance via A* global planning and local replanning when obstacles are detected

## Features

- **A* Path Planning**: Efficient global path planning on occupancy grids
- **Dynamic Local Planning**: Adapts local paths when new obstacles are detected
- **8-Connected Grid**: Supports diagonal movements for smoother paths
- **Coordinate Conversion**: Handles grid ↔ world coordinate transformations
- **Visualization**: Publishes path markers for RViz visualization

## Data Flow

- **Sensors** (e.g. depth camera) → **Perception** (point cloud → occupancy grid) → **Planning** (this package: occupancy + pose → paths) → **Control** (paths → position commands → FCU).
- The **sensors_interface** and simulation expose camera/depth topics; the **perception** package consumes them and publishes an occupancy grid. Planning subscribes to that grid and to the current pose; it does not subscribe to raw sensors directly.

## Package Structure

```
planning/
├── planning/
│   ├── __init__.py
│   ├── path_planner.py          # Core A* path planning algorithm
│   ├── planning_node.py        # ROS 2 planning node
│   └── mock_perception_node.py # Mock node for testing (generates occupancy + pose)
├── test/
├── launch/
├── config/
│   └── planning_params.yaml
├── package.xml
├── setup.py
└── README.md
```

## Installation

### Prerequisites

- ROS 2 (Humble or later)
- Python 3.8+
- NumPy

### Build Instructions

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select planning

# Source the workspace
source install/setup.bash
```

## Usage

### Running Unit Tests

Test the path planning algorithms with hypothetical data:

```bash
# Run comprehensive unit tests
python3 src/planning/test/test_path_planner.py

# Or use pytest
pytest src/planning/test/test_path_planner.py -v
```

### Running with Mock Data

Launch the planning node with a mock perception node that generates test occupancy grids and pose:

```bash
# Static obstacles test
ros2 launch planning planning_test.launch.py

# Dynamic obstacles test
ros2 launch planning planning_dynamic.launch.py
```

### Running with Real Perception and Drone Pose

Use the full autonomous launch (see control package) so that planning receives `/perception/occupancy` and `/mavlink/local_position/pose`. Configure `occupancy_topic` and `pose_topic` in `config/planning_params.yaml` or via launch parameters.

## ROS 2 Interface

### Subscribed Topics (configurable via parameters)

- `occupancy_topic` (default `/occupancy_grid`): set to `/perception/occupancy` when using the real perception node.
- `pose_topic` (default `/current_pose`): set to `/mavlink/local_position/pose` to use real drone pose from the control/FCU stack.

Message types: `nav_msgs/OccupancyGrid` (0–100 occupancy, -1 unknown), `geometry_msgs/PoseStamped` (current position for local planning and dynamic start).

### Published Topics

- `/global_path` (`nav_msgs/Path`)
  - Complete path from start to goal
  - Computed using A* algorithm

- `/local_path` (`nav_msgs/Path`)
  - Local path segment based on current position
  - Updated dynamically with lookahead distance

- `/path_markers` (`visualization_msgs/MarkerArray`)
  - Visualization markers for RViz
  - Blue line: global path
  - Red line: local path

### Parameters

- `occupancy_topic` (string, default `/occupancy_grid`): Topic for occupancy grid. Use `/perception/occupancy` with real perception.
- `pose_topic` (string, default `/current_pose`): Topic for current pose. Use `/mavlink/local_position/pose` for real drone pose.
- `path_frame` (string, default `map`): Frame ID for published paths. Must match the frame of the occupancy grid (see Coordinate frames below).
- `occupancy_threshold` (int, default: 50): Threshold for considering a cell occupied (0–100).
- `lookahead_distance` (int, default: 20): Number of cells to look ahead for local path planning.
- `planning_frequency` (double, default: 2.0): Frequency (Hz) for local path updates.
- `start_x`, `start_y` (double): Fallback start in world coordinates when pose is not yet available.
- `goal_x`, `goal_y` (double): Goal position in world coordinates (meters).

## Coordinate Frames

- **path_frame**: All published paths use this frame. It should match the frame of the occupancy grid.
- **World-frame (Option A)**: Perception publishes occupancy in a fixed world frame (e.g. `map`), using TF (world → odom → base_link → camera). Planning then uses `path_frame: map` and world-frame start/goal. Best when you have a consistent world map and localization.
- **Robot-centric (Option B)**: Perception publishes a robot-centric grid (e.g. camera link frame). Planning would use the same frame and short-horizon relative goals; start is at the robot. For full autonomous pipeline with a single world map, prefer Option A and configure perception to publish in `map` when TF is available.

## Algorithm Details

### A* Path Planning

The implementation uses the A* algorithm with:
- **Cost Function**: g(n) + h(n)
  - g(n): Actual cost from start to node n
  - h(n): Heuristic (Euclidean distance) from n to goal
- **Grid Type**: 8-connected (allows diagonal movement)
- **Move Costs**: 1.0 for orthogonal, 1.414 (√2) for diagonal

### Local Path Planning

Local path planning:
1. Finds closest point on global path to current position
2. Extracts lookahead segment
3. Validates segment (checks for new obstacles)
4. Replans if obstacles detected
5. Returns valid local path segment

## Testing

The package includes comprehensive unit tests covering:

- **Node Class**: Creation, equality, comparison
- **PathPlanner Class**: 
  - Grid updates
  - Cell validity checking
  - Heuristic calculations
  - Neighbor generation
  - Simple path planning
  - Obstacle avoidance
  - No-path scenarios
  - Local path planning
  - Coordinate conversions
- **Hypothetical Scenarios**:
  - Maze navigation
  - Narrow passage traversal
  - Dynamic environment adaptation

Run tests with:
```bash
python3 src/planning/test/test_path_planner.py
```

## Visualization

To visualize paths in RViz:

```bash
# Launch RViz
rviz2

# Add displays:
# - Map: /occupancy_grid
# - Path: /global_path (blue)
# - Path: /local_path (red)
# - MarkerArray: /path_markers
```

## Configuration

Edit `config/planning_params.yaml` to customize parameters. For real perception and drone pose, set:

```yaml
planning_node:
  ros__parameters:
    occupancy_topic: '/perception/occupancy'
    pose_topic: '/mavlink/local_position/pose'
    path_frame: 'map'
    occupancy_threshold: 50
    lookahead_distance: 20
    planning_frequency: 2.0
    goal_x: 8.0
    goal_y: 8.0
```

## Mock Perception Node

The mock perception node (in this package) generates test occupancy grids and a fake current pose with:

- **Static Obstacles**: Walls, barriers, scattered obstacles
- **Dynamic Obstacles** (optional): Moving obstacles for testing adaptation
- **Grid Size**: Configurable dimensions and resolution
- **Publishing Frequency**: Configurable update rate

This allows testing the planning module without running real perception or a drone.

## Future Enhancements

Potential improvements for production use:

1. **Advanced Algorithms**:
   - D* Lite for incremental replanning
   - RRT/RRT* for complex environments
   - Hybrid A* for non-holonomic constraints

2. **Optimization**:
   - Path smoothing
   - Trajectory optimization
   - Velocity profiling

3. **Robustness**:
   - Inflation layers for safety margins
   - Multi-resolution planning
   - Recovery behaviors

4. **Integration**:
   - TF2 integration for coordinate transforms
   - Costmap integration
   - Action server interface

## License

MIT License

## Authors

BoilerHawk Team

## Contact

For questions or issues, please contact: aabugendia@gmail.com

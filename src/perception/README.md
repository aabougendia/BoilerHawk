# Perception Module

A ROS 2 perception package for path planning with occupancy grid processing.

## Overview

This package implements a perception system that:
- Receives occupancy grids from a planning module
- Computes global paths using A* algorithm
- Updates local paths dynamically based on occupancy changes
- Handles obstacle avoidance and dynamic replanning

## Features

- **A* Path Planning**: Efficient global path planning on occupancy grids
- **Dynamic Local Planning**: Adapts local paths when new obstacles are detected
- **8-Connected Grid**: Supports diagonal movements for smoother paths
- **Coordinate Conversion**: Handles grid ↔ world coordinate transformations
- **Visualization**: Publishes path markers for RViz visualization

## Package Structure

```
perception/
├── perception/
│   ├── __init__.py
│   ├── path_planner.py          # Core A* path planning algorithm
│   ├── perception_node.py       # Main ROS 2 perception node
│   └── mock_planning_node.py    # Mock node for testing
├── test/
│   ├── test_path_planner.py     # Unit tests with hypothetical data
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── launch/
│   ├── perception_test.launch.py      # Launch with static obstacles
│   └── perception_dynamic.launch.py   # Launch with dynamic obstacles
├── config/
│   └── perception_params.yaml   # Configuration parameters
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
colcon build --packages-select perception

# Source the workspace
source install/setup.bash
```

## Usage

### Running Unit Tests

Test the path planning algorithms with hypothetical data:

```bash
# Run comprehensive unit tests
python3 src/perception/test/test_path_planner.py

# Or use pytest
pytest src/perception/test/test_path_planner.py -v
```

### Running with Mock Data

Launch the perception node with a mock planning node that generates test occupancy grids:

```bash
# Static obstacles test
ros2 launch perception perception_test.launch.py

# Dynamic obstacles test
ros2 launch perception perception_dynamic.launch.py
```

### Running Standalone Node

```bash
# Start the perception node
ros2 run perception perception_node

# In another terminal, start the mock planning node
ros2 run perception mock_planning_node
```

## ROS 2 Interface

### Subscribed Topics

- `/occupancy_grid` (`nav_msgs/OccupancyGrid`)
  - Occupancy grid from planning module
  - Values: 0-100 (probability of occupancy), -1 (unknown)

- `/current_pose` (`geometry_msgs/PoseStamped`)
  - Current robot pose for local path planning

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

- `occupancy_threshold` (int, default: 50)
  - Threshold for considering a cell occupied (0-100)

- `lookahead_distance` (int, default: 20)
  - Number of cells to look ahead for local path planning

- `planning_frequency` (double, default: 2.0)
  - Frequency (Hz) for local path updates

- `start_x`, `start_y` (double)
  - Starting position in world coordinates (meters)

- `goal_x`, `goal_y` (double)
  - Goal position in world coordinates (meters)

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
python3 src/perception/test/test_path_planner.py
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

Edit `config/perception_params.yaml` to customize parameters:

```yaml
perception_node:
  ros__parameters:
    occupancy_threshold: 50
    lookahead_distance: 20
    planning_frequency: 2.0
    start_x: 1.0
    start_y: 1.0
    goal_x: 8.0
    goal_y: 8.0
```

## Mock Planning Node

The mock planning node generates test occupancy grids with:

- **Static Obstacles**: Walls, barriers, scattered obstacles
- **Dynamic Obstacles** (optional): Moving obstacles for testing adaptation
- **Grid Size**: Configurable dimensions and resolution
- **Publishing Frequency**: Configurable update rate

This allows testing the perception module independently from the actual planning module.

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

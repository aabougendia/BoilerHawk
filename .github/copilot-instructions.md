# BoilerHawk - ROS 2 Jazzy + Gazebo Harmonic Project

## Project Overview

BoilerHawk is a robotics project using **ROS 2 Jazzy** with **Gazebo Harmonic** simulation. The workspace follows standard ROS 2 workspace structure with multiple packages for autonomous system development (perception, planning, control, localization, sensors).

**Active development is on the `sim/depth_cam` branch** focusing on depth camera simulation integration.

## Architecture

### Package Structure
```
src/
├── sensors_interface/    # Python package (ament_python) - sensor integration & visualization
├── sim_models/          # CMake package (ament_cmake) - Gazebo models, worlds, launch files
├── perception/          # Empty - future perception algorithms
├── planning/            # Empty - future path planning
├── control/             # Empty - future control systems
└── localization/        # Empty - future localization
```

### Key Integration Pattern: Gazebo Harmonic ↔ ROS 2 Jazzy

**Critical**: This project uses Gazebo Harmonic (new generation), NOT Gazebo Classic. The bridge package is `ros_gz_bridge`, not `gazebo_ros`.

#### Sensor Data Flow
1. **Gazebo sensors** publish on Gazebo topics (e.g., `/camera/image`, `/camera/depth_image`)
2. **ros_gz_bridge** bridges Gazebo ↔ ROS 2 topics with message type conversion:
   ```python
   # Example from launch files:
   '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
   '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'
   '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'
   ```
3. **ROS 2 nodes** subscribe to bridged topics using standard ROS 2 message types

#### Model Loading Patterns
- **World files** (`src/sim_models/worlds/*.sdf`) reference models two ways:
  - Fuel models: `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Model Name`
  - Local models: `model://model_name` (requires `GZ_SIM_RESOURCE_PATH` or models/ directory)
- **Local models** require `model.config` + `model.sdf` in `src/sim_models/models/<model_name>/`

### Sensor Implementation: Depth Camera

Located in `src/sim_models/models/depth_cam/model.sdf`:
- Sensor type: `rgbd_camera` (RGB + depth in one sensor)
- Publishes: RGB image, depth image, and point cloud
- Static camera model with visual cylinder representation
- 640x480 resolution, 60° FOV, 0.1-20m range, 30 Hz update rate

## Development Workflows

### Building the Workspace

```bash
# From workspace root
cd ~/boilerHawk_ws

# Build all packages
colcon build

# Build specific package (recommended during development)
colcon build --packages-select sensors_interface
colcon build --packages-select sim_models

# Use symlink-install for Python packages to avoid rebuilding on code changes
colcon build --packages-select sensors_interface --symlink-install

# Source the workspace after building
source install/setup.bash
```

### Running Simulations

**Option 1: Direct Gazebo launch (current active method)**
```bash
gz sim ~/boilerHawk_ws/src/sim_models/worlds/main_world.sdf
```

**Option 2: ROS 2 launch file (includes RViz + bridges)**
```bash
ros2 launch sim_models depth_camera_sim.launch.py
```

### Environment Setup

**IMPORTANT**: Set `GZ_SIM_RESOURCE_PATH` to locate local models:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/boilerHawk_ws/src/sim_models/models
```
Add to `~/.bashrc` for persistence. Without this, `model://depth_camera` references will fail.

### Testing

Tests use standard ROS 2 ament linters:
- `ament_flake8` - Python style checking
- `ament_pep257` - Docstring conventions
- `ament_copyright` - Copyright header validation (currently skipped)
- Run tests: `colcon test --packages-select sensors_interface`

## Code Conventions

### Package Structure Rules

**Python packages** (`sensors_interface`):
- Build type: `ament_python`
- Use `setup.py` with `setuptools`
- Source in `<package_name>/<package_name>/`
- Launch files in `launch/`
- Entry points define ROS 2 nodes in `setup.py`

**CMake packages** (`sim_models`):
- Build type: `ament_cmake`
- Use `CMakeLists.txt` with `ament_cmake`
- Install data files (models, worlds, launch) via `install()` directives
- No source code compilation - purely asset distribution

### Launch File Patterns

Use absolute paths expanded with `os.path.expanduser()`:
```python
world_path = os.path.expanduser('~/boilerHawk_ws/src/sim_models/worlds/main_world.sdf')
```

Launch Gazebo via `ExecuteProcess`, not a Node:
```python
ExecuteProcess(cmd=['gz', 'sim', world_path], output='screen')
```

### ROS 2 Node Patterns

Example from `camera_listener.py`:
- Inherit from `rclpy.node.Node`
- Create subscriptions in `__init__`
- Use `cv_bridge` for image conversion (RGB and depth)
- OpenCV for visualization (`cv2.imshow`, `cv2.waitKey(1)`)
- Standard `main()` pattern: `rclpy.init()` → `rclpy.spin()` → `rclpy.shutdown()`

### SDF Model Conventions

- Version: `1.9` for Gazebo Harmonic compatibility
- Sensor topic names: Use simple names (e.g., `camera`), bridge adds namespace
- Always include `model.config` with model metadata
- Static models: `<static>true</static>` for non-moving sensors/objects

## Project-Specific Details

### Current Depth Camera Node Status

The `depth_camera_node` entry point is defined in `setup.py` but the actual node file doesn't exist yet. The `camera_listener.py` node exists but isn't wired as an entry point.

### File Organization Quirks

- Launch files exist in **both** `sensors_interface/launch/` and `sim_models/launch/` - sim_models version is more complete
- Some setup.py data_files are commented out (depth camera resources)
- Empty placeholder packages have only `src/test.txt` files

### Dependencies

Core ROS 2 packages required:
- `rclpy` - Python ROS 2 client
- `ros_gz_sim`, `ros_gz_bridge` - Gazebo Harmonic integration
- `sensor_msgs`, `geometry_msgs`, `std_msgs` - Message types
- `cv_bridge` - ROS ↔ OpenCV image conversion
- `rviz2` - Visualization
- `tf2_ros` - Transform broadcasting

## Common Pitfalls

1. **Forgetting to source the workspace**: Always `source install/setup.bash` after building
2. **Using wrong Gazebo**: This uses `gz sim`, not `gazebo` (Harmonic vs Classic)
3. **Model path issues**: Set `GZ_SIM_RESOURCE_PATH` or models won't load
4. **Message type mismatches**: Bridge requires exact type mapping (e.g., `PointCloud2` vs `PointCloudPacked`)
5. **Python package changes**: Use `--symlink-install` to avoid constant rebuilds
6. **Colcon build artifacts**: `build/`, `install/`, `log/` are gitignored - never commit

## Quick Reference

```bash
# Full development cycle
cd ~/boilerHawk_ws
colcon build --packages-select sensors_interface --symlink-install
source install/setup.bash
gz sim ~/boilerHawk_ws/src/sim_models/worlds/main_world.sdf

# Check available topics (in another terminal after sourcing)
ros2 topic list
ros2 topic echo /camera/image

# Verify model path is set
echo $GZ_SIM_RESOURCE_PATH
```

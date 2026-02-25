# Perception Module

ROS 2 package that turns depth (and other) sensor data into an occupancy grid for the planning module.

## Role in the Pipeline

- **Inputs**: Point cloud from depth camera (and optionally other sensors). The **sensors_interface** package and simulation expose camera topics (e.g. `/depth_camera/points`, `/camera/depth/color/points`) that this node subscribes to.
- **Output**: Occupancy grid on a configurable topic (default `/perception/occupancy`), consumed by the **planning** package for obstacle avoidance and path planning. Planning does not subscribe to raw sensors; it uses this occupancy output.

## Coordinate Frames

- **grid_frame** (parameter, default `map`): Frame ID of the published occupancy grid. The planning node’s `path_frame` should match this.
  - **World-frame**: Use `map` when you have a fixed world map and TF (e.g. world → odom → base_link → camera). Perception would need to transform points into the map frame before building the grid (current implementation is robot-centric; world-frame support can be added via TF lookup).
  - **Robot-centric**: Use the camera or body frame (e.g. `iris_drone/depth_camera_link`) so the grid moves with the drone. Planning then operates in the same frame with short-horizon relative goals.

## Parameters

- `pointcloud_topic`: Input point cloud (e.g. `/depth_camera/points` in Gazebo).
- `occupancy_topic`: Output occupancy grid (default `/perception/occupancy`). Planning subscribes here when using real perception (set planning’s `occupancy_topic` to this).
- `resolution`, `max_range`, `grid_frame`: Grid construction and frame ID.

## Usage

Run with real sensors (e.g. after starting Gazebo and depth camera bridges):

```bash
ros2 run perception perception_node
```

For the full autonomous pipeline, use the launch file that starts perception + planning + control with correct topic and frame configuration.

# Sensors Interface

This package provides interfaces to onboard sensors (e.g. depth camera) used by the autonomy stack.

## Data flow

- **Sensors** (depth camera, etc.) are exposed as ROS 2 topics (from simulation bridges or real hardware). This package can subscribe to them (e.g. `CameraListener` for images and point clouds).
- The **perception** package subscribes to the depth point cloud topic (e.g. `/depth_camera/points` or `/camera/depth/color/points`) and publishes an **occupancy grid**.
- The **planning** package subscribes to that occupancy grid and to the current drone pose, then publishes **waypoint paths** to the **control** package.

So: **Sensors → Perception (occupancy) → Planning (paths) → Control (commands)**. Planning does not subscribe to raw sensor topics; it uses perception’s occupancy output and the drone pose topic.

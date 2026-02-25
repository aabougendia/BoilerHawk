# Testing the Planning and Autonomous Pipeline

## 1. Quick test with mock data (no Gazebo/SITL)

Uses mock occupancy grid and fake pose; no real sensors or drone.

```bash
cd /path/to/BoilerHawk
source /opt/ros/jazzy/setup.bash   # or humble
colcon build --packages-select planning control
source install/setup.bash

# Launch planning + mock perception + (optionally) control
ros2 launch planning planning_test.launch.py
```

In another terminal:

```bash
source install/setup.bash

# Planning should be publishing paths
ros2 topic list | grep -E "path|occupancy|pose"
ros2 topic hz /local_path
ros2 topic echo /local_path --once
```

You should see `/occupancy_grid`, `/current_pose`, `/global_path`, `/local_path`, `/path_markers`. With `planning_test.launch.py`, only planning + mock_perception run (no control).

---

## 2. Full system with mock (Gazebo + SITL + planning + control)

This is the standard flight script; planning gets mock occupancy and mock pose (not from the depth camera or FCU).

```bash
./fly.sh
```

This starts:

1. Gazebo + depth camera bridges + RViz  
2. ArduPilot SITL  
3. **full_system.launch.py**: mock_perception_node, planning_node, control_node  

Check that planning and control are running:

```bash
ros2 node list
ros2 topic hz /local_path
ros2 topic echo /control/status --once
```

The drone should arm, take off, and follow the path from the **mock** data. This does **not** use real perception or real drone pose for planning.

---

## 3. Full autonomous pipeline (real perception + real pose)

Here planning receives:

- Occupancy from the **perception** node (depth camera → `/perception/occupancy`)
- Pose from the **control** node’s MAVLink position (`/mavlink/local_position/pose`)

### 3a. Build perception

```bash
colcon build --packages-select perception planning control sim_models
source install/setup.bash
```

### 3b. Start Gazebo + SITL (same as fly.sh, but without launching control yet)

**Terminal 1 – Gazebo + bridges:**

```bash
source install/setup.bash
ros2 launch sim_models full_gazebo_sitl.launch.py rviz:=true mavros:=false
```

Wait until Gazebo and the depth camera bridges are up (about 15 seconds). You should have `/depth_camera/points`:

```bash
ros2 topic hz /depth_camera/points
```

**Terminal 2 – ArduPilot SITL:**

```bash
cd ~/ardupilot/ArduCopter   # or your ArduPilot path
export GZ_SIM_SYSTEM_PLUGIN_PATH=...   # same as fly.sh
export GZ_SIM_RESOURCE_PATH=...
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console
# Or use the same SITL start command that fly.sh uses (e.g. arducopter with gazebo-iris)
```

**Terminal 3 – Perception + planning + control (full autonomous):**

```bash
source install/setup.bash
ros2 launch control full_autonomous.launch.py
```

Default launch args:

- `pointcloud_topic:=/depth_camera/points` (matches Gazebo bridge)
- `occupancy_topic:=/perception/occupancy`
- Planning subscribes to `/perception/occupancy` and `/mavlink/local_position/pose`

### 3c. Verify the pipeline

```bash
# Perception: depth → occupancy
ros2 topic hz /perception/occupancy

# Planning: occupancy + pose → paths
ros2 topic hz /local_path

# Control publishes pose (from MAVLink)
ros2 topic hz /mavlink/local_position/pose
ros2 topic echo /control/status --once
```

If the drone is armed and in GUIDED mode, planning should be computing paths from **real** occupancy and **real** pose, and control should be following `/local_path`.

### 3d. Optional: use a different point cloud topic

If your depth camera is on another topic:

```bash
ros2 launch control full_autonomous.launch.py pointcloud_topic:=/camera/depth/points
```

---

## 4. Unit tests (planning)

```bash
source install/setup.bash
cd /path/to/BoilerHawk
python3 -m pytest src/planning/test/test_path_planner.py -v
```

All 18 path planner tests should pass.

---

## Summary

| Test | What runs | Planning input |
|------|-----------|----------------|
| **1. planning_test.launch** | Mock perception + planning | Mock `/occupancy_grid`, `/current_pose` |
| **2. fly.sh** | Gazebo + SITL + mock + planning + control | Mock data (no real perception/pose) |
| **3. full_autonomous** | Gazebo + SITL + **perception** + planning + control | Real `/perception/occupancy`, `/mavlink/local_position/pose` |

To confirm that **autonomous obstacle avoidance** is driven by real sensors and pose, use section 3 and check that `/perception/occupancy` and `/mavlink/local_position/pose` are publishing and that `/local_path` updates accordingly.

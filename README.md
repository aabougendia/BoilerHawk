# BoilerHawk

**Autonomous drone simulation with real-time obstacle avoidance.**

BoilerHawk is a fully integrated ROS 2 robotics stack that flies an Iris quadcopter through a Gazebo environment, detects obstacles with a depth camera, plans paths around them using A\*, and follows those paths via ArduPilot SITL. Everything launches with a single command.

<br>

## Table of Contents

- [Architecture](#architecture)
- [Quick Start](#quick-start)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Packages](#packages)
  - [sim\_models](#sim_models)
  - [sensors\_interface](#sensors_interface)
  - [perception](#perception)
  - [planning](#planning)
  - [control](#control)
- [ROS 2 Topic Map](#ros-2-topic-map)
- [Configuration](#configuration)
- [Testing](#testing)
- [Project Structure](#project-structure)

<br>

---

## Architecture

```
┌──────────────┐    JSON     ┌──────────────┐   MAVLink    ┌──────────────┐
│   Gazebo     │◄───────────►│  ArduPilot   │◄────────────►│   MAVROS     │
│  Harmonic    │  (physics)  │    SITL      │  (UDP 14550) │              │
└──────┬───────┘             └──────────────┘              └──────┬───────┘
       │                                                          │
       │ Gazebo topics                                 /mavros/*  │
       │ (depth_camera/*)                                         │
       ▼                                                          ▼
┌──────────────┐  ros_gz_bridge   ┌──────────────┐        ┌──────────────┐
│  Depth Cam   │─────────────────►│  Perception  │        │   Control    │
│  320×240     │  PointCloud2     │              │        │              │
│  @ 10 Hz     │                  │ Point cloud  │        │ MAVROS I/F   │
└──────────────┘                  │ → Occupancy  │        │ Auto-arm     │
                                  │   grid       │        │ Yaw-toward-  │
                                  │ + inflation  │        │   waypoint   │
                                  └──────┬───────┘        └──────▲───────┘
                                         │                       │
                                /occupancy_grid           /local_path
                                         │                       │
                                         ▼                       │
                                  ┌──────────────┐               │
                                  │   Planning   │───────────────┘
                                  │              │
                                  │ Straight-line│
                                  │ global path  │
                                  │ + A* local   │
                                  │   replanning │
                                  └──────────────┘
```

**Data flow in one sentence:** The depth camera produces a point cloud → perception converts it to an inflated occupancy grid → planning generates waypoints using A\* around obstacles → control sends position setpoints to ArduPilot through MAVROS.

<br>

---

## Quick Start

```bash
# 1. Build the workspace
cd ~/BoilerHawk/BoilerHawk
colcon build --symlink-install
source install/setup.bash

# 2. Launch everything (Gazebo + ArduPilot + MAVROS + all nodes + RViz)
./launch_sim.sh
```

The drone will automatically:
1. Wait for ArduPilot SITL and MAVROS to connect (~25 s)
2. Switch to GUIDED mode
3. Arm and take off to 2 m altitude
4. Follow waypoints from (0, 0) → (10, 10), avoiding the two wall obstacles at (4, 4) and (7, 7)

<br>

---

## Prerequisites

| Component | Version | Purpose |
|-----------|---------|---------|
| **Ubuntu** | 24.04 (or WSL2) | Host OS |
| **ROS 2** | Jazzy Jalisco | Middleware |
| **Gazebo** | Harmonic | Physics simulation |
| **ArduPilot** | Latest `master` | Flight controller (SITL) |
| **MAVROS** | ROS 2 Jazzy build | MAVLink ↔ ROS 2 bridge |
| **Python** | 3.12+ | All node source code |

Additional ROS 2 packages: `ros_gz_bridge`, `ros_gz_sim`, `tf2_ros`, `cv_bridge`, `sensor_msgs_py`.

<br>

---

## Installation

### 1. ArduPilot SITL

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### 2. ArduPilot Gazebo Plugin

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && make -j4
```

### 3. Gazebo Model Path

Add to `~/.bashrc`:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/BoilerHawk/BoilerHawk/src/sim_models/models:~/ardupilot_gazebo/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/ardupilot_gazebo/build
```

### 4. Build BoilerHawk

```bash
cd ~/BoilerHawk/BoilerHawk
colcon build --symlink-install
source install/setup.bash
```

<br>

---

## Packages

### sim\_models

**Type:** `ament_cmake` — asset-only package (no compiled code)

Contains all Gazebo simulation assets: SDF world files, drone model, launch files, and RViz configs.

| Directory | Contents |
|-----------|----------|
| `worlds/` | SDF world files — `outdoor_world_ardupilot.sdf` is the primary world |
| `models/` | Local Gazebo models (Iris drone, depth camera, ground planes) |
| `launch/` | ROS 2 launch files — `stage123_control.launch.py` is the master launcher |
| `rviz/` | RViz configuration files |

#### World: `outdoor_world_ardupilot.sdf`

An outdoor environment with:
- Ground plane, building, pine trees, rocks, construction cone and barrel
- Two test obstacles on the (0,0) → (10,10) diagonal:
  - **Red wall** at (4, 4) — 1.0 × 1.0 × 3.0 m
  - **Blue wall** at (7, 7) — 0.8 × 0.8 × 3.0 m
- Iris quadcopter spawned at origin with the depth camera model included
- Physics: 1 ms step size, shadows disabled for performance
- Spherical coordinates set for ArduPilot GPS (Canberra, Australia default)

#### Model: `iris_with_depth_cam`

An `iris_with_standoffs` quadcopter extended with:
- **RGBD depth camera** — 320×240 @ 10 Hz, 60° FOV, 0.1–10 m range, mounted forward-facing with 11° downward tilt
- **GPS sensor** — 10 Hz with Gaussian noise
- **Magnetometer** — 50 Hz
- **Air pressure sensor** — 50 Hz
- **Lift-drag plugins** for all four rotors (realistic aerodynamics)
- **ArduPilot plugin** for JSON-based SITL communication

#### Launch: `stage123_control.launch.py`

The master launch file that starts the entire system in sequence:

| Delay | Component | Notes |
|-------|-----------|-------|
| 0 s | Gazebo Harmonic | Loads `outdoor_world_ardupilot.sdf` with `-r` (run immediately) |
| 0 s | ros\_gz\_bridge (×4) | Bridges depth camera image, depth, points, and camera\_info topics |
| 0 s | Static TF | `world` → `odom` identity transform |
| 0 s | MAVROS TF Broadcaster | `odom` → `base_link` dynamic TF from MAVROS pose |
| 0 s | Perception, Planning, Control | Start immediately (wait internally for data) |
| 2 s | ArduPilot SITL | `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON` |
| 5 s | RViz | Visualization with `drone_outdoor.rviz` config |
| 25 s | MAVROS | Connects to SITL via UDP (waits for heartbeat) |

---

### sensors\_interface

**Type:** `ament_python`

Provides sensor utility nodes and the TF bridge between MAVROS and the rest of the system.

#### Nodes

| Executable | Source | Description |
|------------|--------|-------------|
| `mavros_tf_broadcaster` | `mavros_tf_broadcaster.py` | **Critical node.** Subscribes to `/mavros/local_position/pose`, broadcasts the `odom` → `base_link` dynamic TF, and republishes the pose on `/current_pose` for the planning node. |
| `depth_camera_node` | `camera_listener.py` | Development/debug tool. Subscribes to RGB and depth image topics and displays them with OpenCV. Not used in autonomous flight. |
| `drone_teleop_key` | `drone_teleop_key.py` | Keyboard teleoperation for manual drone control in Gazebo (WASD + TG + QE). Not used in autonomous flight. |

---

### perception

**Type:** `ament_python`

Converts raw depth camera point clouds into an occupancy grid for the planner.

#### Node: `perception_node`

**Subscriptions:**
| Topic | Type | Description |
|-------|------|-------------|
| `/depth_camera/points` | `PointCloud2` | Raw point cloud from the depth camera via ros\_gz\_bridge |
| `/mavros/local_position/pose` | `PoseStamped` | Drone position and yaw for coordinate transforms |

**Publications:**
| Topic | Type | Description |
|-------|------|-------------|
| `/occupancy_grid` | `OccupancyGrid` | 2D grid centered on the drone, published every point cloud frame |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | `0.2` | Grid cell size in meters |
| `max_range` | `5.0` | Grid extends ±5 m from drone (50×50 grid) |
| `inflate_radius` | `0.6` | Obstacles are expanded by this radius for safety margin |
| `grid_frame` | `map` | TF frame ID for the published grid |

**Processing pipeline:**
1. Read `PointCloud2`, filter by `max_range`
2. Rotate points by drone yaw, translate to world frame (ENU)
3. Rasterize into a 50×50 occupancy grid centered on the drone
4. **Inflate** all occupied cells by `inflate_radius` / `resolution` cells (iterative 8-connected dilation) — this ensures the planner treats thin walls as thick obstacles, giving the drone a safe clearance buffer
5. Publish as `OccupancyGrid`

---

### planning

**Type:** `ament_python`

Path planning using A\* on the live occupancy grid with a two-tier strategy:
- **Global path:** Straight line from start to goal (because A\* can't run on a 10 m local grid when the goal is 14 m away)
- **Local replanning:** When an obstacle is detected ahead, A\* runs on the local grid to compute a detour, then stitches the result back to the remaining global path

#### Node: `planning_node`

**Subscriptions:**
| Topic | Type | Description |
|-------|------|-------------|
| `/occupancy_grid` | `OccupancyGrid` | From perception |
| `/current_pose` | `PoseStamped` | From MAVROS TF broadcaster |

**Publications:**
| Topic | Type | Description |
|-------|------|-------------|
| `/global_path` | `Path` | Full start → goal path |
| `/local_path` | `Path` | Upcoming waypoint window (sent to control) |
| `/path_markers` | `MarkerArray` | RViz visualization (blue = global, red = local) |

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `start_x`, `start_y` | `0.0, 0.0` | Start position in meters |
| `goal_x`, `goal_y` | `10.0, 10.0` | Goal position in meters |
| `occupancy_threshold` | `50` | Grid values ≥ this are treated as occupied |
| `lookahead_distance` | `30` | Waypoints ahead to include in local path |
| `planning_frequency` | `2.0` | How often (Hz) to check for obstacles and update local path |

**Replanning logic:**
1. Every 0.5 s, scan waypoints ahead of the drone (within lookahead window) for obstacles
2. Only check waypoints that fall inside the current occupancy grid (far-away points are skipped)
3. If an obstacle is found and the 5 s cooldown has elapsed:
   - Walk the path forward to find the first **clear** waypoint past the obstacle
   - Run A\* from the drone's current position to that clear waypoint
   - If A\* start or goal lands inside an inflated obstacle cell, BFS-search for the nearest free cell
   - Stitch: `[A* detour] + [remaining global waypoints]`
4. If A\* fails (e.g., completely surrounded), keep the current path rather than replacing it with a line through the obstacle

#### Library: `path_planner.py`

The `PathPlanner` class implements:
- **A\* search** on an 8-connected grid with Euclidean heuristic. Diagonal moves cost √2, cardinal moves cost 1.
- **`_nearest_free_cell()`** — BFS fallback when start or goal is in an occupied cell (searches up to 15 cells out)
- **`world_to_grid()` / `grid_to_world()`** — Coordinate conversion between world meters and grid (row, col). Uses `round()` for sub-cell accuracy.

---

### control

**Type:** `ament_python`

The MAVROS interface — receives paths from planning and sends position setpoints to ArduPilot.

#### Node: `control_node`

**Subscriptions:**
| Topic | Type | Description |
|-------|------|-------------|
| `/local_path` | `Path` | Waypoints from planning |
| `/mavros/state` | `State` | Armed, mode, connected status |
| `/mavros/local_position/pose` | `PoseStamped` | Current drone position |

**Publications:**
| Topic | Type | Description |
|-------|------|-------------|
| `/mavros/setpoint_position/local` | `PoseStamped` | Position commands at 20 Hz |
| `/mavros/rc/override` | `OverrideRCIn` | Throttle failsafe override for SITL |
| `/control/status` | `String` | Human-readable status |

**Parameters** (from `config/control_params.yaml`):
| Parameter | Default | Description |
|-----------|---------|-------------|
| `waypoint_threshold` | `0.5` | Distance (m) to consider a waypoint reached |
| `setpoint_rate` | `20.0` | Setpoint publishing frequency (Hz) |
| `auto_arm` | `true` | Automatically arm when MAVROS connects |
| `auto_mode_switch` | `true` | Automatically switch to GUIDED mode |
| `target_altitude` | `2.0` | Fixed flight altitude (m) |

**Key behaviors:**
- **Auto-sequencing:** On MAVROS connection → switch to GUIDED → arm → takeoff → begin waypoint following. No manual intervention needed.
- **Yaw-toward-waypoint:** Computes `yaw = atan2(dy, dx)` from current position to the target waypoint and sets the orientation quaternion. This keeps the forward-facing depth camera pointed in the direction of travel.
- **Smart path indexing:** On each new path, finds the closest waypoint then skips forward past any already-reached waypoints to prevent back-tracking.
- **Throttle failsafe override:** Continuously publishes RC channel 3 = 1500 to prevent ArduPilot's low-throttle failsafe from triggering in SITL.
- **Diagnostics:** Every 30 s, logs a full state summary (connection, mode, position, waypoint progress, issue detection).

<br>

---

## ROS 2 Topic Map

```
Gazebo                          ROS 2                              Node
──────                          ─────                              ────
depth_camera/image         ──►  /depth_camera/image                (bridge)
depth_camera/depth_image   ──►  /depth_camera/depth_image          (bridge)
depth_camera/points        ──►  /depth_camera/points               (bridge)
depth_camera/camera_info   ──►  /depth_camera/camera_info          (bridge)

                                /mavros/local_position/pose    ──► perception, control, tf_broadcaster
                                /mavros/state                  ──► control
                                /mavros/setpoint_position/local ◄── control
                                /mavros/rc/override            ◄── control
                                /mavros/cmd/arming             ◄── control (service)
                                /mavros/set_mode               ◄── control (service)
                                /mavros/cmd/takeoff            ◄── control (service)

                                /current_pose                  ──► planning         (from tf_broadcaster)
                                /occupancy_grid                ──► planning         (from perception)
                                /global_path                   ──► (RViz)           (from planning)
                                /local_path                    ──► control          (from planning)
                                /path_markers                  ──► (RViz)           (from planning)
                                /control/status                ──► (monitoring)     (from control)
```

**TF Tree:**
```
world → odom → base_link
 (static)  (dynamic, from mavros_tf_broadcaster)
```

<br>

---

## Configuration

### Changing the Goal

```bash
./launch_sim.sh goal_x:=15.0 goal_y:=5.0
```

Or edit the defaults in `src/sim_models/launch/stage123_control.launch.py`.

### Tuning Parameters

| What | File | Key Parameter |
|------|------|---------------|
| Flight altitude | `src/control/config/control_params.yaml` | `target_altitude` |
| Waypoint reach distance | `src/control/config/control_params.yaml` | `waypoint_threshold` |
| Obstacle safety margin | Launch file (`stage123_control.launch.py`) | `inflate_radius` |
| Grid resolution | Launch file | `resolution` |
| Sensor range | Launch file | `max_range` |
| Replan cooldown | `src/planning/planning/planning_node.py` | `5_000_000_000` ns (5 s) |
| Camera resolution | `src/sim_models/models/iris_with_depth_cam/model.sdf` | `<width>`, `<height>` |

### Adding Obstacles

Edit `src/sim_models/worlds/outdoor_world_ardupilot.sdf` and add a new `<model>` block:

```xml
<model name="my_obstacle">
  <static>true</static>
  <pose>X Y 1.5 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry><box><size>1.0 1.0 3.0</size></box></geometry>
    </visual>
    <collision name="collision">
      <geometry><box><size>1.0 1.0 3.0</size></box></geometry>
    </collision>
  </link>
</model>
```

Then rebuild `sim_models`:

```bash
colcon build --packages-select sim_models
```

<br>

---

## Testing

### Unit Tests

```bash
# Path planner tests (18 tests: A*, maze, narrow passage, coordinate conversion, etc.)
colcon test --packages-select planning
colcon test-result --verbose

# Control utility tests (distance calc, waypoint threshold)
colcon test --packages-select control

# Linters (flake8, pep257)
colcon test --packages-select perception sensors_interface
```

### Manual Flight Verification

```bash
# In a second terminal, monitor drone position:
source install/setup.bash
ros2 topic echo /mavros/local_position/pose --field pose.position

# Check if obstacles are being detected:
ros2 topic echo /occupancy_grid --field info  # grid metadata
ros2 topic hz /occupancy_grid                 # should be ~10 Hz

# Check the planned path:
ros2 topic echo /local_path --field poses --once
```

<br>

---

## Project Structure

```
BoilerHawk/
├── launch_sim.sh                          # One-command launcher
├── src/
│   ├── sim_models/                        # [ament_cmake] Simulation assets
│   │   ├── CMakeLists.txt
│   │   ├── launch/
│   │   │   └── stage123_control.launch.py # Master launch file
│   │   ├── models/
│   │   │   └── iris_with_depth_cam/       # Iris drone + RGBD camera + ArduPilot
│   │   ├── worlds/
│   │   │   └── outdoor_world_ardupilot.sdf
│   │   └── rviz/
│   │       └── drone_outdoor.rviz
│   │
│   ├── sensors_interface/                 # [ament_python] TF bridge & sensor utilities
│   │   └── sensors_interface/
│   │       ├── mavros_tf_broadcaster.py   # odom→base_link TF + /current_pose
│   │       ├── camera_listener.py         # Debug: OpenCV depth/RGB viewer
│   │       └── drone_teleop_key.py        # Debug: keyboard control
│   │
│   ├── perception/                        # [ament_python] Point cloud → occupancy grid
│   │   └── perception/
│   │       └── perception.py              # PerceptionNode (inflate, transform, grid)
│   │
│   ├── planning/                          # [ament_python] A* path planning
│   │   ├── planning/
│   │   │   ├── planning_node.py           # PlanningNode (global/local paths)
│   │   │   └── path_planner.py            # PathPlanner (A*, grid utils)
│   │   ├── config/
│   │   │   └── planning_params.yaml
│   │   └── test/
│   │       └── test_path_planner.py       # 18 unit tests
│   │
│   ├── control/                           # [ament_python] MAVROS waypoint follower
│   │   ├── control/
│   │   │   └── control_node.py            # ControlNode (arm, takeoff, setpoints, yaw)
│   │   ├── config/
│   │   │   └── control_params.yaml
│   │   └── test/
│   │       └── test_control.py
│   │
│   └── localization/                      # [placeholder] Future localization package
│
├── build/                                 # colcon build output (gitignored)
├── install/                               # colcon install output (gitignored)
└── log/                                   # colcon log output (gitignored)
```

<br>

---

## Engineering Design Process

BoilerHawk was developed through a rigorous iterative design cycle of **identify → prototype → test → analyze → refine**, applied repeatedly across every subsystem.

### Requirements Definition

The project began with a clear top-level requirement: an autonomous drone that can fly from a start position to a goal while avoiding obstacles in real time, using only onboard sensing. This was decomposed into functional subsystems — simulation environment, sensor integration, perception, path planning, and flight control — each with measurable acceptance criteria (e.g., the drone must reach within 0.5 m of the goal, obstacle clearance must exceed the airframe radius, the system must launch from a single command).

### Iterative Prototyping & Experimentation

Development followed an incremental integration strategy. Each subsystem was built and validated in isolation before being composed with the others:

1. **Simulation environment** — The Gazebo world and drone model were tested first by verifying sensor output (depth images, point clouds) and ArduPilot SITL connectivity before any autonomy code existed.
2. **Perception pipeline** — Point cloud → occupancy grid conversion was validated by echoing `/occupancy_grid` and comparing reported occupied cells against known obstacle positions in the world. Early tests revealed that raw occupied cells were too thin for reliable avoidance, leading to the addition of **obstacle inflation** (0.6 m dilation radius) — a design decision driven directly by experimental failure data.
3. **Path planning** — A\* was first tested on a static grid with 18 unit tests covering empty grids, mazes, narrow passages, unreachable goals, and coordinate round-trips. When integrated with the live perception grid, experiments showed that a pure A\* global planner failed for goals outside the 10 m sensor range. This led to the **two-tier architecture**: a straight-line global path with local A\* replanning only when obstacles enter the grid — a key architectural pivot driven by data from failed flight tests.
4. **Control** — Waypoint-following was validated in obstacle-free flights first (start → goal in open space), confirming smooth convergence. Yaw control was added after observing that the drone flew sideways, leaving the forward-facing depth camera blind to upcoming obstacles.

### Data Analysis & Interpretation

Each flight test produced observable data — drone position logs, occupancy grid snapshots, planned path topics, and control status diagnostics — that were analyzed to diagnose issues:

- **Back-and-forth oscillation** was diagnosed by examining `/local_path` waypoint indices: the control node was re-selecting already-passed waypoints on each path update. The fix was to skip forward past all waypoints within the `waypoint_threshold` radius after finding the closest one.
- **Replanning instability** was identified when the drone alternated between two paths every planning cycle. Topic logs showed A\* producing a new detour each iteration because it was also checking waypoints *behind* the drone. The solution: only scan waypoints ahead of the current position and enforce a 5-second replan cooldown.
- **A\* failure near inflated obstacles** appeared as repeated "A\* replan failed" log messages when the start or goal cell landed inside an inflated region. Grid data analysis confirmed this, leading to the **nearest-free-cell BFS fallback** — a breadth-first search that shifts the start/goal to the closest unoccupied cell.
- **Performance degradation** was measured by monitoring Gazebo real-time factor. Reducing the depth camera from 640×480 @ 30 Hz to 320×240 @ 10 Hz and disabling shadows recovered acceptable simulation performance, validated by real-time factor returning above 0.8.

### Engineering Judgment & Trade-offs

Several design decisions required balancing competing constraints:

| Decision | Trade-off | Rationale |
|----------|-----------|-----------|
| 0.2 m grid resolution | Finer resolution detects smaller gaps but increases A\* computation | 0.2 m is ≈ 2× the airframe radius — sufficient for clearance without excessive cost |
| 0.6 m inflation radius | Larger margin is safer but may close off valid passages | Chosen to exceed the drone's physical radius while keeping standard doorway-width gaps navigable |
| Physics step = 1 ms | Smaller steps improve accuracy but reduce real-time factor | ArduPilot requires ≥ 400 Hz main loop; 1 ms was the maximum step that maintained stable flight |
| Straight-line global path | Simpler than full global A\* but doesn't pre-plan around distant obstacles | Acceptable because the drone replans locally as it approaches each obstacle; a global planner would need a pre-built map that doesn't exist |
| 5 s replan cooldown | Prevents oscillation but delays reaction to new obstacles | Balances stability against responsiveness; at 2 m/s cruise the drone covers 10 m between replans, within sensor range |

### Verification & Validation

The system was validated at multiple levels:
- **Unit tests** (18 path planner + control tests) verify algorithmic correctness in isolation
- **Integration flights** confirm end-to-end behavior: the drone successfully navigates from (0, 0) to (10, 10) while detouring around both wall obstacles
- **Regression testing** after each change ensures previous fixes remain intact (e.g., verifying that the physics step revert didn't break arming, that inflation didn't close off valid paths)

This structured process — grounded in empirical testing, quantitative analysis of topic data, and deliberate engineering trade-offs — transformed an initial collection of ROS 2 packages into a cohesive autonomous system.

<br>

---

## License

MIT

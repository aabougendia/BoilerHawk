# Mission Manager — Architecture & Implementation Report

> **Package**: `mission_manager`  
> **Branch**: `app`  
> **Date**: February 27, 2026  
> **Author**: Copilot session  
> **Status**: Implemented, builds clean, all module imports verified  

---

## 1. Purpose

The `mission_manager` package is a **generic application-layer orchestrator** for the BoilerHawk autonomous quadcopter system. It sits above the existing pipeline:

```
Gazebo sensors → perception → planning → control → ArduPilot
```

It provides **mission lifecycle management** (load, start, pause, resume, abort, RTL) via ROS 2 services and uses a **Finite State Machine (FSM)** to coordinate flight operations. The design is intentionally **generic** — it does not depend on any specific downstream node being alive and does not modify any existing code.

---

## 2. Package Structure

```
src/mission_manager/
├── package.xml                             # ROS 2 package manifest (ament_python)
├── setup.py                                # Setuptools config + entry points
├── setup.cfg                               # Install script directories
├── resource/mission_manager                # ament index marker
├── config/
│   └── mission_params.yaml                 # Default parameters
├── launch/
│   └── mission.launch.py                   # Standalone launch file
└── mission_manager/
    ├── __init__.py
    ├── mission_state.py                    # FSM states + transition rules
    ├── mission_manager_node.py             # Main ROS 2 node (583 lines)
    ├── strategies/
    │   ├── __init__.py
    │   ├── base_strategy.py                # Abstract base class for all strategies
    │   ├── waypoint_survey.py              # Lawnmower area-coverage strategy
    │   ├── search_and_rescue.py            # Expanding-square search strategy
    │   └── perimeter_patrol.py             # Polygon perimeter patrol strategy
    └── utils/
        ├── __init__.py
        └── geo_utils.py                    # Pose creation, distance, pattern generators
```

---

## 3. FSM States & Transitions

```
                   ┌────────────────────────────────────────────────────┐
                   │              EMERGENCY (from any state)            │
                   └────────────────────┬───────────────────────────────┘
                                        │ (on ground)
                                        ▼
 ┌──────┐  /mission/start  ┌───────────┐     ┌─────────┐     ┌───────────┐
 │ IDLE │ ───────────────► │ PREFLIGHT │ ──► │ TAKEOFF │ ──► │ EXECUTING │
 └──▲───┘                  └─────┬─────┘     └─────────┘     └──┬──┬──┬──┘
    │                        fail│                               │  │  │
    │                        back│              /mission/pause ──┘  │  │
    │                            │              ┌────────┐          │  │
    │                            ▼              │ PAUSED │◄─────────┘  │
    │                          IDLE             └──┬──┬──┘             │
    │                                     resume ──┘  │               │
    │                                                 │  /mission/rtl │
    │                                                 ▼               │
    │                                              ┌─────┐            │
    │                                              │ RTL │◄───────────┘
    │                                              └──┬──┘
    │                                                 │
    │                                              ┌──▼─────┐
    └──────────────────────────────────────────────│ LANDING │
                                                   └─────────┘
```

### State definitions (mission_state.py)

| State | Value | Description |
|-------|-------|-------------|
| `IDLE` | 1 | Waiting for mission load + start |
| `PREFLIGHT` | 2 | Checking system health (strategy loaded, MAVLink connected) |
| `TAKEOFF` | 3 | Monitoring altitude — waits for 85% of target alt |
| `EXECUTING` | 4 | Delegating to active strategy — advancing waypoints |
| `PAUSED` | 5 | Mission frozen — holding last goal position |
| `RTL` | 6 | Flying back to launch coordinates |
| `LANDING` | 7 | Descending — waits for alt < 0.3m, then → IDLE |
| `EMERGENCY` | 8 | Failsafe — holds position, auto-recovers when on ground |

### Transition rules

- **EMERGENCY** is reachable from **any** state (always valid)
- Validated via `is_valid_transition(from, to)` — rejects invalid jumps
- Full transition table in `VALID_TRANSITIONS` dict

---

## 4. Mission Manager Node

**File**: `mission_manager/mission_manager_node.py`  
**Node name**: `mission_manager_node`  
**Entry point**: `mission_manager_node = mission_manager.mission_manager_node:main`

### 4.1 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `takeoff_altitude` | float | 2.0 | Target altitude in metres |
| `waypoint_threshold` | float | 0.5 | Distance (m) to consider waypoint reached |
| `fsm_tick_rate` | float | 2.0 | Hz — FSM evaluation frequency |
| `goal_publish_rate` | float | 1.0 | Hz — goal re-publish to `/mission/goal` |
| `health_check_rate` | float | 0.5 | Hz — upstream node health monitoring |
| `health_timeout` | float | 5.0 | Seconds of silence before declaring node dead |
| `rtl_x` | float | 0.0 | Return-to-launch X (local frame) |
| `rtl_y` | float | 0.0 | Return-to-launch Y (local frame) |
| `frame_id` | string | "map" | TF frame for all goal poses |
| `strategy_name` | string | "waypoint_survey" | Strategy to load (set before `/mission/load`) |
| `strategy_params` | string | "{}" | JSON string of strategy parameters |

### 4.2 Publishers

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/mission/state` | `std_msgs/String` | 1 Hz | Current FSM state name |
| `/mission/goal` | `geometry_msgs/PoseStamped` | `goal_publish_rate` Hz | Current goal for planning/control |
| `/mission/feedback` | `std_msgs/String` | At FSM tick rate | Human-readable progress |

### 4.3 Subscribers

| Topic | Type | Source | Description |
|-------|------|--------|-------------|
| `/control/status` | `std_msgs/String` | control_node | Monitor control health + armed/mode |
| `/mavlink/local_position/pose` | `geometry_msgs/PoseStamped` | control_node | Current drone position |

### 4.4 Services (provided)

| Service | Type | Valid from states | Description |
|---------|------|-------------------|-------------|
| `/mission/load` | `std_srvs/SetBool` | IDLE only | `data=true` loads strategy, `data=false` unloads |
| `/mission/start` | `std_srvs/Trigger` | IDLE | Begins mission → PREFLIGHT → TAKEOFF → EXECUTING |
| `/mission/pause` | `std_srvs/Trigger` | EXECUTING | Freezes strategy, holds position |
| `/mission/resume` | `std_srvs/Trigger` | PAUSED | Resumes strategy execution |
| `/mission/abort` | `std_srvs/Trigger` | Any | Immediate → EMERGENCY |
| `/mission/rtl` | `std_srvs/Trigger` | EXECUTING, PAUSED | Return to launch position |

### 4.5 Timers

| Timer | Rate | Purpose |
|-------|------|---------|
| FSM tick | `fsm_tick_rate` Hz (default 2) | Evaluate current state, run state handler |
| Goal publisher | `goal_publish_rate` Hz (default 1) | Re-publish current goal to `/mission/goal` |
| Health check | `health_check_rate` Hz (default 0.5) | Monitor upstream node liveness |
| State publisher | 1 Hz | Publish FSM state to `/mission/state` |

### 4.6 Health Monitoring

- Tracks `time.monotonic()` timestamps of last messages from `/control/status` and `/mavlink/local_position/pose`
- If no message received within `health_timeout` seconds, logs a warning
- **Advisory only** — does NOT auto-abort. Designed to be tolerant of partially-available pipelines

### 4.7 Strategy Loading Flow

```
1. ros2 param set /mission_manager_node strategy_name "waypoint_survey"
2. ros2 param set /mission_manager_node strategy_params '{"min_x":0,...}'
3. ros2 service call /mission/load std_srvs/srv/SetBool "{data: true}"
   → Reads strategy_name + strategy_params from node parameters
   → Looks up class in STRATEGY_REGISTRY
   → Calls strategy.initialize(params)
   → On success: stores strategy, responds with success message
```

---

## 5. Strategy Pattern

### 5.1 Abstract Base Class (`base_strategy.py`)

All strategies subclass `MissionStrategy` and implement:

| Method | Signature | Purpose |
|--------|-----------|---------|
| `initialize` | `(params: Dict) → bool` | Validate & store mission parameters |
| `reset` | `() → None` | Clear internal state for re-use |
| `get_next_goal` | `(current_pose) → PoseStamped or None` | Return current goal waypoint |
| `advance` | `(current_pose, threshold) → bool` | Check if drone reached goal, advance index |
| `is_complete` | `() → bool` | True when no remaining goals |
| `on_pause` | `() → None` | Optional — save state on pause |
| `on_resume` | `() → None` | Optional — restore state on resume |
| `get_progress` | `() → str` | Human-readable progress string |
| `get_status_dict` | `() → Dict` | Machine-readable status dict |

### 5.2 Strategy Registry

New strategies are registered in `STRATEGY_REGISTRY` in `mission_manager_node.py`:

```python
STRATEGY_REGISTRY: Dict[str, type] = {
    "waypoint_survey": WaypointSurveyStrategy,
    "search_and_rescue": SearchAndRescueStrategy,
    "perimeter_patrol": PerimeterPatrolStrategy,
}
```

**Adding a new strategy**: create a file in `strategies/`, subclass `MissionStrategy`, add to this dict.

### 5.3 Implemented Strategies

#### Waypoint Survey (`waypoint_survey.py`)

Generates a **lawnmower (boustrophedon)** coverage pattern over a rectangular area.

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `min_x` | float | Yes* | — | Bounding box minimum X |
| `min_y` | float | Yes* | — | Bounding box minimum Y |
| `max_x` | float | Yes* | — | Bounding box maximum X |
| `max_y` | float | Yes* | — | Bounding box maximum Y |
| `spacing` | float | No | 1.0 | Row spacing in metres |
| `altitude` | float | No | 2.0 | Flight altitude |
| `frame_id` | string | No | "map" | TF frame |
| `waypoints` | list | No | — | Override: raw `[[x,y],...]` list bypasses lawnmower |

*Required if `waypoints` is not provided.

Progress: `"Waypoint 5/24 (21%)"`

#### Search and Rescue (`search_and_rescue.py`)

Generates an **expanding-square** search pattern from a centre point.

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `center_x` | float | Yes* | — | Search centre X |
| `center_y` | float | Yes* | — | Search centre Y |
| `initial_leg` | float | No | 2.0 | First leg length |
| `leg_increment` | float | No | 2.0 | Leg increase per pair |
| `num_legs` | int | No | 16 | Total number of legs |
| `altitude` | float | No | 2.0 | Flight altitude |
| `frame_id` | string | No | "map" | TF frame |
| `waypoints` | list | No | — | Override: raw waypoint list |

*Required if `waypoints` is not provided.

**Future hook**: `on_detection(pose, label)` method for perception integration — records detections with waypoint index.

Progress: `"Search 5/17 (29%), detections: 2"`

#### Perimeter Patrol (`perimeter_patrol.py`)

Repeatedly circuits a **polygon perimeter** for a configurable number of loops.

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `vertices` | list | Yes | — | `[[x,y], ...]` polygon vertices (≥2) |
| `loops` | int | No | 1 | Number of circuits (0 = infinite) |
| `altitude` | float | No | 2.0 | Flight altitude |
| `frame_id` | string | No | "map" | TF frame |

Progress: `"Loop 3/5, waypoint 7/12"` or `"Loop 4/∞, waypoint 2/8"`

---

## 6. Utility Functions (`geo_utils.py`)

| Function | Signature | Description |
|----------|-----------|-------------|
| `make_pose` | `(x, y, z, frame_id) → PoseStamped` | Create pose with identity orientation |
| `distance_xy` | `(a, b) → float` | XY-plane Euclidean distance |
| `distance_3d` | `(a, b) → float` | Full 3D Euclidean distance |
| `generate_lawnmower` | `(min_x, min_y, max_x, max_y, alt, spacing) → [PoseStamped]` | Boustrophedon pattern |
| `generate_perimeter` | `(vertices, alt) → [PoseStamped]` | Polygon vertices to pose list |
| `generate_expanding_square` | `(cx, cy, alt, initial_leg, increment, num_legs) → [PoseStamped]` | Spiral search pattern |

---

## 7. Configuration

**File**: `config/mission_params.yaml`

```yaml
mission_manager_node:
  ros__parameters:
    fsm_tick_rate: 2.0
    goal_publish_rate: 1.0
    health_check_rate: 0.5
    takeoff_altitude: 2.0
    waypoint_threshold: 0.5
    frame_id: "map"
    rtl_x: 0.0
    rtl_y: 0.0
    health_timeout: 5.0
    strategy_name: "waypoint_survey"
    strategy_params: '{"min_x": 0, "min_y": 0, "max_x": 5, "max_y": 5, "spacing": 1.0}'
```

---

## 8. Launch

**File**: `launch/mission.launch.py`

Launches `mission_manager_node` with `mission_params.yaml`:

```bash
ros2 launch mission_manager mission.launch.py
```

---

## 9. Integration with Existing System

### 9.1 What the node connects to (no modifications to existing code)

```
mission_manager_node
    ├── subscribes: /control/status        (from control_node)
    ├── subscribes: /mavlink/local_position/pose  (from control_node)
    ├── publishes:  /mission/state         (for monitoring)
    ├── publishes:  /mission/goal          (for planning/control)
    └── publishes:  /mission/feedback      (for monitoring)
```

### 9.2 How it fits in the full pipeline

```
┌─────────────────────┐
│  mission_manager    │  ← Orchestrator (NEW)
│  /mission/goal ─────┼──────────────────────────────────┐
│  /mission/state     │                                  │
│  /mission/feedback  │                                  │
└────────┬────────────┘                                  │
         │ subscribes                                    │ publishes
         │ /control/status                               │ PoseStamped goals
         │ /mavlink/local_position/pose                  │
         │                                               ▼
┌────────┴────────┐    ┌──────────────┐    ┌─────────────────────┐
│  control_node   │◄───│ planning_node│◄───│ perception_node     │
│  (pymavlink)    │    │ (A* paths)   │    │ (pointcloud→grid)   │
│  /local_path    │    │ /local_path  │    │ /occupancy_grid     │
└────────┬────────┘    └──────────────┘    └─────────┬───────────┘
         │                                           │
         │ MAVLink TCP:5760                          │ /camera/depth/...
         ▼                                           │
┌─────────────────┐    ┌─────────────┐               │
│  ArduPilot SITL │    │   Gazebo    │───────────────┘
│                 │◄──►│  Harmonic   │  ros_gz_bridge
└─────────────────┘    └─────────────┘
```

### 9.3 Known integration gaps (NOT addressed — no existing code modified)

1. **Perception → Planning topic mismatch**: `perception_node` publishes `/perception/occupancy` but `planning_node` subscribes to `/occupancy_grid`. The mission manager does not depend on either.
2. **Pose feedback gap**: `planning_node` subscribes to `/current_pose` but `control_node` publishes on `/mavlink/local_position/pose`. The mission manager uses the control topic directly.
3. **`/mission/goal` is not yet consumed by planning_node**: Future work — planning_node needs a subscriber for dynamic goal updates from the mission manager.

---

## 10. Operator Usage

### Build

```bash
cd ~/BoilerHawk
colcon build --packages-select mission_manager --symlink-install
source install/setup.bash
```

### Launch standalone

```bash
ros2 launch mission_manager mission.launch.py
```

### Load and run a mission

```bash
# 1. Set strategy parameters
ros2 param set /mission_manager_node strategy_name "waypoint_survey"
ros2 param set /mission_manager_node strategy_params \
  '{"min_x": 0, "min_y": 0, "max_x": 10, "max_y": 10, "spacing": 2.0}'

# 2. Load mission
ros2 service call /mission/load std_srvs/srv/SetBool "{data: true}"

# 3. Start
ros2 service call /mission/start std_srvs/srv/Trigger

# 4. Monitor
ros2 topic echo /mission/state
ros2 topic echo /mission/feedback
ros2 topic echo /mission/goal
```

### In-flight commands

```bash
ros2 service call /mission/pause  std_srvs/srv/Trigger   # freeze
ros2 service call /mission/resume std_srvs/srv/Trigger   # continue
ros2 service call /mission/rtl    std_srvs/srv/Trigger   # return home
ros2 service call /mission/abort  std_srvs/srv/Trigger   # emergency
```

### Unload mission (while IDLE)

```bash
ros2 service call /mission/load std_srvs/srv/SetBool "{data: false}"
```

---

## 11. Adding a New Strategy

1. Create `src/mission_manager/mission_manager/strategies/my_strategy.py`
2. Subclass `MissionStrategy` from `base_strategy.py`
3. Implement all abstract methods: `initialize`, `reset`, `get_next_goal`, `advance`, `is_complete`, `get_progress`
4. Register in `mission_manager_node.py`:
   ```python
   from mission_manager.strategies.my_strategy import MyStrategy
   STRATEGY_REGISTRY["my_strategy"] = MyStrategy
   ```
5. Rebuild: `colcon build --packages-select mission_manager --symlink-install`

---

## 12. Future Work

| Priority | Item | Description |
|----------|------|-------------|
| High | Connect `/mission/goal` to planning | Add a subscriber in `planning_node` for dynamic goal updates |
| High | Fix perception→planning topic | Align `/perception/occupancy` with `/occupancy_grid` |
| Medium | Custom service types | Replace `SetBool` for `/mission/load` with a typed `MissionLoad.srv` that carries strategy name + params directly |
| Medium | 3D waypoints | Extend strategies to support variable-altitude missions |
| Medium | S&R perception hook | Connect `on_detection()` to a perception detection topic |
| Low | Behavior tree migration | If mission complexity grows, consider replacing FSM with a BT |
| Low | Parameter reconfiguration | Support dynamic reconfiguration of FSM rates mid-flight |
| Low | Mission save/load from file | Serialize/deserialize mission configs to YAML files |

---

## 13. Dependencies

### package.xml

```xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>std_srvs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<exec_depend>control</exec_depend>
<exec_depend>planning</exec_depend>
<exec_depend>perception</exec_depend>
```

### Python (via setup.py)

- `setuptools`

### No external pip packages required — everything uses ROS 2 standard messages.

---

## 14. Verification Results

```
$ colcon build --packages-select mission_manager --symlink-install
Summary: 1 package finished [2.00s]

$ python3 -c "from mission_manager.mission_manager_node import MissionManagerNode, STRATEGY_REGISTRY; ..."
Strategies: ['waypoint_survey', 'search_and_rescue', 'perimeter_patrol']
Node import OK

$ python3 tests...
Survey: Waypoint 0/6 (0%)
SAR: Search 0/9 (0%), detections: 0
Patrol: Loop 1/2, waypoint 1/4
Lawnmower: 6 waypoints
Expanding square: 9 waypoints
All tests passed!
```

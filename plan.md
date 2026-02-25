---
name: drone-falls-on-obstacle
overview: Diagnose and fix the behavior where the drone drops instead of steering around obstacles when the planner detects an obstacle.
todos:
  - id: instrument-behavior
    content: Instrument planner and control nodes to log `/local_path`, preferred angles, modes, and setpoints at obstacle encounter time.
    status: pending
  - id: tune-vfh
    content: Tune VFH parameters and optionally add smoothing/hysteresis so the planner always outputs stable, non-empty waypoints.
    status: pending
  - id: control-safety
    content: Verify and harden control node so it always sends continuous setpoints and maintains altitude when paths are missing or changing.
    status: pending
  - id: scenario-tests
    content: Design and run a set of obstacle scenarios in sim (avoid-only and goal modes) to validate the new behavior.
    status: pending
isProject: false
---

## Goal

Ensure that when the planner detects an obstacle, the drone **keeps flying at roughly constant altitude and steers around the obstacle**, instead of dropping toward the ground or giving up.

## Current Behavior & Likely Causes

- **Observed**: When an obstacle appears in front of the drone, it "falls down to ground" instead of trying alternate directions.
- **Key components involved**:
  - Planner (`planning_node.py`) now uses `VFHPlanner` in robot-centric mode to publish `/local_path` waypoints.
  - Control (`control_node.py`) subscribes to `/local_path` and continuously sends `SET_POSITION_TARGET_LOCAL_NED` with a **fixed altitude** (`target_altitude`).
- **Hypotheses**:
  - **H1 – No valid waypoints / empty paths**: On obstacle detection the planner may sometimes publish empty or extremely small paths, causing the control node to hold or drift and ArduPilot to fall back to LAND/failsafe.
  - **H2 – Violent direction changes**: VFH may occasionally choose a direction that causes large, abrupt lateral accelerations; with limited thrust margin, this can look like the drone "falls" as it pitches/rolls hard to accelerate sideways.
  - **H3 – Mode/failsafe changes**: When path updates become irregular or stop, ArduPilot might exit GUIDED or enter LAND/RTH, causing a descent.

We will **instrument first** to see which hypothesis matches, then adjust planner/control behavior.

## Plan

### 1. Instrument what happens at obstacle detection

- **Add / enable diagnostics during a test run**:
  - In `control_node.py`, watch `/control/status` and log lines from `_diagnostics_loop()` to see:
    - Mode transitions (`GUIDED` → `LAND` or others).
    - Whether `current_path` becomes `None` or stays set.
    - Whether `target_setpoint` disappears.
  - In `planning_node.py`, enable logging around `planning_callback` when `use_robot_centric_grid` is true, to print:
    - The **length of each `/local_path**` being published.
    - The **preferred angle** and whether `_goal_reached()` is triggering.
- **Visualize in RViz / topic echo**:
  - Plot `/local_path` and `/mavlink/local_position/pose` simultaneously.
  - Use `ros2 topic echo /local_path` around the time the obstacle appears to see if paths stop updating or shrink to zero.

### 2. Verify VFH outputs in typical obstacle setups

- **Create simple canned scenarios** in the perception grid (either via mock perception or placing boxes in Gazebo):
  - No obstacle, obstacle directly ahead, wall on one side, narrow gap.
- For each, log from `planning_node.py`:
  - Preferred angle (0 in avoid-only, bearing-to-goal in goal mode).
  - Resulting VFH waypoints (first and last few points in grid coords and world coords).
- **Check for anomalies**:
  - Waypoints jumping far away (tens of meters) when an obstacle appears.
  - Waypoints trending consistently down in Y or X with no relation to obstacle layout.

### 3. Make planner behavior safer and more predictable

Assuming instrumentation shows that `/local_path` sometimes becomes empty or erratic when obstacles are close:

- **3.1. Guarantee non-empty paths and smooth changes**
  - In `planning_node.py` `planning_callback` (robot-centric branch), introduce a simple **smoothing / hysteresis**:
    - Keep the last successful `path` and **reuse it** when a new VFH call would produce an empty or extremely short path.
    - Limit maximum change in heading between cycles (e.g., clamp the difference between new `preferred_angle` and the last chosen direction to some `max_turn_rate_per_cycle`).
  - Ensure that we **never publish an empty `/local_path**`; if VFH or upstream data fails, publish a 1–2 waypoint path that just holds the current direction.
- **3.2. Tune VFH parameters for your environment**
  - Increase `safety_radius_cells` gradually to avoid hugging obstacles too closely while still keeping some free sectors.
  - Adjust `valley_threshold` so that sectors only count as blocked when clearly occupied; too low a threshold can mark nearly everything as blocked and force awkward directions.
  - Tune `num_waypoints` and `waypoint_spacing` to make changes gentler (e.g., 2–3 waypoints spaced 0.5–1.0 m).
- **3.3. Make goal mode more robust**
  - For non-`avoid_only` runs, confirm `_bearing_to_goal_in_robot_frame()` matches the actual drone/goal geometry by checking that, with no obstacles, the path is a straight line from current pose to `(goal_x, goal_y)`.
  - If the goal is off to the side, verify that VFH steers smoothly toward it instead of making an abrupt turn at the first cycle.

### 4. Ensure control node keeps altitude and doesn’t trigger LAND unintentionally

- **4.1. Confirm continuous setpoints**
  - From `_setpoint_loop()` in `control_node.py`, verify that when **no `/local_path` is received for a short period**, the node still calls `_send_position_target()` with the last known `current_pose` (this already exists, but verify it in logs).
  - If tests show gaps (no MAVLink setpoints) during obstacle encounters, add a guard to always send at least a "hold current position" setpoint at `target_altitude` when `takeoff_complete` is true, regardless of path state.
- **4.2. Watch modes and failsafes**
  - Using the MAVLink messages logged in `_mavlink_loop()`, check whether ArduPilot is switching to `LAND` / `RTL` at the moment of descent.
  - If yes, look at the SITL params (outside this repo) to relax any over-sensitive failsafe that may be triggered by temporary position loss.

### 5. End-to-end test matrix

Once the above changes / tunings are made, run a small test matrix:

- **Avoid-only mode (default)**
  - Straight wall ahead at ~3–4 m: drone should steer left or right while staying at `target_altitude`.
  - Single column obstacle: drone should slightly deviate and then recenter.
- **Goal mode (A to B)**
  - Goal directly behind a single obstacle: path should bend around and then realign with the goal line.
  - Goal through a narrow gap: VFH should pick the gap if it’s roughly aligned with the bearing.

For each scenario, verify:

- `/local_path` is always non-empty and updates smoothly.
- `/control/status` remains in `GUIDED` with the drone armed.
- Altitude stays close to `target_altitude` and the drone does not descend unless commanded (goal reached or explicit LAND).

## Todos

- **instrument-behavior**: Add/enable logs and use RViz/topic echo to observe `/local_path`, control status, and MAVLink mode at the instant an obstacle is detected.
- **tune-vfh**: Adjust VFH parameters and optionally add smoothing/hysteresis so that VFH always produces reasonable, non-empty waypoints with limited direction changes.
- **control-safety**: Confirm the control node always sends continuous setpoints (hold position when in doubt) and does not inadvertently stop sending when paths change.
- **scenario-tests**: Run a focused set of simulation scenarios (avoid-only and goal mode) to validate that the drone steers around obstacles instead of descending.


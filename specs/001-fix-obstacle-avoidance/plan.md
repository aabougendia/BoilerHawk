# Implementation Plan: Fix Obstacle Avoidance

**Branch**: `001-fix-obstacle-avoidance` | **Date**: 2026-02-25 | **Spec**: [spec.md](file:///home/zizo2004/BoilerHawk/specs/001-fix-obstacle-avoidance/spec.md)
**Input**: Feature specification from `/specs/001-fix-obstacle-avoidance/spec.md`

## Summary

The drone's obstacle avoidance is unreliable — the VFH direction picker has
coordinate bugs and the velocity setpoint pipeline produces circular or
oscillating flight. This plan fixes the VFH control loop by correcting the
grid-to-angle mapping, adding FOV-aware sector masking, implementing emergency
stop thresholds, and validating through SITL testing.

## Technical Context

**Language/Version**: Python 3.12, ROS 2 Jazzy
**Primary Dependencies**: rclpy, pymavlink, numpy, nav_msgs, geometry_msgs
**Storage**: N/A (real-time stream processing)
**Testing**: pytest (unit), ArduPilot SITL + Gazebo (integration)
**Target Platform**: Ubuntu 22.04+, ArduPilot Copter (SITL and hardware)
**Project Type**: ROS 2 multi-package drone autonomy system
**Performance Goals**: 20Hz velocity setpoints, ≤100ms VFH processing latency
**Constraints**: Altitude stability ±0.3m; collision-free 5-minute flights
**Scale/Scope**: Single drone, static obstacles, 5m depth camera range

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Safety-First Autonomy | ✅ PASS | Emergency stop at ≤1.5m, hover when blocked, SITL-first testing |
| II. Modular ROS 2 Packages | ✅ PASS | Changes scoped to `control` package; perception untouched |
| III. Simulation Before Hardware | ✅ PASS | All testing via `fly_autonomous.sh` + SITL + Gazebo |
| IV. Test Coverage | ✅ PASS | Existing VFH unit tests preserved; SITL integration validation |
| V. Observability & Diagnostics | ✅ PASS | Diagnostic timer logs heading, speed, obstacle distances, mode |

## Project Structure

### Documentation (this feature)

```text
specs/001-fix-obstacle-avoidance/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Phase 0: VFH coordinate analysis
├── data-model.md        # Phase 1: entity definitions
└── tasks.md             # Phase 2: task list (/speckit.tasks)
```

### Source Code (repository root)

```text
src/
├── control/
│   ├── control/
│   │   └── control_node.py      # PRIMARY: VFH + velocity setpoints + OBSTACLE_DISTANCE
│   ├── config/
│   │   └── control_params.yaml  # VFH params, speed tiers, thresholds
│   └── launch/
│       └── full_autonomous.launch.py  # perception + control launch
├── perception/
│   └── perception/
│       └── perception.py        # READ-ONLY: depth → occupancy grid
├── planning/
│   └── planning/
│       ├── path_planner.py      # Reference: VFH algorithm (not used at runtime)
│       └── test/
│           └── test_path_planner.py  # Existing unit tests
└── sim_models/
    └── config/
        └── sitl_extra.parm      # ArduPilot BendyRuler + proximity params
```

**Structure Decision**: All changes are in the `control` package. The VFH is
embedded in `control_node.py` (single-node architecture). Perception is read-only.
Planning package is preserved but not launched in the autonomous pipeline.

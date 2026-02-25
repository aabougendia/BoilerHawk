<!--
Sync Impact Report
- Version change: 0.0.0 → 1.0.0 (initial ratification)
- Added sections: Core Principles (5), Safety & Hardware Constraints, Development Workflow, Governance
- Templates requiring updates:
  - plan-template.md ✅ no changes needed (Constitution Check section is generic)
  - spec-template.md ✅ no changes needed (requirements format compatible)
  - tasks-template.md ✅ no changes needed (phase structure compatible)
- Follow-up TODOs: none
-->

# BoilerHawk Constitution

## Core Principles

### I. Safety-First Autonomy

All autonomous flight behavior MUST prioritize safety over mission objectives.
The system MUST maintain stable altitude and attitude during all maneuvers.
Obstacle avoidance MUST be tested in SITL simulation before any real-world flight.
Failsafe behaviors (hover, RTL) MUST activate when sensor data is missing or stale.
Any code path that sends commands to the flight controller MUST include bounds checking.

### II. Modular ROS 2 Packages

Each subsystem (perception, planning, control, localization, sensors_interface, sim_models)
MUST be a self-contained ROS 2 package with its own `package.xml`, config, and tests.
Packages communicate exclusively via ROS 2 topics and services — no direct imports
across package boundaries for runtime logic.
Parameters MUST be declared in YAML config files, not hardcoded.

### III. Simulation Before Hardware

All flight behaviors MUST be validated in ArduPilot SITL + Gazebo before deployment.
The `fly_autonomous.sh` script MUST remain the single-command entry point for SITL testing.
ArduPilot parameters MUST be managed via `sitl_extra.parm` for reproducibility.
Simulation models and worlds live in the `sim_models` package.

### IV. Test Coverage

Unit tests MUST exist for all algorithmic components (path planning, VFH, grid processing).
Tests MUST pass via `pytest` before merging.
Integration testing is performed through SITL simulation runs with diagnostic logging.
New features MUST NOT break existing tests.

### V. Observability & Diagnostics

Every ROS 2 node MUST log its operational state at regular intervals (diagnostics timer).
Mode changes, failover events, and error conditions MUST be logged at WARN level or above.
MAVLink communication state (heartbeat age, message counts) MUST be visible in logs.
SITL logs MUST be captured to `/tmp/boilerhawk_logs/` for post-flight analysis.

## Safety & Hardware Constraints

- **Flight controller**: ArduPilot (Copter) via pymavlink over TCP/serial
- **Coordinate conventions**: ArduPilot uses NED; ROS 2 uses ENU. All conversions
  MUST be explicit and documented at the conversion boundary.
- **Target platform**: Ubuntu 22.04+, ROS 2 Jazzy, Python 3.12
- **Depth sensor**: Intel RealSense or Gazebo simulated depth camera
- **MAVLink**: All commands to ArduPilot MUST use pymavlink directly (no MAVROS dependency)
- **Altitude**: Target altitude MUST be configurable via `control_params.yaml`

## Development Workflow

1. **Branch**: Feature branches off `main`; descriptive names (e.g., `fix/vfh-oscillation`)
2. **Build**: `colcon build` MUST succeed with zero errors before committing
3. **Test**: `pytest` on all test files MUST pass
4. **SITL**: Run `./fly_autonomous.sh` and verify behavior via logs and Gazebo
5. **Review**: Code changes to control node or flight parameters require extra scrutiny
   due to safety implications

## Governance

This constitution is the authoritative reference for all BoilerHawk development decisions.
Amendments require documentation of the change rationale and a version bump.
All pull requests MUST verify compliance with the principles above.
Complexity beyond the current architecture MUST be justified against the Safety-First
and Modular principles.

**Version**: 1.0.0 | **Ratified**: 2026-02-25 | **Last Amended**: 2026-02-25

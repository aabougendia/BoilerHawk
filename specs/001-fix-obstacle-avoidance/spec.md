# Feature Specification: Fix Obstacle Avoidance

**Feature Branch**: `001-fix-obstacle-avoidance`
**Created**: 2026-02-25
**Status**: Draft
**Input**: User description: "obstacle avoidance isnt working properly, we need to fix it or recreate it if needed"

## Clarifications

### Session 2026-02-25

- Q: Which axis represents "forward" in the Gazebo depth camera pointcloud? → A: X = forward (Gazebo default). Grid columns = forward direction.
- Q: How should the system treat directions outside the camera's field of view? → A: Treat unknown sectors as blocked (conservative — only steer where camera confirms clear).
- Q: At what distance should the system switch from slow-steer to full stop? → A: Stop at ≤1.5m, slow-steer from 1.5m–5.0m (extra margin for braking distance).

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Drone Flies Forward Without Hitting Obstacles (Priority: P1)

When the drone is launched in avoid-only mode, it MUST fly forward at a steady
speed and smoothly steer around any obstacle in its path, maintaining stable
altitude throughout the maneuver.

**Why this priority**: This is the core use case — a drone that crashes into
obstacles or drops altitude during turns is unsafe and unusable.

**Independent Test**: Launch SITL with Gazebo world containing walls/boxes.
Drone takes off, flies forward, encounters obstacle, steers around it without
altitude loss or collision, and continues forward.

**Acceptance Scenarios**:

1. **Given** the drone is airborne at target altitude in avoid-only mode,
   **When** an obstacle appears within 5m ahead,
   **Then** the drone steers to a clear direction without losing more than 0.3m altitude.

2. **Given** the drone is steering around an obstacle,
   **When** the obstacle is cleared,
   **Then** the drone resumes a forward heading within 5 seconds.

3. **Given** the drone is airborne with no obstacles in range,
   **When** the path is clear,
   **Then** the drone flies straight ahead at cruising speed.

---

### User Story 2 - Drone Navigates from A to B Around Obstacles (Priority: P2)

When given a goal position, the drone MUST navigate to the goal while avoiding
any obstacles encountered along the way.

**Why this priority**: Goal-directed navigation builds on avoid-only mode and is
required for autonomous missions.

**Independent Test**: Set a goal 20m away with obstacles between start and goal.
Drone reaches within 1m of the goal without collisions.

**Acceptance Scenarios**:

1. **Given** a goal position is set and obstacles exist between the drone and goal,
   **When** the drone encounters an obstacle,
   **Then** the drone diverts, clears the obstacle, and resumes heading toward the goal.

2. **Given** the drone is navigating to a goal,
   **When** it arrives within the goal threshold,
   **Then** the drone hovers in place.

---

### User Story 3 - Stable Altitude During All Maneuvers (Priority: P1)

The drone MUST maintain stable altitude during turns, speed changes, and
obstacle avoidance maneuvers.

**Why this priority**: Altitude drops during turns cause crashes in low-altitude
flight and are a safety-critical issue.

**Independent Test**: Command the drone to make 45° and 90° turns. Measure altitude
deviation — it MUST stay within ±0.3m of target.

**Acceptance Scenarios**:

1. **Given** the drone is flying at target altitude,
   **When** a sharp turn (>30°) is commanded,
   **Then** altitude deviation MUST NOT exceed 0.3m.

2. **Given** the drone is flying at target altitude,
   **When** speed changes from cruising to turn speed,
   **Then** altitude remains stable within ±0.3m.

---

### Edge Cases

- What happens when the drone is surrounded by obstacles on all sides?
  → The drone MUST stop and hover rather than attempt to fly through obstacles.
- What happens when depth sensor data is temporarily unavailable?
  → The drone MUST hold position (zero velocity) until sensor data returns.
- What happens when an obstacle appears very close (≤1.5m)?
  → The drone MUST stop immediately (zero velocity) rather than attempt to steer.
  Slow-steer avoidance applies only when the nearest obstacle is between 1.5m and 5.0m.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST detect obstacles from depth camera data and determine
  free directions for navigation.
- **FR-002**: System MUST convert obstacle detections into smooth velocity commands
  that ArduPilot can follow without aggressive attitude changes.
- **FR-003**: System MUST reduce speed proportionally to the steering angle —
  sharper turns require slower speeds.
- **FR-004**: System MUST filter heading changes to prevent oscillation between
  directions (no rapid direction reversals).
- **FR-005**: System MUST maintain target altitude within ±0.3m during all maneuvers.
- **FR-006**: System MUST hover in place when no valid avoidance direction exists.
- **FR-007**: System MUST resume forward flight when obstacles are cleared.
- **FR-008**: System MUST log diagnostic information (heading, speed, obstacle distances)
  for post-flight analysis.
- **FR-009**: System MUST treat directions outside the depth camera's field of view
  as blocked — the drone MUST only steer into directions where the camera confirms clearance.
- **FR-010**: System MUST execute an emergency stop (zero velocity) when the nearest
  obstacle in the chosen direction is ≤1.5m. Speed-scaled steering applies only
  when obstacles are between 1.5m and 5.0m.

### Key Entities

- **Occupancy Grid**: 2D grid centered on the drone representing obstacle locations,
  updated from depth camera point cloud at ~5Hz. Grid columns correspond to the
  camera's X axis (forward); grid rows correspond to Y axis (lateral).
- **VFH Histogram**: Polar obstacle-density histogram used to identify free directions.
- **Velocity Setpoint**: NED velocity command sent to ArduPilot at 20Hz.
- **Heading Filter**: EMA low-pass filter applied to VFH output to prevent oscillation.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Drone navigates around obstacles in SITL without any collision in a
  5-minute continuous test flight.
- **SC-002**: Altitude deviation stays within ±0.3m of target during all turns and
  avoidance maneuvers.
- **SC-003**: Drone successfully avoids obstacles approaching from straight ahead,
  30°, and 45° angles.
- **SC-004**: No heading oscillation observed — direction changes are smooth and
  monotonic when avoiding a single obstacle.
- **SC-005**: Drone resumes forward heading within 5 seconds after clearing an obstacle.
- **SC-006**: All unit tests pass (existing planning tests + any new tests).

## Assumptions

- The depth camera provides a usable point cloud at ≥2Hz in the Gazebo simulation.
- ArduPilot SITL correctly simulates GUIDED mode velocity tracking.
- The occupancy grid from the perception node accurately represents obstacles
  within the camera's field of view.
- The drone operates at low altitude (1-3m) in environments with static obstacles.

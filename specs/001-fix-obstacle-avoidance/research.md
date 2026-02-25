# Research: Fix Obstacle Avoidance

## Grid Coordinate Convention

**Decision**: Camera X axis = forward. Grid columns = forward direction.
VFH angle 0 = +column direction. Use `atan2(-dr, dc)` for angle computation.

**Rationale**: Gazebo depth camera sensor outputs pointcloud in sensor frame
where X = forward (depth). The perception node maps `points[:, 0]` (X) to
grid columns. The original VFH planner in `path_planner.py` uses
`atan2(-dr, dc)` which gives angle 0 = +col = forward. An earlier bug used
`atan2(dc, -dr)` which rotated all angles 90°, causing circular flight.

**Alternatives considered**: ROS optical frame (Z = forward) — rejected because
Gazebo bridge outputs in sensor frame, not optical frame.

## FOV-Aware Sector Masking

**Decision**: Mark sectors outside the camera's ~87° FOV as blocked in the VFH
histogram. The drone only steers into directions where the camera has confirmed
clearance.

**Rationale**: With a forward-facing camera covering ~87° horizontally, sectors
outside this arc have no data. Treating them as free allows the drone to fly
blindly into unseen obstacles. Conservative masking is safer.

**Alternatives considered**: Optimistic (unknown = free) — rejected per
clarification Q2. Moderately risky (weighted penalty) — more complex, deferred
for future iteration.

## Emergency Stop Threshold

**Decision**: Full stop (zero velocity) when nearest obstacle ≤1.5m.
Speed-scaled steering applies only for obstacles between 1.5m and 5.0m.

**Rationale**: At 1.5 m/s max cruise speed, the drone needs ~1m to decelerate.
A 1.5m threshold provides margin for braking distance.

**Alternatives considered**: 1.0m (too close, insufficient braking margin),
2.0m (too conservative, limits maneuverability).

## Velocity Setpoint Architecture

**Decision**: Single-node architecture — VFH embedded in the control node.
No separate planning node for the avoidance pipeline.

**Rationale**: Eliminates inter-node coordinate transform bugs. The control node
already owns the MAVLink connection and has access to occupancy data.
The planning package's VFH is preserved for reference and future use but is not
launched in the autonomous pipeline.

**Alternatives considered**: Separate planning node with path topic — rejected
due to persistent coordinate bugs between grid frame and NED/ENU.
ArduPilot-native BendyRuler — rejected because it did not arm/take off reliably
in testing (Approach A failure).

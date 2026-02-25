# Data Model: Fix Obstacle Avoidance

## Entities

### OccupancyGrid (from perception)

| Field | Type | Description |
|-------|------|-------------|
| grid | int8[H×W] | 2D array. 0 = free, 100 = occupied. |
| resolution | float | Metres per cell (default 0.1) |
| width, height | int | Grid dimensions (default 100×100) |
| origin | (float, float) | World position of grid corner |

**Convention**: Columns = camera X (forward), Rows = camera Y (lateral).
Grid center = drone position.

### VFH Histogram

| Field | Type | Description |
|-------|------|-------------|
| sectors | float[72] | Obstacle density per 5° angular bin |
| sector_width | float | 2π / 72 ≈ 0.087 rad |

**Angle convention**: Sector 0 = forward (+column). Angles measured counter-clockwise.
`atan2(-dr, dc)` maps grid offsets to angles.

### Velocity Setpoint

| Field | Type | Description |
|-------|------|-------------|
| vn | float | North velocity (m/s, NED) |
| ve | float | East velocity (m/s, NED) |
| vd | float | Down velocity (m/s, NED, negative = up) |

**Conversion**: Body-frame heading → NED via `ned_heading = current_yaw + body_heading`.

### Control State

| Field | Type | Description |
|-------|------|-------------|
| vfh_heading | float | EMA-filtered body-frame heading (rad) |
| current_yaw | float | NED yaw from ArduPilot ATTITUDE message |
| takeoff_complete | bool | True when altitude ≥ 85% of target |
| nearest_obstacle_m | float | Distance to closest obstacle in chosen direction |

## State Transitions

```text
INIT → CONNECTING → GUIDED_MODE → ARMED → TAKING_OFF → FLYING
                                                         ├── STEERING (obstacle 1.5m–5m)
                                                         ├── STOPPED (obstacle ≤1.5m)
                                                         └── HOVERING (no valid direction)
```

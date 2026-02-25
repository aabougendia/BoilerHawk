# Tasks: Fix Obstacle Avoidance

**Input**: Design documents from `/specs/001-fix-obstacle-avoidance/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, quickstart.md ✅

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)

---

## Phase 1: Setup

**Purpose**: Ensure build environment and configuration are correct

- [x] T001 Verify `colcon build --packages-select control sim_models` succeeds with current code
- [x] T002 [P] Verify `src/sim_models/config/sitl_extra.parm` contains BendyRuler + proximity params (PRX1_TYPE=2, OA_TYPE=1, AVOID_ENABLE=7) — add any missing entries

**Checkpoint**: Build passes, SITL params ready ✅

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Fix the VFH core so all user stories have a working direction picker

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T003 Verify VFH angle computation in `src/control/control/control_node.py` uses `atan2(-dr, dc)` so angle 0 = forward (+column direction) — fix if incorrect
- [x] T004 Add FOV-aware sector masking in `_run_vfh()` in `src/control/control/control_node.py` — mark sectors outside the camera's ~87° horizontal FOV as blocked (high histogram value) so the drone only steers into confirmed-clear directions
- [x] T005 Add nearest-obstacle distance calculation in `_run_vfh()` in `src/control/control/control_node.py` — raycast along the chosen VFH heading to find the distance to the nearest obstacle in that direction, store as `self._nearest_obstacle_m`
- [x] T006 Implement emergency stop logic in `_setpoint_loop()` in `src/control/control/control_node.py` — if `nearest_obstacle_m ≤ 1.5m`, send zero velocity (full stop); if between 1.5m–5.0m, use speed-scaled steering; if >5m, cruise at max speed
- [x] T007 Add hover-when-blocked logic in `_setpoint_loop()` in `src/control/control/control_node.py` — if VFH finds no valid free direction (all sectors blocked), send zero velocity and log "hovering — all directions blocked"
- [x] T008 Add sensor-timeout safety in `_setpoint_loop()` in `src/control/control/control_node.py` — if no occupancy grid received for >2 seconds, send zero velocity and log "hovering — no sensor data"

**Checkpoint**: VFH correctly picks forward direction, respects FOV limits, stops for close obstacles ✅

---

## Phase 3: User Story 1 — Drone Flies Forward Avoiding Obstacles (Priority: P1) 🎯 MVP

**Goal**: Drone flies forward at steady speed, smoothly steers around obstacles, maintains altitude

**Independent Test**: Launch `./fly_autonomous.sh`, drone takes off, flies forward, encounters wall/box, steers around it, continues forward. No collision, altitude stable ±0.3m.

### Implementation for User Story 1

- [x] T009 [US1] Tune VFH parameters in `src/control/config/control_params.yaml` — set `valley_threshold`, `safety_radius_cells`, `heading_ema_alpha` for smooth obstacle reaction
- [x] T010 [US1] Tune speed tier parameters in `src/control/config/control_params.yaml` — set `max_speed=1.5`, `turn_speed_medium=0.8`, `turn_speed_sharp=0.3` for gentle turns
- [x] T011 [US1] Verify heading-to-NED conversion in `_setpoint_loop()` in `src/control/control/control_node.py` — confirm `ned_heading = current_yaw + body_heading` produces correct NED velocity for forward flight and turns
- [x] T012 [US1] Verify altitude correction in `_setpoint_loop()` in `src/control/control/control_node.py` — confirm `vd = -(target_alt - current_alt) * gain` keeps altitude within ±0.3m
- [ ] T013 [US1] Run SITL integration test: `./fly_autonomous.sh` with Gazebo obstacle world, observe drone avoids obstacles and flies forward for 2+ minutes without collision. Verify heading changes are smooth with no rapid reversals (FR-004) and drone resumes forward after clearing obstacles (FR-007)

**Checkpoint**: US1 complete — drone flies forward avoiding obstacles with stable altitude

---

## Phase 4: User Story 3 — Stable Altitude During Maneuvers (Priority: P1)

**Goal**: Altitude stays within ±0.3m during sharp turns and speed changes

**Independent Test**: Command 45° and 90° turns via VFH heading changes, measure altitude deviation in logs.

### Implementation for User Story 3

- [x] T014 [US3] Tune ArduPilot parameters in `src/sim_models/config/sitl_extra.parm` — verify `ATC_ANGLE_BOOST=1`, `WPNAV_ACCEL=100`, `WPNAV_JERK=5` prevent altitude loss during aggressive attitude changes
- [x] T015 [US3] Verify speed-scaling reduces velocity during turns in `_speed_for_steer()` in `src/control/control/control_node.py` — confirm all 3 tiers work: <15°→1.5m/s, 15-35°→0.8m/s, >35°→0.3m/s (FR-003). Check in SITL logs that speed drops during turns
- [ ] T016 [US3] Run SITL altitude stability test: fly through multiple obstacles, check logs for altitude deviation — MUST stay within ±0.3m

**Checkpoint**: US3 complete — altitude stable during all turns and speed changes

---

## Phase 5: User Story 2 — Navigate A→B Around Obstacles (Priority: P2)

**Goal**: Drone navigates to a goal position, steering around obstacles en route

**Independent Test**: Set `avoid_only=false`, `goal_x=10.0`, `goal_y=0.0` with obstacles between start and goal. Drone reaches within 1m of goal.

### Implementation for User Story 2

- [x] T017 [US2] Implement goal-bearing computation in `_occupancy_callback()` in `src/control/control/control_node.py` — when `avoid_only=false`, compute bearing to `(goal_x, goal_y)` in body frame and pass as `preferred_angle` to VFH
- [x] T018 [US2] Implement goal-reached detection in `_setpoint_loop()` in `src/control/control/control_node.py` — when distance to `(goal_x, goal_y)` < 1.0m, switch to hover (zero velocity) and log "goal reached"
- [x] T019 [US2] Add goal parameters to `src/control/config/control_params.yaml` — `goal_x`, `goal_y`, `goal_reached_threshold: 1.0`
- [ ] T020 [US2] Run SITL goal navigation test: set a goal 20m away with obstacles, verify drone reaches goal within 1m and hovers

**Checkpoint**: US2 complete — goal-directed navigation with obstacle avoidance

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and cleanup

- [x] T021 [P] Update diagnostic logging in `_diagnostics_loop()` in `src/control/control/control_node.py` — add `nearest_obstacle_m`, `speed`, `steer_deg` to diagnostics output
- [x] T022 [P] Update `src/control/launch/full_autonomous.launch.py` — verify perception + control launch works with all new params
- [ ] T023 Run full 5-minute SITL flight test per SC-001: no collisions, altitude ±0.3m, smooth heading changes
- [x] T024 Verify all existing unit tests pass: `cd src/planning && python -m pytest test/`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies — start immediately
- **Foundational (Phase 2)**: Depends on Setup — BLOCKS all user stories
- **US1 (Phase 3)**: Depends on Foundational — MVP
- **US3 (Phase 4)**: Depends on US1 (shares velocity/altitude pipeline)
- **US2 (Phase 5)**: Depends on Foundational — can proceed in parallel with US3
- **Polish (Phase 6)**: Depends on all stories complete

### Parallel Opportunities

- T001, T002 can run in parallel (setup)
- T009, T010 can run in parallel (config tuning)
- T021, T022 can run in parallel (polish)
- US2 and US3 can proceed in parallel after US1

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (VFH fixes — CRITICAL)
3. Complete Phase 3: User Story 1 (avoid obstacles)
4. **STOP and VALIDATE**: 2-minute SITL flight, no collisions
5. If passing, proceed to US3 then US2

### Incremental Delivery

1. Setup + Foundational → VFH works correctly
2. US1 → Drone avoids obstacles (MVP!)
3. US3 → Altitude stability confirmed
4. US2 → Goal navigation works
5. Polish → Full 5-minute validation

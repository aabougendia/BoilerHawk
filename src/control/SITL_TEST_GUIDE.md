# SITL Testing Guide for BoilerHawk Control Node

## Overview

This guide provides complete instructions for testing the BoilerHawk control node with ArduPilot SITL (Software-In-The-Loop). It includes step-by-step procedures, common issues encountered, and their solutions based on actual testing experience.

---

## Prerequisites

### Required Software
- ✅ ArduPilot SITL installed (`~/ardupilot/ArduCopter`)
- ✅ ROS 2 (Jazzy or compatible)
- ✅ MAVROS for ROS 2
- ✅ BoilerHawk packages built (perception, control)

### Verify Installation
```bash
# Check ArduPilot SITL
which sim_vehicle.py

# Check ROS 2 packages
ros2 pkg list | grep -E '(mavros|control|perception)'

# Verify build status
cd ~/BoilerHawk
colcon build --packages-select control perception
```

---

## Step-by-Step Testing Procedure

### Terminal 1: Launch ArduPilot SITL

**WSL Users - IMPORTANT:**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map --out=127.0.0.1:14550
```

**Non-WSL Users:**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py --console --map
```

**Expected Output:**
```
MAV> Detected vehicle 1:1 on link 0
STABILIZE> Received 1363 parameters (ftp)
```

**Issue #1: MAVROS Connection Fails in WSL**
- **Symptom**: MAVROS shows `connected: false`
- **Cause**: SITL outputs to Windows host IP (172.29.x.x) instead of localhost
- **Solution**: Use `--out=127.0.0.1:14550` flag ✅
- **Verification**: See "MAVROS connection successful" messages

---

### Terminal 2: Launch MAVROS

```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555
```

**Expected Output:**
```
[mavros_node] FCU: ArduCopter V4.7.0-dev
[mavros_node] VER: 1.1: Flight software: 04070000
[mavros_node] FCU: Frame: QUAD/PLUS
[mavros_node] PR: parameters list received
```

**Verify Connection:**
```bash
ros2 topic echo /mavros/state --once
```

**Expected:**
```yaml
connected: true
mode: GUIDED
```

---

### Terminal 3: Launch Full System

```bash
cd ~/BoilerHawk
source install/setup.bash
ros2 launch control full_system.launch.py
```

**Expected Output:**
```
[mock_planning_node] Mock planning node initialized
[perception_node] Perception node initialized
[control_node] Control node initialized
[control_node] Setpoint rate: 20.0Hz
```

**Verify System:**
```bash
# Check control status
ros2 topic echo /control/status --once

# Verify setpoint rate (should be ~20Hz)
ros2 topic hz /mavros/setpoint_position/local --window 10
```

---

### Terminal 4: RViz2 Visualization (Optional)

```bash
cd ~/BoilerHawk
source install/setup.bash
rviz2 -d src/control/config/control_viz.rviz
```

**What You'll See:**
- 🔵 Drone position (axes at origin ~0,0,0)
- 🟡 Setpoint target (axes at current waypoint ~2,2,2)
- 🟢 Global path (green line)
- 🔴 Local path (red line, 20 waypoints)

---

## Flight Testing Sequence

### 1. Set RC Throttle (Critical!)

**In MAVProxy (Terminal 1):**
```
rc 3 1500
```

**Issue #2: Instant Disarm After Arming**
- **Symptom**: Drone arms but immediately disarms after 1-2 seconds
- **Cause**: RC throttle failsafe - throttle at minimum triggers auto-disarm
- **Solution**: Set RC channel 3 (throttle) to mid-position (1500) ✅
- **Why**: ArduPilot expects valid RC input; minimum throttle = failsafe condition

**Verification:**
Monitor armed status stays true:
```bash
ros2 topic echo /mavros/state | grep armed
```

---

### 2. Arm the Drone

**Method 1: Via MAVROS Service (Recommended)**
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

**Method 2: Via MAVProxy**
```
arm throttle
```

**Expected Response:**
```
response: success=True, result=0
```

**Verify:**
```bash
ros2 topic echo /mavros/state --once | grep armed
# Output: armed: true
```

**Monitor Control Node:**
```bash
ros2 topic echo /control/status --once
# Output: Status: Following path - Waypoint: 1/20 - Mode: GUIDED - Armed: True
```

---

### 3. Verify Setpoint Publishing

**Check Setpoint Rate:**
```bash
ros2 topic hz /mavros/setpoint_position/local --window 10
```

**Expected:**
```
average rate: 20.000
    min: 0.050s max: 0.050s std dev: 0.00020s
```

**Check Setpoint Content:**
```bash
ros2 topic echo /mavros/setpoint_position/local --once
```

**Expected:**
```yaml
pose:
  position:
    x: 2.0  # Current waypoint
    y: 2.0
    z: 2.0  # target_altitude parameter
```

---

### 4. Test Takeoff

**In MAVProxy:**
```
takeoff 2
```

**Expected:**
```
GUIDED> Take Off started
```

**Monitor Position:**
```bash
ros2 topic echo /mavros/local_position/pose
```

**Watch for altitude increase:**
```
position:
  z: 0.5 → 1.0 → 1.5 → 2.0  # Should climb to 2m
```

---

### 5. Monitor Waypoint Following

**Watch Control Logs:**
The control node logs will show waypoint progression:
```
[control_node] Received new path with 20 waypoints
[control_node] Target waypoint 1: (2.00, 2.00, 2.00)
[control_node] Waypoint reached! Advancing to waypoint 2
[control_node] Target waypoint 2: (2.20, 2.20, 2.00)
```

**Monitor Position Updates:**
```bash
ros2 topic echo /mavros/local_position/pose | grep -A3 "position:"
```

**Expected Behavior:**
- Drone should move toward waypoint (2.0, 2.0, 2.0)
- When within 0.5m (waypoint_threshold), advances to next waypoint
- Continues through all 20 waypoints in path

---

## System Architecture

```
Mock Planning Node → Occupancy Grids → Perception Node → A* Planning
                                              ↓
                                         /local_path
                                              ↓
                               Control Node (20Hz setpoints)
                                              ↓
                                  /mavros/setpoint_position/local
                                              ↓
                                          MAVROS
                                              ↓ MAVLink
                                       ArduPilot SITL
                                              ↓
                                  /mavros/local_position/pose
```

---

## Common Issues & Solutions

### Issue #1: MAVROS Not Connecting (WSL)

**Symptoms:**
```yaml
connected: false
armed: false
mode: ''
```

**Diagnosis:**
```bash
# Check SITL output address
# Look for: --out 172.29.x.x:14550  (wrong for WSL)
```

**Solution:**
```bash
# Restart SITL with localhost output:
sim_vehicle.py --console --map --out=127.0.0.1:14550
```

**Why:** WSL networking routes `172.29.x.x` to Windows host, but MAVROS runs in WSL and needs `127.0.0.1`

---

### Issue #2: Instant Disarm After Arming

**Symptoms:**
- Drone arms successfully
- Disarms after 1-2 seconds automatically
- MAVROS oscillates: `armed: true → false → true → false`

**Diagnosis:**
Check ArduPilot messages for "Failsafe" or "Throttle below minimum"

**Solution:**
```bash
# In MAVProxy:
rc 3 1500
```

**Why:** ArduPilot RC failsafe triggers when throttle < minimum threshold. Setting RC ch 3 to 1500 (mid-position) satisfies the check.

**Permanent Fix (Optional):**
```bash
# Disable RC failsafe in SITL (use with caution):
param set FS_THR_ENABLE 0
```

---

### Issue #3: RViz QoS Warnings

**Symptoms:**
```
[WARN] New publisher discovered on topic '/mavros/local_position/pose', 
       offering incompatible QoS. RELIABILITY_QOS_POLICY
```

**Solution:** Configure RViz topics with BEST_EFFORT reliability
- Already handled in provided `control_viz.rviz` config ✅

**Why:** MAVROS uses BEST_EFFORT QoS (for real-time data), RViz defaults to RELIABLE

---

### Issue #4: Drone Not Moving Despite Setpoints

**Symptoms:**
- ✅ Drone armed and staying armed
- ✅ Setpoints publishing at 20Hz
- ✅ GUIDED mode active
- ❌ Position unchanged (z ≈ 0)

**Possible Causes:**

1. **EKF Not Ready**
   ```bash
   # Check EKF status in MAVProxy
   # Look for: "EKF3 IMU0 is using GPS"
   ```

2. **GPS Lock Not Acquired**
   ```bash
   # SITL should auto-acquire GPS in ~5 seconds
   # Check: GUIDED mode should only be available with GPS
   ```

3. **Position Control Not Enabled**
   - ArduPilot might need explicit takeoff before accepting position commands
   - Try MAVROS takeoff service:
   ```bash
   ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "{altitude: 2.0}"
   ```

---

## Configuration Parameters

### Control Node (`control_params.yaml`)

```yaml
control_node:
  ros__parameters:
    waypoint_threshold: 0.5      # Distance to reach waypoint (m)
    setpoint_rate: 20.0          # Publishing rate (Hz)
    auto_arm: false              # Auto-arm on start
    auto_mode_switch: true       # Auto switch to GUIDED
    target_altitude: 2.0         # Flight altitude (m)
```

**Tuning Tips:**
- **waypoint_threshold**: Smaller = more precise, larger = faster progression
- **setpoint_rate**: Must be ≥10Hz for GUIDED mode, 20Hz recommended
- **target_altitude**: Overrides Z component from planning paths

---

## Monitoring Commands

### Quick Status Check
```bash
# All-in-one status check
echo "=== MAVROS State ===" && \
ros2 topic echo /mavros/state --once && \
echo "=== Control Status ===" && \
ros2 topic echo /control/status --once && \
echo "=== Setpoint Rate ===" && \
timeout 3 ros2 topic hz /mavros/setpoint_position/local
```

### Continuous Monitoring
```bash
# Watch drone position
watch -n 1 'ros2 topic echo /mavros/local_position/pose --once | grep -A3 position'

# Monitor control node logs
ros2 topic echo /control/status
```

### Performance Metrics
```bash
# Setpoint publishing rate (should be 20Hz)
ros2 topic hz /mavros/setpoint_position/local --window 10

# Path update rate
ros2 topic hz /local_path

# Check topic list
ros2 topic list | grep -E '(mavros|control|path)'
```

---

## Expected Timeline

1. **T+0s**: Launch SITL → Boots in ~5 seconds
2. **T+5s**: Launch MAVROS → Connects in ~2 seconds
3. **T+10s**: Launch full system → All nodes ready
4. **T+15s**: Set RC throttle → Prevents failsafe
5. **T+20s**: Arm drone → Stays armed continuously
6. **T+25s**: Takeoff command → Should climb to 2m
7. **T+30s+**: Waypoint following → Progressive movement

---

## Troubleshooting Checklist

- [ ] SITL running with `--out=127.0.0.1:14550` (WSL)
- [ ] MAVROS shows `connected: true`
- [ ] Control node publishing at 20Hz
- [ ] RC throttle set to 1500
- [ ] Drone armed and **staying armed** (key indicator!)
- [ ] GUIDED mode active
- [ ] EKF initialized ("EKF3 IMU0 is using GPS")
- [ ] Setpoints visible in RViz2
- [ ] No error messages in logs

---

## Success Criteria

✅ **Minimum Success** (Control Node Verified):
- MAVROS connected
- Control node publishing setpoints at 20Hz
- Drone arms and stays armed
- Setpoints visible in RViz2

✅ **Full Success** (Flight Verified):
- Drone takes off to target altitude
- Moves toward waypoints
- Waypoint advancement logic working
- Completes path following

---

## Quick Reference Commands

### Stop Everything
```bash
# Kill all ROS nodes
killall -9 ros2

# Stop SITL (Ctrl+C in MAVProxy terminal)
```

### Restart Clean
```bash
# Kill existing processes
killall -9 sim_vehicle.py mavproxy.py

# Clear old logs
rm ~/ardupilot/ArduCopter/*.tlog*
rm ~/ardupilot/ArduCopter/mav.parm.old

# Restart from Terminal 1
```

---

## Next Steps

After successful SITL testing:

1. **Parameter Tuning**: Adjust `waypoint_threshold`, `target_altitude`
2. **Path Testing**: Modify perception start/goal for different routes
3. **Failure Testing**: Test obstacle avoidance, lost paths
4. **Hardware Deploy**: Transfer to real drone with physical MAVROS connection

---

## Contact & Support

For issues or questions:
- Check ArduPilot docs: https://ardupilot.org/copter/
- MAVROS wiki: http://wiki.ros.org/mavros
- BoilerHawk team: aabugendia@gmail.com

**Last Updated**: 2025-11-26  
**Tested With**: ArduPilot V4.7.0-dev, ROS 2 Jazzy, MAVROS 2.x

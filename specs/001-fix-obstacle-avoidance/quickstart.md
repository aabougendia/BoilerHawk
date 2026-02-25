# Quickstart: Fix Obstacle Avoidance

## Prerequisites

- Ubuntu 22.04+ with ROS 2 Jazzy installed
- ArduPilot SITL (sim_vehicle.py) compiled
- Gazebo Harmonic with depth camera world
- Python 3.12 with pymavlink, numpy

## Build

```bash
cd ~/BoilerHawk
colcon build --packages-select control sim_models
source install/setup.bash
```

## Run

```bash
./fly_autonomous.sh
```

This launches Gazebo → ArduPilot SITL → Perception + Control.
The drone arms, takes off to 1m, and flies forward avoiding obstacles.

## Verify

1. Watch Gazebo: drone should steer around obstacles without collision
2. Check logs: `tail -f /tmp/boilerhawk_logs/control.log`
3. Look for `[DIAG]` lines showing heading changes and obstacle distances
4. Confirm altitude stays within ±0.3m of target

## Configuration

Edit `src/control/config/control_params.yaml`:
- `target_altitude`: flight altitude (default 1.0m)
- `max_speed`: cruise speed (default 1.5 m/s)
- `heading_ema_alpha`: heading smoothing (0.3 = moderate filter)
- `valley_threshold`: VFH sensitivity (0.5 = default)

## Stop

Press `Ctrl+C` in the terminal running `fly_autonomous.sh`.

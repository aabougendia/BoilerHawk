# BoilerHawk Drone Simulation Setup

This guide helps you set up a complete drone simulation environment with:
- **Gazebo Harmonic** for realistic 3D simulation
- **ArduPilot SITL** for flight controller emulation  
- **ROS 2 Jazzy** for sensor data and control interfaces
- **Depth camera** mounted on quadcopter for perception

## Quick Start

### 1. Install ArduPilot + Gazebo Integration

Run the automated setup script:

```bash
cd ~/boilerHawk_ws
./setup_ardupilot.sh
```

This will:
- Install Gazebo Harmonic dependencies
- Clone and build the ArduPilot Gazebo plugin
- Configure environment variables
- Set up drone models (Iris quadcopter)

**Then restart your terminal** or run: `source ~/.bashrc`

### 2. Install ArduPilot SITL (if not already installed)

```bash
cd ~
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

# Install dependencies
Tools/environment_install/install-prereqs-ubuntu.sh -y

# Reload environment
. ~/.profile

# Build for Copter
./waf configure --board sitl
./waf copter
```

### 3. Test Basic Setup

#### Terminal 1: Launch Gazebo with Iris drone
```bash
source ~/.bashrc
gz sim iris_runway.sdf
```

#### Terminal 2: Launch ArduPilot SITL
```bash
cd ~/ardupilot
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

#### Terminal 3: Control the drone
```
mode guided
arm throttle
takeoff 5
```

## Integration with BoilerHawk

### Option 1: Simple Depth Camera (Current Working Setup)

Launch everything in one command:

```bash
cd ~/boilerHawk_ws
./launch_depth_cam.sh
```

This launches:
- Gazebo with depth camera
- ROS 2 bridges for camera data
- RViz visualization
- TF transforms

### Option 2: Drone + Depth Camera (In Development)

This will be available after completing the drone integration:

```bash
cd ~/boilerHawk_ws
./launch_drone_sim.sh
```

Will launch:
- Gazebo with outdoor world + Iris quadcopter + mounted depth camera
- ArduPilot SITL
- ROS 2 bridges for camera AND drone sensors (IMU, GPS, battery, odometry)
- RViz with point cloud + drone pose
- Control interface nodes

## Project Structure

```
~/boilerHawk_ws/
├── src/
│   ├── sim_models/              # Gazebo models and worlds
│   │   ├── models/
│   │   │   ├── depth_cam/       # RGB-D camera sensor
│   │   │   └── (drone models will be added here)
│   │   ├── worlds/
│   │   │   ├── main_world.sdf   # Simple test world
│   │   │   └── outdoor_world.sdf # Realistic outdoor environment
│   │   └── launch/
│   │       └── depth_camera_sim.launch.py
│   ├── sensors_interface/       # ROS 2 sensor nodes
│   ├── control/                 # Control nodes (manual, ArduPilot interface)
│   ├── perception/              # Future: object detection, SLAM
│   ├── planning/                # Future: path planning
│   └── localization/            # Future: localization algorithms
├── setup_ardupilot.sh          # ArduPilot setup automation
├── launch_depth_cam.sh         # Launch depth camera simulation
└── launch_drone_sim.sh         # Launch full drone simulation (coming soon)

~/gz_ws/                         # ArduPilot Gazebo workspace
└── src/
    └── ardupilot_gazebo/        # ArduPilot plugin and drone models
        ├── models/
        │   ├── iris/            # Iris quadcopter
        │   └── zephyr/          # Zephyr fixed-wing
        └── worlds/
            ├── iris_runway.sdf
            └── zephyr_runway.sdf
```

## Control Package (Coming Soon)

The `control` package will provide:

1. **Manual Control Node** (`manual_control_node.py`)
   - Keyboard teleop: WASD for movement, Q/E for yaw, +/- for altitude
   - Gamepad support (Xbox/PS4 controller)
   - Published to `/cmd_vel` or MAVLink commands

2. **ArduPilot Interface Node** (`ardupilot_interface_node.py`)
   - MAVLink communication with SITL
   - ROS 2 service interface for missions
   - Status monitoring and telemetry

3. **Autonomous Mission Node** (`mission_node.py`)
   - Waypoint following
   - Obstacle avoidance using depth camera
   - Integration with planning package

## Environment Variables

These are set automatically by `setup_ardupilot.sh`:

```bash
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/gz_ws/src/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
export GZ_SIM_RESOURCE_PATH=$HOME/gz_ws/src/ardupilot_gazebo/models:$HOME/gz_ws/src/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_RESOURCE_PATH=/opt/ros/jazzy/share:$HOME/boilerHawk_ws/src/sim_models/models:${GZ_SIM_RESOURCE_PATH}
```

## Troubleshooting

### Gazebo can't find drone models
```bash
# Verify environment variables
echo $GZ_SIM_RESOURCE_PATH
# Should include ~/gz_ws/src/ardupilot_gazebo/models

# Source your bashrc
source ~/.bashrc
```

### ArduPilot SITL won't connect
```bash
# Check Gazebo plugin is loaded
gz sim iris_runway.sdf --verbose 4 | grep -i ardupilot

# Verify JSON interface port
netstat -an | grep 9002
```

### ROS 2 topics not appearing
```bash
# Source workspace
cd ~/boilerHawk_ws
source install/setup.bash

# Check bridges are running
ros2 node list | grep bridge

# List available topics
ros2 topic list
```

### Point cloud shows NaN values
- Ensure depth camera `<depth_camera>` section exists in sensor config
- Check camera orientation (yaw) is pointing at objects
- Verify objects are within clip range (0.1m - 20m)

## Next Steps

### Immediate Tasks (Manual Implementation)
1. ✅ Create realistic outdoor world
2. ⏳ Integrate Iris drone model into BoilerHawk workspace
3. ⏳ Mount depth camera on drone
4. ⏳ Create control package nodes
5. ⏳ Test full drone + camera simulation

### Future Enhancements
- Add gimbal for camera stabilization
- Implement object detection pipeline (YOLOv8 + depth)
- Add SLAM for autonomous navigation
- Multi-drone swarm simulation
- Competition scenarios (racing gates, delivery missions)

## Resources

- [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [ArduPilot SITL Documentation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
- [ROS 2 Jazzy Docs](https://docs.ros.org/en/jazzy/)
- [Gazebo Fuel Models](https://app.gazebosim.org/fuel/models)

## Support

For questions or issues:
- GitHub Issues: [Create issue in BoilerHawk repository]
- ArduPilot Forums: https://discuss.ardupilot.org/
- ROS Answers: https://answers.ros.org/

# BoilerHawk: MAVROS → PyMAVLink Migration Plan

> **Goal**: Replace MAVROS with PyMAVLink for direct MAVLink communication with ArduPilot SITL, enabling a reliable demo flight in Gazebo Harmonic.

---

## Table of Contents

1. [Why Migrate](#1-why-migrate)
2. [Architecture: Before vs After](#2-architecture-before-vs-after)
3. [Step-by-Step Implementation](#3-step-by-step-implementation)
   - [Step 1: Install pymavlink](#step-1-install-pymavlink)
   - [Step 2: Rewrite `control_node.py`](#step-2-rewrite-control_nodepy)
   - [Step 3: Update config / params](#step-3-update-config--params)
   - [Step 4: Update `package.xml` & `setup.py`](#step-4-update-packagexml--setuppy)
   - [Step 5: Rewrite `fly.sh`](#step-5-rewrite-flysh)
   - [Step 6: Update launch files](#step-6-update-launch-files)
4. [Files Deleted / Deprecated](#4-files-deleted--deprecated)
5. [Demo Flight Sequence](#5-demo-flight-sequence)
6. [Testing Checklist](#6-testing-checklist)
7. [Rollback Plan](#7-rollback-plan)

---

## 1. Why Migrate

| Problem with MAVROS | How PyMAVLink Fixes It |
|---|---|
| `create_subscription` crash when launched via `ros2 launch` | No ROS 2 subscriptions for MAVLink — pymavlink uses raw TCP/UDP sockets |
| Extremely slow param download (1 param/3-4s × 1397 params = ~70 min) | We skip param download entirely; we set params via `--defaults` in SITL |
| Service call timeouts for arming/mode/takeoff | Direct MAVLink command messages — instant, no middleware |
| Requires `mavros_msgs` package dependency | Zero extra ROS 2 dependencies — pymavlink is a pure Python pip package |
| Complex plugin system with config YAML | One `mavutil.mavlink_connection()` call |
| Respawn loops when MAVROS crashes | No separate process to crash — MAVLink runs inside the control node |

---

## 2. Architecture: Before vs After

### Before (MAVROS)
```
┌─────────────┐   TCP:5760   ┌─────────┐  ROS 2 Topics/Services  ┌──────────────┐
│ ArduPilot   │◄────────────►│ MAVROS  │◄────────────────────────►│ control_node │
│ SITL        │   MAVLink    │ (node)  │  /mavros/state           │              │
│ (separate)  │              │(separate│  /mavros/local_position  │              │
│             │              │ process)│  /mavros/cmd/arming      │              │
└─────────────┘              └─────────┘  /mavros/set_mode        └──────────────┘
                                          /mavros/cmd/takeoff
                                          /mavros/setpoint_position
                                          /mavros/rc/override
```

### After (PyMAVLink)
```
┌─────────────┐   TCP:5760   ┌──────────────┐
│ ArduPilot   │◄────────────►│ control_node │
│ SITL        │   MAVLink    │  (pymavlink  │
│             │   (direct)   │   inside)    │
└─────────────┘              └──────────────┘
```

**Key insight**: The control node talks to ArduPilot directly. No MAVROS process, no bridging, no service calls, no plugin configs.

---

## 3. Step-by-Step Implementation

### Step 1: Install pymavlink

```bash
pip install pymavlink
```

Verify:
```bash
python3 -c "from pymavlink import mavutil; print('pymavlink OK')"
```

---

### Step 2: Rewrite `control_node.py`

**File**: `src/control/control/control_node.py`

This is the core change. The new node:
- Opens a direct MAVLink TCP connection to ArduPilot SITL
- Runs a 50 Hz `_mavlink_loop()` timer to receive heartbeats, position, and state
- Sends `SET_MODE`, `COMMAND_LONG` (arm/takeoff), and `SET_POSITION_TARGET_LOCAL_NED` directly
- Still subscribes to `/local_path` (ROS 2, from planning) and publishes `/control/status` (ROS 2)
- Publishes `/mavros/local_position/pose` so RViz and other nodes still get position data

#### MAVROS → PyMAVLink API Mapping

| MAVROS (old) | PyMAVLink (new) |
|---|---|
| `Sub('/mavros/state')` → `State` msg | `recv_match(type='HEARTBEAT')` → parse `.base_mode`, `.custom_mode` |
| `Sub('/mavros/local_position/pose')` → `PoseStamped` | `recv_match(type='LOCAL_POSITION_NED')` → build `PoseStamped` & publish |
| `Pub('/mavros/setpoint_position/local')` | `mav.set_position_target_local_ned_send()` |
| `Pub('/mavros/rc/override')` | Not needed — failsafes disabled via `sitl_extra.parm` |
| `Srv('/mavros/set_mode')` | `mav.set_mode_send()` or `mav.command_long_send(MAV_CMD_DO_SET_MODE)` |
| `Srv('/mavros/cmd/arming')` | `mav.command_long_send(MAV_CMD_COMPONENT_ARM_DISARM)` |
| `Srv('/mavros/cmd/takeoff')` | `mav.command_long_send(MAV_CMD_NAV_TAKEOFF)` |

#### New `control_node.py` — Full Implementation

```python
#!/usr/bin/env python3
"""
Control Node for BoilerHawk (PyMAVLink version).
Communicates directly with ArduPilot via MAVLink over TCP.
Receives paths from planning module and sends position commands.
"""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from pymavlink import mavutil


class ControlNode(Node):
    """
    ROS 2 node for drone control via direct MAVLink (pymavlink).
    """

    # ArduPilot GUIDED mode number
    GUIDED_MODE = 4
    # Copter modes from ArduPilot
    MODE_MAP = {0: 'STABILIZE', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED',
                5: 'LOITER', 6: 'RTL', 9: 'LAND', 16: 'POSHOLD'}

    def __init__(self):
        super().__init__('control_node')

        # ── Parameters ───────────────────────────────────────────────
        self.declare_parameter('waypoint_threshold', 0.5)
        self.declare_parameter('setpoint_rate', 20.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_mode_switch', True)
        self.declare_parameter('target_altitude', 2.0)
        self.declare_parameter('fcu_connection', 'tcp:127.0.0.1:5760')
        self.declare_parameter('mavlink_poll_rate', 50.0)

        self.waypoint_threshold = self.get_parameter('waypoint_threshold').value
        setpoint_rate = self.get_parameter('setpoint_rate').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_mode_switch = self.get_parameter('auto_mode_switch').value
        self.target_altitude = self.get_parameter('target_altitude').value
        fcu_connection = self.get_parameter('fcu_connection').value
        mavlink_poll_rate = self.get_parameter('mavlink_poll_rate').value

        # ── State variables ──────────────────────────────────────────
        self.current_path = None
        self.current_waypoint_idx = 0
        self.current_pose = None          # PoseStamped (NED → ENU converted)
        self.target_setpoint = None       # PoseStamped

        self.connected = False
        self.armed = False
        self.mode = 'UNKNOWN'
        self.takeoff_requested = False
        self.takeoff_complete = False

        # Counters
        self.diag_path_count = 0
        self.diag_setpoint_count = 0
        self.diag_heartbeat_count = 0

        # ── MAVLink connection ───────────────────────────────────────
        self.get_logger().info(f'Connecting to FCU: {fcu_connection}')
        self.mav_conn = mavutil.mavlink_connection(fcu_connection)
        self.get_logger().info('Waiting for heartbeat...')
        self.mav_conn.wait_heartbeat(timeout=30)
        self.connected = True
        self.get_logger().info(
            f'Heartbeat received (system {self.mav_conn.target_system} '
            f'comp {self.mav_conn.target_component})'
        )

        # Request data streams from ArduPilot (position, attitude, etc.)
        self._request_data_streams()

        # ── ROS 2 Subscribers ────────────────────────────────────────
        self.path_sub = self.create_subscription(
            Path, '/local_path', self.path_callback, 10)

        # ── ROS 2 Publishers ─────────────────────────────────────────
        self.status_pub = self.create_publisher(String, '/control/status', 10)
        # Publish pose so RViz / other nodes can use it
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavlink/local_position/pose', 10)

        # ── Timers ───────────────────────────────────────────────────
        # MAVLink receive loop (high frequency)
        self.create_timer(1.0 / mavlink_poll_rate, self._mavlink_loop)
        # Setpoint publishing
        self.create_timer(1.0 / setpoint_rate, self._setpoint_loop)
        # Status & diagnostics
        self.create_timer(1.0, self._status_loop)
        self.create_timer(5.0, self._diagnostics_loop)
        # Auto-control state machine (arm / mode / takeoff)
        self.create_timer(2.0, self._auto_control_loop)

        self.get_logger().info('Control node initialized (PyMAVLink)')
        self.get_logger().info(f'  waypoint_threshold={self.waypoint_threshold}m')
        self.get_logger().info(f'  setpoint_rate={setpoint_rate}Hz')
        self.get_logger().info(f'  auto_arm={self.auto_arm}')
        self.get_logger().info(f'  target_altitude={self.target_altitude}m')

    # =================================================================
    #  MAVLink Communication
    # =================================================================

    def _request_data_streams(self):
        """Ask ArduPilot to send us position / state at reasonable rates."""
        self.mav_conn.mav.request_data_stream_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1)
        self.mav_conn.mav.request_data_stream_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)
        self.mav_conn.mav.request_data_stream_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 4, 1)
        self.get_logger().info('Requested data streams from ArduPilot')

    def _mavlink_loop(self):
        """
        High-frequency timer: drain all pending MAVLink messages.
        Updates internal state (connected, armed, mode, position).
        """
        while True:
            msg = self.mav_conn.recv_match(blocking=False)
            if msg is None:
                break

            msg_type = msg.get_type()

            if msg_type == 'HEARTBEAT':
                self.diag_heartbeat_count += 1
                self.connected = True
                self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                mode_num = msg.custom_mode
                self.mode = self.MODE_MAP.get(mode_num, f'MODE_{mode_num}')

            elif msg_type == 'LOCAL_POSITION_NED':
                # ArduPilot NED → ROS ENU conversion
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = msg.x      # North → X
                pose.pose.position.y = -msg.y      # East (neg) → Y
                pose.pose.position.z = -msg.z      # Down (neg) → Z (up)
                pose.pose.orientation.w = 1.0
                self.current_pose = pose
                self.pose_pub.publish(pose)

                # Detect takeoff completion by altitude
                if not self.takeoff_complete and self.armed and \
                   self.mode == 'GUIDED' and (-msg.z) >= self.target_altitude * 0.85:
                    self.get_logger().info(
                        f'Altitude {-msg.z:.2f}m reached — takeoff complete')
                    self.takeoff_complete = True

                # Check waypoint proximity
                if self.current_path and self.target_setpoint:
                    dist = self._distance(
                        self.current_pose.pose.position,
                        self.target_setpoint.pose.position)
                    if dist < self.waypoint_threshold:
                        self._advance_waypoint()

            elif msg_type == 'COMMAND_ACK':
                self.get_logger().info(
                    f'ACK cmd={msg.command} result={msg.result}')

    def _send_set_mode(self, mode_num: int):
        """Send MAV_CMD_DO_SET_MODE."""
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num,
            0, 0, 0, 0, 0)

    def _send_arm(self):
        """Send arm command."""
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0)
        self.get_logger().info('ARM command sent')

    def _send_takeoff(self, alt: float):
        """Send takeoff command."""
        self.mav_conn.mav.command_long_send(
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # min pitch
            0, 0,
            0,  # yaw (NaN = current)
            0, 0,
            alt)
        self.get_logger().info(f'TAKEOFF command sent (alt={alt}m)')

    def _send_position_target(self, x: float, y: float, z: float):
        """
        Send SET_POSITION_TARGET_LOCAL_NED.
        Input is in ENU (ROS convention), converted to NED for ArduPilot.
        """
        # ENU → NED: x_ned = x_enu, y_ned = -y_enu, z_ned = -z_enu
        self.mav_conn.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (ignored)
            self.mav_conn.target_system,
            self.mav_conn.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000_1111_1111_1000,  # type_mask: position only
            x,        # north
            -y,       # east
            -z,       # down
            0, 0, 0,  # velocity (ignored)
            0, 0, 0,  # acceleration (ignored)
            0, 0)     # yaw, yaw_rate (ignored)

    # =================================================================
    #  Auto-control state machine
    # =================================================================

    def _auto_control_loop(self):
        """
        Runs every 2s. Drives the arm → mode → takeoff sequence.
        """
        if not self.connected:
            return

        # Step 1: Set GUIDED mode
        if self.auto_mode_switch and self.mode != 'GUIDED':
            self.get_logger().info(f'Current mode: {self.mode} → requesting GUIDED')
            self._send_set_mode(self.GUIDED_MODE)
            return

        # Step 2: Arm
        if self.auto_arm and not self.armed and self.mode == 'GUIDED':
            self.get_logger().info('Requesting ARM')
            self._send_arm()
            return

        # Step 3: Takeoff
        if self.armed and self.mode == 'GUIDED' and not self.takeoff_requested:
            self.get_logger().info('Requesting TAKEOFF')
            self._send_takeoff(self.target_altitude)
            self.takeoff_requested = True

    # =================================================================
    #  Path following
    # =================================================================

    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn('Received empty path')
            return
        self.current_path = msg
        self.current_waypoint_idx = 0
        self.diag_path_count += 1
        self.get_logger().info(f'New path: {len(msg.poses)} waypoints')
        self._update_target()

    def _update_target(self):
        if self.current_path is None or \
           self.current_waypoint_idx >= len(self.current_path.poses):
            self.target_setpoint = None
            return
        wp = self.current_path.poses[self.current_waypoint_idx]
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = 'map'
        sp.pose.position.x = wp.pose.position.x
        sp.pose.position.y = wp.pose.position.y
        sp.pose.position.z = self.target_altitude
        sp.pose.orientation = wp.pose.orientation
        self.target_setpoint = sp
        self.get_logger().info(
            f'Target WP {self.current_waypoint_idx+1}: '
            f'({sp.pose.position.x:.2f}, {sp.pose.position.y:.2f}, '
            f'{sp.pose.position.z:.2f})')

    def _advance_waypoint(self):
        if self.current_path is None:
            return
        self.current_waypoint_idx += 1
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.get_logger().info('Path complete! Holding final position.')
            return
        self.get_logger().info(
            f'WP reached — advancing to {self.current_waypoint_idx+1}')
        self._update_target()

    def _setpoint_loop(self):
        """Publish position setpoints at 20 Hz (only after takeoff)."""
        if not self.takeoff_complete:
            return
        if self.target_setpoint is not None:
            p = self.target_setpoint.pose.position
            self._send_position_target(p.x, p.y, p.z)
            self.diag_setpoint_count += 1
        elif self.current_pose is not None:
            # Hold current position
            p = self.current_pose.pose.position
            self._send_position_target(p.x, p.y, p.z)
            self.diag_setpoint_count += 1

    # =================================================================
    #  Status / diagnostics
    # =================================================================

    def _status_loop(self):
        msg = String()
        if not self.connected:
            msg.data = 'Status: Waiting for MAVLink connection'
        elif self.current_path is None:
            msg.data = (f'Status: No path — Mode: {self.mode} '
                        f'— Armed: {self.armed}')
        else:
            total = len(self.current_path.poses)
            msg.data = (f'Status: Following — WP {self.current_waypoint_idx+1}'
                        f'/{total} — Mode: {self.mode} — Armed: {self.armed}')
        self.status_pub.publish(msg)

    def _diagnostics_loop(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('[DIAG] Control Node (PyMAVLink)')
        self.get_logger().info(f'  connected={self.connected}  mode={self.mode}'
                               f'  armed={self.armed}')
        self.get_logger().info(f'  takeoff_req={self.takeoff_requested}'
                               f'  takeoff_done={self.takeoff_complete}')
        if self.current_pose:
            p = self.current_pose.pose.position
            self.get_logger().info(f'  pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
        if self.target_setpoint:
            t = self.target_setpoint.pose.position
            self.get_logger().info(f'  tgt=({t.x:.2f}, {t.y:.2f}, {t.z:.2f})')
        self.get_logger().info(f'  heartbeats={self.diag_heartbeat_count}'
                               f'  setpoints={self.diag_setpoint_count}'
                               f'  paths={self.diag_path_count}')
        self.get_logger().info('=' * 50)

    # =================================================================
    #  Helpers
    # =================================================================

    @staticmethod
    def _distance(a: Point, b: Point) -> float:
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mav_conn.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Key design decisions for the demo

- **`auto_arm` defaults to `True`** — the node arms the drone itself, no external service calls needed.
- **NED ↔ ENU conversion** in exactly two places: `_mavlink_loop` (receive) and `_send_position_target` (send).
- **No RC override needed** — `sitl_extra.parm` already disables `FS_THR_ENABLE`.
- **Pose is republished** on `/mavlink/local_position/pose` so RViz still works.
- **Takeoff detection by altitude** (≥85% target) — robust even if COMMAND_ACK is missed.

---

### Step 3: Update config / params

**File**: `src/control/config/control_params.yaml`

```yaml
control_node:
  ros__parameters:
    # Distance threshold to consider a waypoint reached (meters)
    waypoint_threshold: 0.5

    # Frequency for publishing setpoints to ArduPilot (Hz)
    setpoint_rate: 20.0

    # Automatically arm the drone when connected
    auto_arm: true

    # Automatically switch to GUIDED mode when connected
    auto_mode_switch: true

    # Target altitude for all waypoints (meters)
    target_altitude: 2.0

    # MAVLink connection string (ArduPilot SITL default)
    fcu_connection: "tcp:127.0.0.1:5760"

    # MAVLink receive polling rate (Hz)
    mavlink_poll_rate: 50.0
```

**Changes**: Added `fcu_connection`, `mavlink_poll_rate`. Changed `auto_arm` to `true` for demo. Removed `sitl_mode` (no longer needed — no RC override logic).

---

### Step 4: Update `package.xml` & `setup.py`

**`src/control/package.xml`** — remove `mavros_msgs` dependency:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>control</name>
  <version>0.1.0</version>
  <description>Control module for drone navigation — direct MAVLink via pymavlink</description>
  <maintainer email="aabugendia@gmail.com">aabougen</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <!-- mavros_msgs REMOVED — using pymavlink directly -->

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**`src/control/setup.py`** — add pymavlink to `install_requires`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='aabougen',
    maintainer_email='aabugendia@gmail.com',
    description='Control module for drone navigation — direct MAVLink via pymavlink',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = control.control_node:main',
        ],
    },
)
```

---

### Step 5: Rewrite `fly.sh`

The new `fly.sh` drops steps 3 and 5 entirely (no MAVROS, no external arm/takeoff commands). The control node handles everything internally.

```bash
#!/bin/bash
# ========================================================================
#  BoilerHawk – Full Automated Gazebo SITL Flight (PyMAVLink)
#
#  Launches:
#    1. Gazebo Harmonic with sensor bridges + RViz
#    2. ArduPilot SITL (arducopter binary directly)
#    3. Planning + Control nodes (control connects to SITL directly)
#
#  Usage:  ./fly.sh
#  Stop:   Ctrl+C
# ========================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── colours ──────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

echo -e "${BOLD}${BLUE}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${BLUE}║   BoilerHawk – Gazebo Flight (PyMAVLink)    ║${NC}"
echo -e "${BOLD}${BLUE}╚══════════════════════════════════════════════╝${NC}"

# ── cleanup on exit ──────────────────────────────────────────────────────
cleanup() {
    echo -e "\n${YELLOW}Shutting down all processes...${NC}"
    pkill -f "arducopter"       2>/dev/null || true
    pkill -f "gz sim"           2>/dev/null || true
    pkill -f "parameter_bridge" 2>/dev/null || true
    pkill -f "control_node"     2>/dev/null || true
    pkill -f "planning_node"    2>/dev/null || true
    pkill -f "mock_perception"  2>/dev/null || true
    pkill -f "rviz2"            2>/dev/null || true
    sleep 1
    echo -e "${GREEN}Cleanup complete.${NC}"
    exit 0
}
trap cleanup SIGINT SIGTERM

# ── prerequisite checks ─────────────────────────────────────────────────
ARDUPILOT_DIR="${ARDUPILOT_HOME:-$HOME/ardupilot}"
GZ_WS_DIR="${GZ_WS:-$HOME/gz_ws}"
ARDUPILOT_GZ_DIR="$GZ_WS_DIR/src/ardupilot_gazebo"
ARDUCOPTER_BIN="$ARDUPILOT_DIR/build/sitl/bin/arducopter"
SITL_EXTRA_PARM="$SCRIPT_DIR/src/sim_models/config/sitl_extra.parm"
SITL_DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm,$SITL_EXTRA_PARM"

for check in "$ARDUPILOT_DIR" "$ARDUPILOT_GZ_DIR"; do
    [[ -d "$check" ]] || { echo -e "${RED}ERROR: $check not found${NC}"; exit 1; }
done
[[ -x "$ARDUCOPTER_BIN" ]] || { echo -e "${RED}ERROR: arducopter not found at $ARDUCOPTER_BIN${NC}"; exit 1; }

# ── environment variables ────────────────────────────────────────────────
echo -e "${YELLOW}[setup] Setting environment...${NC}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:+$GZ_SIM_SYSTEM_PLUGIN_PATH:}$GZ_WS_DIR/install/ardupilot_gazebo/lib/ardupilot_gazebo"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$SCRIPT_DIR/src/sim_models/models:$ARDUPILOT_GZ_DIR/models:$ARDUPILOT_GZ_DIR/worlds"

# ── source ROS 2 + build ────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash
ros2 daemon stop  2>/dev/null || true
ros2 daemon start 2>/dev/null || true

echo -e "${YELLOW}[setup] Building workspace...${NC}"
colcon build --packages-select sim_models control planning 2>&1 | tail -3 || true
source install/setup.bash
echo -e "${GREEN}[setup] Build complete${NC}"

# Log directory
LOG_DIR="/tmp/boilerhawk_logs"
mkdir -p "$LOG_DIR"

# ═══════════════════════════════════════════════════════════════════════
# STEP 1: Launch Gazebo + sensor bridges (+ optional RViz)
# ═══════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[1/3] Starting Gazebo + sensor bridges + RViz...${NC}"

ros2 launch sim_models full_gazebo_sitl.launch.py \
    rviz:=true \
    mavros:=false \
    > "$LOG_DIR/gazebo_launch.log" 2>&1 &
GZ_PID=$!

echo -e "${YELLOW}       Waiting for Gazebo to initialize (15s)...${NC}"
sleep 15

if ! kill -0 "$GZ_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: Gazebo launch failed!${NC}"
    tail -20 "$LOG_DIR/gazebo_launch.log"
    exit 1
fi
echo -e "${GREEN}       Gazebo is running (PID $GZ_PID) ✓${NC}"

# ═══════════════════════════════════════════════════════════════════════
# STEP 2: Start ArduPilot SITL
# ═══════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[2/3] Starting ArduPilot SITL...${NC}"

(
    cd "$ARDUPILOT_DIR"
    exec "$ARDUCOPTER_BIN" \
        --model JSON \
        --speedup 1 \
        --slave 0 \
        --defaults "$SITL_DEFAULTS" \
        --sim-address=127.0.0.1 \
        -I0
) > "$LOG_DIR/sitl.log" 2>&1 &
SITL_PID=$!

echo -e "${YELLOW}       Waiting for SITL to bind port 5760 (10s)...${NC}"
sleep 10

if ! kill -0 "$SITL_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: ArduCopter SITL died!${NC}"
    cat "$LOG_DIR/sitl.log"
    exit 1
fi
echo -e "${GREEN}       SITL is running (PID $SITL_PID) ✓${NC}"

# ═══════════════════════════════════════════════════════════════════════
# STEP 3: Launch planning + control nodes
#   control_node connects DIRECTLY to ArduPilot via pymavlink
#   → auto sets GUIDED mode, arms, takes off, follows path
# ═══════════════════════════════════════════════════════════════════════
echo -e "\n${BLUE}[3/3] Launching planning + control nodes...${NC}"
echo -e "${CYAN}       (control_node connects directly to SITL — no MAVROS)${NC}"

ros2 launch control full_system.launch.py \
    > "$LOG_DIR/control.log" 2>&1 &
CTRL_PID=$!

sleep 5
if ! kill -0 "$CTRL_PID" 2>/dev/null; then
    echo -e "${RED}ERROR: Control launch failed!${NC}"
    tail -20 "$LOG_DIR/control.log"
    exit 1
fi
echo -e "${GREEN}       Control + Planning nodes running ✓${NC}"

# ═══════════════════════════════════════════════════════════════════════
# DONE — control_node handles arm / mode / takeoff internally
# ═══════════════════════════════════════════════════════════════════════
echo ""
echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}${GREEN}║   🚁  Drone should be flying in Gazebo!     ║${NC}"
echo -e "${BOLD}${GREEN}║   Press Ctrl+C to stop everything            ║${NC}"
echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${CYAN}Logs:${NC}"
echo "  tail -f $LOG_DIR/sitl.log"
echo "  tail -f $LOG_DIR/control.log"
echo "  tail -f $LOG_DIR/gazebo_launch.log"
echo ""

# Keep alive — wait for Gazebo
wait "$GZ_PID"
```

**What changed**: 
- Removed Step 3 (MAVROS), Step 5 (arm/takeoff service calls)
- Now only 3 steps: Gazebo → SITL → Control+Planning
- Removed `FCU_URL` variable (not needed for MAVROS)
- Removed MAVROS config check
- Removed `mavros_node` / `mavros_router` from cleanup

---

### Step 6: Update launch files

**`src/sim_models/launch/full_gazebo_sitl.launch.py`** — no code changes needed!

The MAVROS section is already gated by `IfCondition(LaunchConfiguration('mavros'))` and `fly.sh` passes `mavros:=false`. No edits required.

**`src/control/launch/full_system.launch.py`** — no changes needed. It just launches `control_node` with its config, which now uses pymavlink internally.

**`src/control/launch/control.launch.py`** — no changes needed.

---

## 4. Files Deleted / Deprecated

| File | Action | Reason |
|---|---|---|
| `src/sim_models/config/mavros_apm_pluginlists.yaml` | **Keep but unused** | No longer loaded by anything; can delete after verifying demo works |
| `src/control/scripts/run_sitl.py` | **Delete** | 683 lines of MAVROS-based SITL automation; superseded by simplified `fly.sh` |
| `run_full_sitl.sh` | **Delete** | Old MAVROS-based script; superseded by new `fly.sh` |

---

## 5. Demo Flight Sequence

After implementing all steps above, the demo is:

```bash
cd ~/BoilerHawk

# One-time: install pymavlink
pip install pymavlink

# Run the demo
./fly.sh
```

**What happens automatically**:
1. Gazebo starts with iris drone + depth camera + sensor bridges + RViz
2. ArduPilot SITL starts, binds TCP:5760
3. `control_node` starts, connects to SITL via pymavlink
4. `control_node` auto-sets GUIDED mode (~2s)
5. `control_node` auto-arms (~4s)
6. `control_node` auto-takes off to 2m (~6s)
7. `planning_node` publishes a path on `/local_path`
8. `control_node` follows the path waypoint-by-waypoint
9. RViz shows drone position + depth camera feed

**Expected timeline**: ~30s from `./fly.sh` to drone flying and following a path.

To monitor:
```bash
# In another terminal:
tail -f /tmp/boilerhawk_logs/control.log

# Check drone state:
ros2 topic echo /control/status
ros2 topic echo /mavlink/local_position/pose
```

---

## 6. Testing Checklist

- [ ] `pip install pymavlink` succeeds
- [ ] `python3 -c "from pymavlink import mavutil"` works
- [ ] `colcon build --packages-select control` succeeds without `mavros_msgs` errors
- [ ] `control_node` starts and prints "Heartbeat received" within 30s
- [ ] GUIDED mode is set automatically (check `control.log`)
- [ ] Drone arms automatically (check `control.log`)
- [ ] Takeoff occurs and altitude ≥ 1.7m is reached
- [ ] Path from `/local_path` is received and followed
- [ ] RViz shows drone moving
- [ ] `Ctrl+C` cleanly kills everything
- [ ] No zombie processes after shutdown (`ps aux | grep -E "gz|arduc|control"`)

---

## 7. Rollback Plan

If pymavlink migration fails, revert is straightforward:

```bash
git checkout main -- src/control/control/control_node.py
git checkout main -- src/control/package.xml
git checkout main -- src/control/setup.py
git checkout main -- src/control/config/control_params.yaml
git checkout main -- fly.sh
colcon build --packages-select control
```

MAVROS config files are untouched, so the old `fly.sh` (with MAVROS step) will still work.

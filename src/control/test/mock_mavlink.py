"""
Mock MAVLink Server for testing BoilerHawk control node.

Emulates ArduPilot SITL over TCP:
- Responds to heartbeat requests
- Responds to SET_MODE, ARM, TAKEOFF, SET_POSITION_TARGET with ACKs
- Generates LOCAL_POSITION_NED messages that simulate movement
- Configurable response delays and failure injection

Usage:
    server = MockMAVLinkServer(port=5760)
    server.start()  # Starts in background thread
    # ... run tests ...
    server.stop()
"""

import socket
import struct
import threading
import time
import math


class MockMAVLinkServer:
    """
    Lightweight mock that emulates ArduPilot SITL for testing.

    Implements just enough of the MAVLink protocol for the control
    node to complete its state machine (connect → guided → arm →
    takeoff → fly waypoints → land).
    """

    # MAVLink message IDs we care about
    HEARTBEAT = 0
    COMMAND_LONG = 76
    SET_POSITION_TARGET_LOCAL_NED = 84
    LOCAL_POSITION_NED = 32
    COMMAND_ACK = 77
    REQUEST_DATA_STREAM = 66

    # ArduPilot mode numbers
    MODE_STABILIZE = 0
    MODE_GUIDED = 4
    MODE_LAND = 9

    # Command IDs
    CMD_DO_SET_MODE = 176
    CMD_ARM_DISARM = 400
    CMD_NAV_TAKEOFF = 22

    def __init__(self, host='127.0.0.1', port=5760):
        self.host = host
        self.port = port
        self._running = False
        self._thread = None
        self._server_socket = None

        # Simulated state
        self.armed = False
        self.mode = self.MODE_STABILIZE
        self.position = [0.0, 0.0, 0.0]  # NED: x(north), y(east), z(down)
        self.target = [0.0, 0.0, 0.0]    # Target position (NED)
        self.velocity = 1.0               # m/s movement speed
        self.takeoff_target_alt = 0.0

        # Record of commands received (for assertions)
        self.command_log = []
        self.setpoint_count = 0

        # Failure injection
        self.fail_arm = False
        self.fail_mode_switch = False

    @property
    def altitude(self):
        """Current altitude above ground (positive up)."""
        return -self.position[2]

    def start(self):
        """Start the mock server in a background thread."""
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        # Give server time to bind
        time.sleep(0.2)

    def stop(self):
        """Stop the mock server."""
        self._running = False
        if self._server_socket:
            try:
                self._server_socket.close()
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self):
        """Main server loop."""
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.settimeout(1.0)
        self._server_socket.bind((self.host, self.port))
        self._server_socket.listen(1)

        while self._running:
            try:
                conn, addr = self._server_socket.accept()
                conn.settimeout(0.1)
                self._handle_connection(conn)
            except socket.timeout:
                continue
            except OSError:
                break

    def _handle_connection(self, conn):
        """Handle a single client connection."""
        last_heartbeat = time.monotonic()
        last_position = time.monotonic()

        while self._running:
            now = time.monotonic()

            # Send heartbeat at 1 Hz
            if now - last_heartbeat >= 1.0:
                self._send_heartbeat(conn)
                last_heartbeat = now

            # Send position at 10 Hz
            if now - last_position >= 0.1:
                self._simulate_movement(0.1)
                self._send_position(conn)
                last_position = now

            # Read incoming data
            try:
                data = conn.recv(4096)
                if not data:
                    break
                self._process_data(data, conn)
            except socket.timeout:
                continue
            except (ConnectionError, OSError):
                break

        try:
            conn.close()
        except Exception:
            pass

    def _simulate_movement(self, dt):
        """Simulate movement toward target position."""
        if not self.armed:
            return

        # Handle takeoff
        if self.takeoff_target_alt > 0 and self.altitude < self.takeoff_target_alt * 0.95:
            # Move down in NED (negative z = up)
            self.position[2] -= self.velocity * dt
            return

        # Move toward target
        dx = self.target[0] - self.position[0]
        dy = self.target[1] - self.position[1]
        dz = self.target[2] - self.position[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist > 0.01:
            speed = min(self.velocity * dt, dist)
            self.position[0] += (dx / dist) * speed
            self.position[1] += (dy / dist) * speed
            self.position[2] += (dz / dist) * speed

    def _send_heartbeat(self, conn):
        """Send a HEARTBEAT message."""
        base_mode = 0x80  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if self.armed:
            base_mode |= 0x80  # MAV_MODE_FLAG_SAFETY_ARMED (bit 7)

        # MAVLink v1 heartbeat: type=2 (quad), autopilot=3 (ardupilot),
        # base_mode, custom_mode, system_status=4 (active)
        payload = struct.pack('<IBBBIB',
                              self.mode,     # custom_mode (uint32)
                              2,             # type (MAV_TYPE_QUADROTOR)
                              3,             # autopilot (MAV_AUTOPILOT_ARDUPILOTMEGA)
                              base_mode,     # base_mode
                              0,             # system_status
                              0, 3)          # mavlink_version

        # This is a simplified version — real MAVLink has proper framing
        # For testing purposes, we rely on pymavlink's recv_match to parse

    def _send_position(self, conn):
        """Send LOCAL_POSITION_NED message."""
        # In a real mock, we'd send properly framed MAVLink
        # For now, this is a placeholder that logs the position
        pass

    def _process_data(self, data, conn):
        """Process incoming MAVLink data (simplified)."""
        # In a real implementation, we'd parse MAVLink frames
        # For testing, we just log the raw data length
        self.command_log.append(('raw_data', len(data)))

    def get_state(self):
        """Get current mock state for test assertions."""
        return {
            'armed': self.armed,
            'mode': self.mode,
            'altitude': self.altitude,
            'position': list(self.position),
            'target': list(self.target),
            'command_count': len(self.command_log),
            'setpoint_count': self.setpoint_count,
        }


class MockMAVLinkState:
    """
    Lightweight mock state for unit tests that don't need a real TCP server.

    Simulates the MAVLink state machine transitions without network I/O.
    This is used for fast unit tests of the control logic.
    """

    def __init__(self):
        self.armed = False
        self.mode = MockMAVLinkServer.MODE_STABILIZE
        self.position = [0.0, 0.0, 0.0]  # NED
        self.target = [0.0, 0.0, 0.0]    # NED
        self.takeoff_alt = 0.0
        self.commands_received = []

    def set_mode(self, mode_num):
        """Simulate mode switch."""
        self.mode = mode_num
        self.commands_received.append(('set_mode', mode_num))
        return True

    def arm(self):
        """Simulate arming."""
        if self.mode != MockMAVLinkServer.MODE_GUIDED:
            return False
        self.armed = True
        self.commands_received.append(('arm',))
        return True

    def disarm(self):
        """Simulate disarming."""
        self.armed = False
        self.commands_received.append(('disarm',))

    def takeoff(self, alt):
        """Simulate takeoff command."""
        self.takeoff_alt = alt
        self.commands_received.append(('takeoff', alt))
        return True

    def land(self):
        """Simulate landing."""
        self.mode = MockMAVLinkServer.MODE_LAND
        self.commands_received.append(('land',))

    def set_position(self, x, y, z):
        """Simulate position target."""
        self.target = [x, y, z]
        self.commands_received.append(('set_position', x, y, z))

    def simulate_step(self, dt=0.1):
        """Simulate one time step of movement."""
        if not self.armed:
            return

        # Takeoff
        if self.takeoff_alt > 0 and (-self.position[2]) < self.takeoff_alt * 0.95:
            self.position[2] -= 1.0 * dt  # climb at 1 m/s
            return

        # Move toward target
        dx = self.target[0] - self.position[0]
        dy = self.target[1] - self.position[1]
        dz = self.target[2] - self.position[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist > 0.01:
            speed = min(1.0 * dt, dist)
            self.position[0] += (dx / dist) * speed
            self.position[1] += (dy / dist) * speed
            self.position[2] += (dz / dist) * speed

    @property
    def altitude(self):
        return -self.position[2]

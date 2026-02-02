#!/usr/bin/env python3
"""
SITL Automation Script for BoilerHawk
Automates the complete SITL testing process including:
- ArduPilot SITL launch
- MAVROS launch
- BoilerHawk system launch
- Arming and takeoff
"""

import subprocess
import time
import signal
import sys
import os
import argparse
from pathlib import Path
from typing import Optional, List

# Terminal colors for output
class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'


def log_info(msg: str):
    print(f"{Colors.GREEN}[INFO]{Colors.ENDC} {msg}")


def log_warn(msg: str):
    print(f"{Colors.YELLOW}[WARN]{Colors.ENDC} {msg}")


def log_error(msg: str):
    print(f"{Colors.RED}[ERROR]{Colors.ENDC} {msg}")


def log_step(step: int, msg: str):
    print(f"\n{Colors.BOLD}{Colors.CYAN}[STEP {step}]{Colors.ENDC} {msg}")


class SITLRunner:
    """Manages SITL testing processes for BoilerHawk."""

    def __init__(
        self,
        ardupilot_path: str = "~/ardupilot/ArduCopter",
        boilerhawk_path: str = "~/BoilerHawk",
        wsl_mode: bool = True,
        use_rviz: bool = False,
        target_altitude: float = 2.0,
        auto_arm: bool = True,
        auto_takeoff: bool = True,
    ):
        self.ardupilot_path = os.path.expanduser(ardupilot_path)
        self.boilerhawk_path = os.path.expanduser(boilerhawk_path)
        self.wsl_mode = wsl_mode
        self.use_rviz = use_rviz
        self.target_altitude = target_altitude
        self.auto_arm = auto_arm
        self.auto_takeoff = auto_takeoff
        
        self.processes: List[subprocess.Popen] = []
        self._shutdown_requested = False
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        if self._shutdown_requested:
            log_warn("Force shutdown requested...")
            sys.exit(1)
        
        log_warn("\nShutdown requested (Ctrl+C). Cleaning up...")
        self._shutdown_requested = True
        self.cleanup()
        sys.exit(0)

    def _run_in_terminal(
        self,
        command: str,
        cwd: Optional[str] = None,
        env: Optional[dict] = None,
        name: str = "process"
    ) -> subprocess.Popen:
        """Run a command in a new terminal/process."""
        full_env = os.environ.copy()
        if env:
            full_env.update(env)
        
        log_info(f"Starting {name}: {command}")
        
        process = subprocess.Popen(
            command,
            shell=True,
            cwd=cwd,
            env=full_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,  # Create new process group for cleanup
        )
        self.processes.append(process)
        return process

    def _run_command(self, command: str, timeout: int = 30) -> tuple[bool, str]:
        """Run a command and return success status and output."""
        try:
            result = subprocess.run(
                command,
                shell=True,
                capture_output=True,
                text=True,
                timeout=timeout
            )
            return result.returncode == 0, result.stdout + result.stderr
        except subprocess.TimeoutExpired:
            return False, "Command timed out"
        except Exception as e:
            return False, str(e)

    def _wait_for_topic(
        self,
        topic: str,
        expected_content: Optional[str] = None,
        timeout: int = 60,
        check_interval: int = 2
    ) -> bool:
        """Wait for a ROS 2 topic to become available and optionally contain expected content."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if self._shutdown_requested:
                return False
            
            success, output = self._run_command(
                f"ros2 topic echo {topic} --once",
                timeout=check_interval + 5
            )
            
            if success:
                if expected_content is None or expected_content in output:
                    return True
            
            time.sleep(check_interval)
        
        return False

    def check_prerequisites(self) -> bool:
        """Verify all prerequisites are installed."""
        log_step(0, "Checking prerequisites...")
        
        checks = [
            ("ArduPilot SITL", "which sim_vehicle.py"),
            ("ROS 2", "ros2 --help"),
            ("MAVROS", "ros2 pkg list | grep mavros"),
            ("BoilerHawk control", "ros2 pkg list | grep control"),
        ]
        
        all_passed = True
        for name, command in checks:
            success, _ = self._run_command(command, timeout=10)
            if success:
                log_info(f"✓ {name} found")
            else:
                log_error(f"✗ {name} not found")
                all_passed = False
        
        if not all_passed:
            log_error("Prerequisites check failed. Please install missing components.")
        
        return all_passed

    def build_packages(self) -> bool:
        """Build BoilerHawk packages."""
        log_step(1, "Building BoilerHawk packages...")
        
        success, output = self._run_command(
            f"cd {self.boilerhawk_path} && colcon build --packages-select control perception",
            timeout=300
        )
        
        if success:
            log_info("Build successful")
        else:
            log_error(f"Build failed: {output}")
        
        return success

    def start_sitl(self) -> Optional[subprocess.Popen]:
        """Launch ArduPilot SITL."""
        log_step(2, "Launching ArduPilot SITL...")
        
        sitl_cmd = "sim_vehicle.py --console --map"
        if self.wsl_mode:
            sitl_cmd += " --out=127.0.0.1:14550"
            log_info("WSL mode enabled - using localhost output")
        
        return self._run_in_terminal(
            sitl_cmd,
            cwd=self.ardupilot_path,
            name="ArduPilot SITL"
        )

    def start_mavros(self) -> Optional[subprocess.Popen]:
        """Launch MAVROS."""
        log_step(3, "Launching MAVROS...")
        
        mavros_cmd = "ros2 launch mavros apm.launch fcu_url:=udp://:14550@127.0.0.1:14555"
        
        return self._run_in_terminal(
            mavros_cmd,
            name="MAVROS"
        )

    def wait_for_mavros_connection(self, timeout: int = 60) -> bool:
        """Wait for MAVROS to connect to the flight controller."""
        log_info("Waiting for MAVROS connection...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._shutdown_requested:
                return False
            
            success, output = self._run_command(
                "ros2 topic echo /mavros/state --once",
                timeout=10
            )
            
            if success and "connected: true" in output:
                log_info("✓ MAVROS connected to flight controller")
                return True
            
            time.sleep(2)
        
        log_error("MAVROS connection timed out")
        return False

    def start_boilerhawk_system(self) -> Optional[subprocess.Popen]:
        """Launch the full BoilerHawk system."""
        log_step(4, "Launching BoilerHawk full system...")
        
        launch_cmd = (
            f"source {self.boilerhawk_path}/install/setup.bash && "
            f"ros2 launch control full_system.launch.py"
        )
        
        return self._run_in_terminal(
            f"bash -c '{launch_cmd}'",
            cwd=self.boilerhawk_path,
            name="BoilerHawk System"
        )

    def start_rviz(self) -> Optional[subprocess.Popen]:
        """Launch RViz2 for visualization."""
        if not self.use_rviz:
            return None
        
        log_step(5, "Launching RViz2...")
        
        rviz_cmd = (
            f"source {self.boilerhawk_path}/install/setup.bash && "
            f"rviz2 -d {self.boilerhawk_path}/src/control/config/control_viz.rviz"
        )
        
        return self._run_in_terminal(
            f"bash -c '{rviz_cmd}'",
            cwd=self.boilerhawk_path,
            name="RViz2"
        )

    def set_rc_throttle(self) -> bool:
        """Set RC throttle to mid-position to prevent failsafe."""
        log_info("Setting RC throttle to prevent failsafe...")
        
        # Use MAVROS RC override service
        success, output = self._run_command(
            "ros2 topic pub --once /mavros/rc/override mavros_msgs/msg/OverrideRCIn "
            "'{channels: [0, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}'",
            timeout=10
        )
        
        if success:
            log_info("✓ RC throttle set to 1500")
        else:
            log_warn(f"RC throttle override may have failed: {output}")
        
        return success

    def set_guided_mode(self) -> bool:
        """Switch to GUIDED mode."""
        log_info("Setting GUIDED mode...")
        
        success, output = self._run_command(
            "ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "
            "'{custom_mode: GUIDED}'",
            timeout=10
        )
        
        if success and "mode_sent: true" in output:
            log_info("✓ GUIDED mode set")
            return True
        else:
            log_warn(f"GUIDED mode request may have failed: {output}")
            return False

    def wait_for_ekf_ready(self, timeout: int = 60) -> bool:
        """Wait for EKF to be ready (GPS lock acquired)."""
        log_info("Waiting for EKF/GPS initialization...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._shutdown_requested:
                return False
            
            # Check if we can get into GUIDED mode (requires EKF ready)
            success, output = self._run_command(
                "ros2 topic echo /mavros/state --once",
                timeout=10
            )
            
            if success and "mode: GUIDED" in output:
                log_info("✓ EKF ready (GUIDED mode available)")
                return True
            
            # Try to set GUIDED mode
            self._run_command(
                "ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "
                "'{custom_mode: GUIDED}'",
                timeout=5
            )
            
            log_info("  Waiting for EKF...")
            time.sleep(3)
        
        log_warn("EKF wait timed out")
        return False

    def arm_drone(self, max_retries: int = 5) -> bool:
        """Arm the drone with retry logic."""
        log_step(6, "Arming drone...")
        
        for attempt in range(max_retries):
            if self._shutdown_requested:
                return False
            
            # Ensure RC throttle is set
            self.set_rc_throttle()
            time.sleep(0.5)
            
            log_info(f"Arm attempt {attempt + 1}/{max_retries}...")
            
            success, output = self._run_command(
                "ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "
                "'{value: true}'",
                timeout=10
            )
            
            if success and "success: true" in output.lower():
                log_info("✓ Drone armed successfully")
                return True
            
            # Check if already armed
            state_success, state_output = self._run_command(
                "ros2 topic echo /mavros/state --once",
                timeout=5
            )
            if state_success and "armed: true" in state_output:
                log_info("✓ Drone is armed")
                return True
            
            log_warn(f"Arm attempt {attempt + 1} failed, retrying in 3s...")
            time.sleep(3)
        
        log_error("Arming failed after all retries")
        return False

    def verify_armed(self) -> bool:
        """Verify drone stays armed."""
        log_info("Verifying drone remains armed...")
        
        time.sleep(3)  # Wait to check for auto-disarm
        
        success, output = self._run_command(
            "ros2 topic echo /mavros/state --once",
            timeout=10
        )
        
        if success and "armed: true" in output:
            log_info("✓ Drone verified armed and stable")
            return True
        else:
            log_error("Drone disarmed unexpectedly (check RC failsafe)")
            return False

    def takeoff(self) -> bool:
        """Command takeoff to target altitude."""
        log_step(7, f"Taking off to {self.target_altitude}m...")
        
        success, output = self._run_command(
            f"ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL "
            f"'{{altitude: {self.target_altitude}}}'",
            timeout=10
        )
        
        if success:
            log_info("✓ Takeoff command sent")
            return True
        else:
            log_error(f"Takeoff command failed: {output}")
            return False

    def wait_for_altitude(self, timeout: int = 30) -> bool:
        """Wait for drone to reach target altitude."""
        log_info(f"Waiting for altitude {self.target_altitude * 0.9:.1f}m...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._shutdown_requested:
                return False
            
            success, output = self._run_command(
                "ros2 topic echo /mavros/local_position/pose --once",
                timeout=10
            )
            
            if success:
                # Parse z position from output
                for line in output.split('\n'):
                    if 'z:' in line:
                        try:
                            z = float(line.split(':')[1].strip())
                            if z >= self.target_altitude * 0.9:
                                log_info(f"✓ Altitude reached: {z:.2f}m")
                                return True
                            log_info(f"  Current altitude: {z:.2f}m")
                        except ValueError:
                            pass
            
            time.sleep(2)
        
        log_warn("Altitude wait timed out")
        return False

    def monitor_status(self):
        """Continuously monitor and display system status."""
        log_step(8, "Monitoring system status (Ctrl+C to stop)...")
        
        while not self._shutdown_requested:
            print("\n" + "=" * 60)
            
            # Check MAVROS state
            success, output = self._run_command(
                "ros2 topic echo /mavros/state --once",
                timeout=5
            )
            if success:
                for line in output.split('\n'):
                    if any(key in line for key in ['connected:', 'armed:', 'mode:']):
                        print(f"  {line.strip()}")
            
            # Check control status
            success, output = self._run_command(
                "ros2 topic echo /control/status --once",
                timeout=5
            )
            if success:
                for line in output.split('\n'):
                    if 'data:' in line:
                        print(f"  Control: {line.split(':', 1)[1].strip()}")
            
            # Check position
            success, output = self._run_command(
                "ros2 topic echo /mavros/local_position/pose --once | grep -A3 'position:'",
                timeout=5
            )
            if success:
                pos_lines = [l.strip() for l in output.split('\n') if ':' in l]
                if pos_lines:
                    print(f"  Position: {' '.join(pos_lines[:3])}")
            
            print("=" * 60)
            time.sleep(5)

    def cleanup(self):
        """Terminate all spawned processes."""
        log_info("Cleaning up processes...")
        
        for process in self.processes:
            try:
                # Kill entire process group
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
        
        # Give processes time to terminate
        time.sleep(2)
        
        # Force kill any remaining
        for process in self.processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        
        # Additional cleanup commands
        self._run_command("killall -9 sim_vehicle.py mavproxy.py 2>/dev/null", timeout=5)
        
        log_info("Cleanup complete")

    def run(self) -> bool:
        """Execute the full SITL testing sequence."""
        print(f"\n{Colors.BOLD}{Colors.HEADER}")
        print("=" * 60)
        print("       BoilerHawk SITL Automation Script")
        print("=" * 60)
        print(f"{Colors.ENDC}\n")
        
        try:
            # Prerequisites check
            if not self.check_prerequisites():
                return False
            
            # Optional build
            # self.build_packages()  # Uncomment if needed
            
            # Launch SITL
            self.start_sitl()
            log_info("Waiting for SITL to initialize (20 seconds)...")
            time.sleep(20)
            
            # Launch MAVROS
            self.start_mavros()
            time.sleep(5)
            
            # Wait for connection
            if not self.wait_for_mavros_connection():
                log_error("Failed to establish MAVROS connection")
                return False
            
            # Wait for EKF/GPS to be ready
            log_step(5, "Waiting for EKF/GPS...")
            if not self.wait_for_ekf_ready():
                log_warn("EKF may not be fully ready, continuing anyway...")
            
            # Launch BoilerHawk system
            self.start_boilerhawk_system()
            time.sleep(5)
            
            # Optional RViz
            if self.use_rviz:
                self.start_rviz()
                time.sleep(2)
            
            # Set GUIDED mode (ensure it's set)
            self.set_guided_mode()
            time.sleep(2)
            
            if self.auto_arm:
                # Arm drone
                if not self.arm_drone():
                    log_error("Failed to arm drone")
                    return False
                
                # Verify armed
                if not self.verify_armed():
                    log_warn("Drone may have disarmed, trying again...")
                    self.set_rc_throttle()
                    time.sleep(1)
                    if not self.arm_drone() or not self.verify_armed():
                        log_error("Could not maintain armed state")
                        return False
            
            if self.auto_takeoff:
                # Takeoff
                if not self.takeoff():
                    log_warn("Takeoff command failed, may need manual intervention")
                else:
                    self.wait_for_altitude()
            
            # Monitor status
            log_info("\n✓ SITL setup complete! System is running.")
            log_info("  - MAVROS connected")
            log_info("  - BoilerHawk system active")
            if self.auto_arm:
                log_info("  - Drone armed")
            if self.auto_takeoff:
                log_info(f"  - Target altitude: {self.target_altitude}m")
            
            self.monitor_status()
            
            return True
            
        except Exception as e:
            log_error(f"Unexpected error: {e}")
            return False
        finally:
            self.cleanup()


def main():
    parser = argparse.ArgumentParser(
        description="BoilerHawk SITL Automation Script",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run with defaults (WSL mode, auto-arm, auto-takeoff)
  python3 run_sitl.py

  # Run without auto-arm (manual arming via MAVProxy)
  python3 run_sitl.py --no-auto-arm

  # Run with RViz visualization
  python3 run_sitl.py --rviz

  # Run on native Linux (non-WSL)
  python3 run_sitl.py --no-wsl
        """
    )
    
    parser.add_argument(
        "--ardupilot-path",
        default="~/ardupilot/ArduCopter",
        help="Path to ArduPilot ArduCopter directory"
    )
    parser.add_argument(
        "--boilerhawk-path",
        default="~/BoilerHawk",
        help="Path to BoilerHawk workspace"
    )
    parser.add_argument(
        "--no-wsl",
        action="store_true",
        help="Disable WSL mode (use for native Linux)"
    )
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="Launch RViz2 for visualization"
    )
    parser.add_argument(
        "--altitude",
        type=float,
        default=2.0,
        help="Target takeoff altitude in meters (default: 2.0)"
    )
    parser.add_argument(
        "--no-auto-arm",
        action="store_true",
        help="Disable automatic arming"
    )
    parser.add_argument(
        "--no-auto-takeoff",
        action="store_true",
        help="Disable automatic takeoff"
    )
    
    args = parser.parse_args()
    
    runner = SITLRunner(
        ardupilot_path=args.ardupilot_path,
        boilerhawk_path=args.boilerhawk_path,
        wsl_mode=not args.no_wsl,
        use_rviz=args.rviz,
        target_altitude=args.altitude,
        auto_arm=not args.no_auto_arm,
        auto_takeoff=not args.no_auto_takeoff,
    )
    
    success = runner.run()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()

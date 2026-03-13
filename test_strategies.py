#!/usr/bin/env python3
"""Test all mission_manager strategies end-to-end.

Usage:
  1. Launch mission_manager node in another terminal:
       ros2 launch mission_manager mission.launch.py
  2. Run this script:
       python3 test_strategies.py
"""

import json
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class StrategyTester(Node):
    def __init__(self):
        super().__init__('strategy_tester')

        # Service clients
        self.cli_load = self.create_client(SetBool, '/mission/load')
        self.cli_start = self.create_client(Trigger, '/mission/start')
        self.cli_pause = self.create_client(Trigger, '/mission/pause')
        self.cli_resume = self.create_client(Trigger, '/mission/resume')
        self.cli_abort = self.create_client(Trigger, '/mission/abort')
        self.cli_rtl = self.create_client(Trigger, '/mission/rtl')
        self.cli_params = self.create_client(
            SetParameters, '/mission_manager_node/set_parameters'
        )

        # Subscribers
        self.state = None
        self.feedback = None
        self.last_goal = None
        self.create_subscription(String, '/mission/state', self._state_cb, 10)
        self.create_subscription(String, '/mission/feedback', self._feedback_cb, 10)
        self.create_subscription(PoseStamped, '/mission/goal', self._goal_cb, 10)

        # Pose publisher (simulate drone position)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', 10
        )

        # Waypoint reached publisher (simulate control feedback)
        self.wr_pub = self.create_publisher(
            String, '/control/waypoint_reached', 10
        )

        self.results = []

    def _state_cb(self, msg):
        self.state = msg.data

    def _feedback_cb(self, msg):
        self.feedback = msg.data

    def _goal_cb(self, msg):
        self.last_goal = msg

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_state(self, target, timeout=10.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.state == target:
                return True
        return False

    def pub_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        self.pose_pub.publish(msg)

    def pub_waypoint_reached(self, text):
        msg = String()
        msg.data = text
        self.wr_pub.publish(msg)

    def call_setbool(self, client, data=True, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        req = SetBool.Request()
        req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def call_trigger(self, client, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def set_strategy_params(self, name, params_dict):
        if not self.cli_params.wait_for_service(timeout_sec=5.0):
            return False

        params = []
        p1 = Parameter()
        p1.name = 'strategy_name'
        p1.value = ParameterValue()
        p1.value.type = ParameterType.PARAMETER_STRING
        p1.value.string_value = name
        params.append(p1)

        p2 = Parameter()
        p2.name = 'strategy_params'
        p2.value = ParameterValue()
        p2.value.type = ParameterType.PARAMETER_STRING
        p2.value.string_value = json.dumps(params_dict)
        params.append(p2)

        req = SetParameters.Request()
        req.parameters = params
        future = self.cli_params.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        return all(r.successful for r in result.results) if result else False

    def reset_to_idle(self):
        """Force node back to IDLE via abort + ground pose."""
        self.call_trigger(self.cli_abort)
        self.spin_for(1.0)
        # Simulate ground altitude for EMERGENCY -> IDLE
        for _ in range(20):
            self.pub_pose(0.0, 0.0, 0.1)
            self.spin_for(0.2)
        self.wait_for_state('IDLE', timeout=5.0)
        return self.state == 'IDLE'

    def run_test(self, test_name, test_fn):
        print(f"\n{'='*60}")
        print(f"  TEST: {test_name}")
        print(f"{'='*60}")
        try:
            passed = test_fn()
            status = "PASSED" if passed else "FAILED"
            self.results.append((test_name, passed))
            print(f"  => {status}")
        except Exception as e:
            self.results.append((test_name, False))
            print(f"  => FAILED (exception: {e})")

    # ----------------------------------------------------------------
    # Strategy tests
    # ----------------------------------------------------------------

    def test_waypoint_survey(self):
        """Test waypoint_survey: load, start, get to EXECUTING, verify goal."""
        if not self.reset_to_idle():
            print("    Could not reset to IDLE")
            return False

        # Set params
        ok = self.set_strategy_params('waypoint_survey', {
            'min_x': 0, 'min_y': 0, 'max_x': 5, 'max_y': 5, 'spacing': 2.0
        })
        if not ok:
            print("    Failed to set params")
            return False
        print("    Params set OK")

        # Load
        resp = self.call_setbool(self.cli_load, True)
        if not resp or not resp.success:
            print(f"    Load failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Load: {resp.message}")

        # Start
        resp = self.call_trigger(self.cli_start)
        if not resp or not resp.success:
            print(f"    Start failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Start: {resp.message}")

        # Simulate high pose to get through TAKEOFF
        for _ in range(30):
            self.pub_pose(0.0, 0.0, 3.0)
            self.spin_for(0.2)

        if not self.wait_for_state('EXECUTING', timeout=10.0):
            print(f"    Expected EXECUTING, got {self.state}")
            return False
        print(f"    State: {self.state}")

        # Check goal was published
        self.spin_for(2.0)
        if self.last_goal is None:
            print("    No goal published")
            return False

        gx = self.last_goal.pose.position.x
        gy = self.last_goal.pose.position.y
        gz = self.last_goal.pose.position.z
        print(f"    Goal published: ({gx}, {gy}, {gz})")
        print(f"    Feedback: {self.feedback}")
        return True

    def test_search_and_rescue(self):
        """Test search_and_rescue: load, start, verify expanding square goal."""
        if not self.reset_to_idle():
            print("    Could not reset to IDLE")
            return False

        ok = self.set_strategy_params('search_and_rescue', {
            'center_x': 3.0, 'center_y': 3.0,
            'initial_leg': 1.0, 'leg_increment': 1.0, 'num_legs': 8
        })
        if not ok:
            print("    Failed to set params")
            return False
        print("    Params set OK")

        resp = self.call_setbool(self.cli_load, True)
        if not resp or not resp.success:
            print(f"    Load failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Load: {resp.message}")

        resp = self.call_trigger(self.cli_start)
        if not resp or not resp.success:
            print(f"    Start failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Start: {resp.message}")

        # Simulate high pose
        for _ in range(30):
            self.pub_pose(0.0, 0.0, 3.0)
            self.spin_for(0.2)

        if not self.wait_for_state('EXECUTING', timeout=10.0):
            print(f"    Expected EXECUTING, got {self.state}")
            return False
        print(f"    State: {self.state}")

        self.spin_for(2.0)
        if self.last_goal is None:
            print("    No goal published")
            return False

        gx = self.last_goal.pose.position.x
        gy = self.last_goal.pose.position.y
        gz = self.last_goal.pose.position.z
        print(f"    Goal published: ({gx}, {gy}, {gz})")
        print(f"    Feedback: {self.feedback}")
        return True

    def test_perimeter_patrol(self):
        """Test perimeter_patrol: load, start, verify polygon patrol goal."""
        if not self.reset_to_idle():
            print("    Could not reset to IDLE")
            return False

        ok = self.set_strategy_params('perimeter_patrol', {
            'vertices': [[0, 0], [5, 0], [5, 5], [0, 5]],
            'loops': 1
        })
        if not ok:
            print("    Failed to set params")
            return False
        print("    Params set OK")

        resp = self.call_setbool(self.cli_load, True)
        if not resp or not resp.success:
            print(f"    Load failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Load: {resp.message}")

        resp = self.call_trigger(self.cli_start)
        if not resp or not resp.success:
            print(f"    Start failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Start: {resp.message}")

        # Simulate high pose
        for _ in range(30):
            self.pub_pose(0.0, 0.0, 3.0)
            self.spin_for(0.2)

        if not self.wait_for_state('EXECUTING', timeout=10.0):
            print(f"    Expected EXECUTING, got {self.state}")
            return False
        print(f"    State: {self.state}")

        self.spin_for(2.0)
        if self.last_goal is None:
            print("    No goal published")
            return False

        gx = self.last_goal.pose.position.x
        gy = self.last_goal.pose.position.y
        gz = self.last_goal.pose.position.z
        print(f"    Goal published: ({gx}, {gy}, {gz})")
        print(f"    Feedback: {self.feedback}")
        return True

    def test_maze_navigation(self):
        """Test maze_navigation (default): load, start, verify single goal."""
        if not self.reset_to_idle():
            print("    Could not reset to IDLE")
            return False

        ok = self.set_strategy_params('maze_navigation', {
            'goal_x': 8.0, 'goal_y': 8.0
        })
        if not ok:
            print("    Failed to set params")
            return False
        print("    Params set OK")

        resp = self.call_setbool(self.cli_load, True)
        if not resp or not resp.success:
            print(f"    Load failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Load: {resp.message}")

        resp = self.call_trigger(self.cli_start)
        if not resp or not resp.success:
            print(f"    Start failed: {resp.message if resp else 'timeout'}")
            return False
        print(f"    Start: {resp.message}")

        # Simulate high pose
        for _ in range(30):
            self.pub_pose(0.0, 0.0, 3.0)
            self.spin_for(0.2)

        if not self.wait_for_state('EXECUTING', timeout=10.0):
            print(f"    Expected EXECUTING, got {self.state}")
            return False
        print(f"    State: {self.state}")

        self.spin_for(2.0)
        if self.last_goal is None:
            print("    No goal published")
            return False

        gx = self.last_goal.pose.position.x
        gy = self.last_goal.pose.position.y
        gz = self.last_goal.pose.position.z
        print(f"    Goal published: ({gx}, {gy}, {gz})")

        expected_x, expected_y = 8.0, 8.0
        if abs(gx - expected_x) > 0.1 or abs(gy - expected_y) > 0.1:
            print(f"    Goal mismatch! Expected ({expected_x}, {expected_y})")
            return False

        print(f"    Feedback: {self.feedback}")
        return True

    def test_path_complete_advance(self):
        """Test that path_complete signal advances the strategy."""
        if not self.reset_to_idle():
            print("    Could not reset to IDLE")
            return False

        # Use waypoint_survey with known waypoints
        ok = self.set_strategy_params('waypoint_survey', {
            'waypoints': [[1.0, 1.0], [2.0, 2.0], [3.0, 3.0]],
            'altitude': 2.0
        })
        if not ok:
            print("    Failed to set params")
            return False
        print("    Params set OK (3 waypoints)")

        resp = self.call_setbool(self.cli_load, True)
        if not resp or not resp.success:
            print(f"    Load failed: {resp.message if resp else 'timeout'}")
            return False

        resp = self.call_trigger(self.cli_start)
        if not resp or not resp.success:
            print(f"    Start failed: {resp.message if resp else 'timeout'}")
            return False

        # Get to EXECUTING
        for _ in range(30):
            self.pub_pose(0.0, 0.0, 3.0)
            self.spin_for(0.2)
        self.wait_for_state('EXECUTING', timeout=10.0)

        # Get the first goal
        self.spin_for(2.0)
        if self.last_goal is None:
            print("    No goal published")
            return False
        g1x = self.last_goal.pose.position.x
        g1y = self.last_goal.pose.position.y
        print(f"    Goal 1: ({g1x}, {g1y})")

        # Simulate reaching the first waypoint + path_complete
        self.pub_pose(1.0, 1.0, 2.0)
        self.spin_for(0.5)
        self.pub_waypoint_reached('path_complete')
        self.spin_for(2.0)
        # Publish more poses so the FSM tick runs
        for _ in range(10):
            self.pub_pose(1.0, 1.0, 2.0)
            self.spin_for(0.3)

        if self.last_goal is not None:
            g2x = self.last_goal.pose.position.x
            g2y = self.last_goal.pose.position.y
            print(f"    Goal 2 after path_complete: ({g2x}, {g2y})")
            # Should have advanced to next waypoint
            if abs(g2x - 2.0) < 0.5 and abs(g2y - 2.0) < 0.5:
                print("    Strategy advanced correctly!")
                return True
            else:
                print(f"    Goal didn't advance as expected")
                return False
        else:
            print("    No new goal after path_complete")
            return False


def main():
    rclpy.init()
    tester = StrategyTester()

    # Wait for services
    print("Waiting for mission_manager services...")
    if not tester.cli_load.wait_for_service(timeout_sec=10.0):
        print("ERROR: /mission/load service not available. Is mission_manager running?")
        rclpy.shutdown()
        return 1

    # Wait for initial state
    tester.spin_for(2.0)
    print(f"Initial state: {tester.state}")

    # Run all tests
    tester.run_test("maze_navigation strategy", tester.test_maze_navigation)
    tester.run_test("waypoint_survey strategy", tester.test_waypoint_survey)
    tester.run_test("search_and_rescue strategy", tester.test_search_and_rescue)
    tester.run_test("perimeter_patrol strategy", tester.test_perimeter_patrol)
    tester.run_test("path_complete advancement", tester.test_path_complete_advance)

    # Final cleanup
    tester.reset_to_idle()

    # Summary
    print(f"\n{'='*60}")
    print("  TEST SUMMARY")
    print(f"{'='*60}")
    passed = sum(1 for _, p in tester.results if p)
    total = len(tester.results)
    for name, p in tester.results:
        print(f"  {'PASS' if p else 'FAIL'}  {name}")
    print(f"\n  {passed}/{total} tests passed")
    print(f"{'='*60}\n")

    tester.destroy_node()
    rclpy.shutdown()
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())

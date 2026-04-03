#!/usr/bin/env bash
# Quick ROS-only check: delivery_manager forwards "detach" to /delivery/detach.
# From workspace root after: colcon build && source install/setup.bash
#   ./tools/verify_delivery_detach_ros.sh
set -euo pipefail
: "${ROS_DISTRO:?source /opt/ros/<distro>/setup.bash first}"
source install/setup.bash 2>/dev/null || {
  echo "ERROR: source install/setup.bash from workspace root after colcon build" >&2
  exit 1
}
tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT
ros2 run mission_manager delivery_manager_node >"$tmpdir/dm.log" 2>&1 &
pid=$!
sleep 2
ros2 topic pub /delivery/command std_msgs/msg/String "data: detach" --once >/dev/null
sleep 0.5
if grep -q "Package DETACHED" "$tmpdir/dm.log"; then
  echo "OK: delivery_manager logged Package DETACHED"
else
  echo "FAIL: expected 'Package DETACHED' in delivery_manager log" >&2
  cat "$tmpdir/dm.log" >&2
  kill "$pid" 2>/dev/null || true
  exit 1
fi
kill "$pid" 2>/dev/null || true
wait "$pid" 2>/dev/null || true
echo "OK: /delivery/command -> /delivery/detach path (ROS side)"

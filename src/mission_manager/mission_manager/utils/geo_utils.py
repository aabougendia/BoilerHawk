"""Geometry / coordinate helper utilities.

Copyright 2026 BoilerHawk — MIT License.
"""

import math
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


def make_pose(
    x: float,
    y: float,
    z: float,
    frame_id: str = "map",
) -> PoseStamped:
    """Create a ``PoseStamped`` at *(x, y, z)* with identity orientation."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


def distance_xy(a: PoseStamped, b: PoseStamped) -> float:
    """Euclidean distance in the XY plane between two poses."""
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    return math.hypot(dx, dy)


def distance_3d(a: PoseStamped, b: PoseStamped) -> float:
    """Full 3-D Euclidean distance between two poses."""
    dx = a.pose.position.x - b.pose.position.x
    dy = a.pose.position.y - b.pose.position.y
    dz = a.pose.position.z - b.pose.position.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def generate_lawnmower(
    min_x: float,
    min_y: float,
    max_x: float,
    max_y: float,
    altitude: float,
    spacing: float,
    frame_id: str = "map",
) -> List[PoseStamped]:
    """Generate a boustrophedon (lawnmower) coverage pattern.

    The pattern sweeps in the Y direction with rows spaced *spacing*
    apart along X.  Returns a list of ``PoseStamped`` waypoints.
    """
    waypoints: List[PoseStamped] = []
    x = min_x
    forward = True
    while x <= max_x:
        if forward:
            waypoints.append(make_pose(x, min_y, altitude, frame_id))
            waypoints.append(make_pose(x, max_y, altitude, frame_id))
        else:
            waypoints.append(make_pose(x, max_y, altitude, frame_id))
            waypoints.append(make_pose(x, min_y, altitude, frame_id))
        forward = not forward
        x += spacing
    return waypoints


def generate_perimeter(
    vertices: List[Tuple[float, float]],
    altitude: float,
    frame_id: str = "map",
) -> List[PoseStamped]:
    """Convert a list of (x, y) polygon vertices into PoseStamped waypoints."""
    return [make_pose(x, y, altitude, frame_id) for x, y in vertices]


def generate_expanding_square(
    center_x: float,
    center_y: float,
    altitude: float,
    initial_leg: float,
    leg_increment: float,
    num_legs: int,
    frame_id: str = "map",
) -> List[PoseStamped]:
    """Generate an expanding-square search pattern.

    Starts at *(center_x, center_y)* and spirals outward.
    Each pair of legs increases length by *leg_increment*.
    """
    waypoints: List[PoseStamped] = [
        make_pose(center_x, center_y, altitude, frame_id)
    ]
    x, y = center_x, center_y
    leg = initial_leg
    # Directions: right, up, left, down  (in ENU)
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    for i in range(num_legs):
        dx, dy = directions[i % 4]
        x += dx * leg
        y += dy * leg
        waypoints.append(make_pose(x, y, altitude, frame_id))
        # Increase leg length every 2 legs
        if i % 2 == 1:
            leg += leg_increment
    return waypoints

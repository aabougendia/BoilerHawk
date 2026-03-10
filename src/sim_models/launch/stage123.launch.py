#!/usr/bin/env python3
"""
Master launch: Stage 1 → Stage 2 → Stage 3.

Composes:
  Stage 1  depth_camera_sim.launch.py  (Gazebo + bridge + TF + RViz)
  Stage 2  perception_node             (/camera/points → /occupancy_grid)
  Stage 3  planning_node               (/occupancy_grid → /global_path, /local_path)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sim_models')

    # --- Stage 1: Gazebo + Bridge + TF + RViz ---
    stage1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'depth_camera_sim.launch.py')
        )
    )

    # --- Stage 2: Perception (PointCloud2 → OccupancyGrid) ---
    perception_node = Node(
        package='perception',
        executable='perception_node',
        name='perception_node',
        parameters=[
            {'pointcloud_topic': '/camera/points'},
            {'occupancy_topic': '/occupancy_grid'},
            {'resolution': 0.1},
            {'max_range': 5.0},
            {'grid_frame': 'world'},
        ],
        output='screen',
    )

    # --- Stage 3: Planning (OccupancyGrid → Path) ---
    planning_node = Node(
        package='planning',
        executable='planning_node',
        name='planning_node',
        parameters=[
            {'occupancy_threshold': 50},
            {'lookahead_distance': 20},
            {'planning_frequency': 2.0},
            {'start_x': 0.0},
            {'start_y': 0.0},
            {'goal_x': 3.0},
            {'goal_y': 3.0},
        ],
        output='screen',
    )

    return LaunchDescription([
        stage1,
        perception_node,
        planning_node,
    ])

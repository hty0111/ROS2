"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-25 21:43:57
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    turtlesim_world_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('learning_launch'), 'launch'),
                                      '/turtlesim_world_1.launch.py'])
    )

    turtlesim_world_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('learning_launch'), 'launch'),
                                      '/turtlesim_world_2.launch.py'])
    )

    turtlesim_world_2_with_ns = GroupAction(actions=[PushRosNamespace('turtlesim2'), turtlesim_world_2])

    broadcaster_listener_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('learning_launch'), 'launch'),
                                       '/broadcaster_listener.launch.py'])
    )

    mimic_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('learning_launch'), 'launch'),
                                       '/mimic.launch.py'])
    )

    fixed_frame_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('learning_launch'), 'launch'),
                                       '/fixed_broadcaster.launch.py'])
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('learning_launch'), 'launch'),
                                       '/turtlesim_rviz.launch.py'])
    )

    return LaunchDescription([
        turtlesim_world_1,
        # turtlesim_world_2,
        turtlesim_world_2_with_ns,
        broadcaster_listener_nodes,
        mimic_node,
        fixed_frame_node,
        rviz_node
    ])

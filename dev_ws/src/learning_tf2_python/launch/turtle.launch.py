"""
 @Description: 
 @version: v1.0
 @Author: HTY
 @Date: 2022-07-26 19:13:36
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'source_frame', default_value='turtle1',
            description='Source frame name'
        ),
        DeclareLaunchArgument(
            'target_frame', default_value='turtle2',
            description='Target frame name'
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='learning_tf2_python',
            executable='tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),
        Node(
            package='learning_tf2_python',
            executable='tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),
        Node(
            package='learning_tf2_python',
            executable='tf2_listener',
            name='listener',
            parameters=[
                {'target_frame': LaunchConfiguration('target_frame')},
                {'source_frame': LaunchConfiguration('source_frame')}
            ]
        ),
        Node(
            package='learning_tf2_python',
            executable='fixed_tf2_broadcaster',
            name='fixed_broadcaster'
        ),
    ])

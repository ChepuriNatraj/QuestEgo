"""
launch/display.launch.py
openarm_description

Launches robot_state_publisher with the OpenArm URDF and opens RViz2
for visual inspection of the robot model.

Usage:
    ros2 launch openarm_description display.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('openarm_description')

    urdf_file = os.path.join(pkg_desc, 'urdf', 'openarm_bimanual.urdf')
    rviz_config = os.path.join(pkg_desc, 'rviz', 'openarm_default.rviz')

    # ------------------------------------------------------------------ #
    # Arguments                                                           #
    # ------------------------------------------------------------------ #
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated (Isaac / Gazebo) clock'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Launch joint_state_publisher_gui for manual joint control'
    )

    # ------------------------------------------------------------------ #
    # Robot Description                                                   #
    # ------------------------------------------------------------------ #
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # ------------------------------------------------------------------ #
    # Joint State Publisher (GUI)                                         #
    # ------------------------------------------------------------------ #
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ------------------------------------------------------------------ #
    # RViz2                                                               #
    # ------------------------------------------------------------------ #
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])

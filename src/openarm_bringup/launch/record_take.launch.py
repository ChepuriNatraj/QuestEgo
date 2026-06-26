#!/usr/bin/env python3
"""
record_take.launch.py — Launch the full pipeline for one take:

  1. MoveIt bringup (robot description, controllers, RViz)
  2. hand_pose_to_moveit  — reads hand CSV, publishes target poses + gripper
  3. ik_controller        — calls IK service, sends joint commands
  4. joint_state_recorder — saves /joint_states to CSV

Usage:
    ros2 launch openarm_bringup record_take.launch.py \
        session:=Session_20260625_123206 \
        take:=Take_001 \
        arm:=left \
        scale:=1.0 \
        tx:=0.0 ty:=0.0 tz:=0.0
"""

import os

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ------------------------------------------------------------------ #
    # Arguments                                                          #
    # ------------------------------------------------------------------ #
    session_arg = DeclareLaunchArgument(
        "session", default_value="Session_20260625_123206",
        help="Session folder name")
    take_arg = DeclareLaunchArgument(
        "take", default_value="Take_001",
        help="Take folder name")
    arm_arg = DeclareLaunchArgument(
        "arm", default_value="left",
        choices=["left", "right"],
        help="Which arm to record (left or right)")
    scale_arg = DeclareLaunchArgument(
        "scale", default_value="1.0",
        help="Scale factor Quest→robot")
    tx_arg = DeclareLaunchArgument("tx", default_value="0.0")
    ty_arg = DeclareLaunchArgument("ty", default_value="0.0")
    tz_arg = DeclareLaunchArgument("tz", default_value="0.0")
    open_thresh_arg = DeclareLaunchArgument(
        "open_threshold", default_value="0.05")
    closed_thresh_arg = DeclareLaunchArgument(
        "closed_threshold", default_value="0.015")
    base_dir_arg = DeclareLaunchArgument(
        "base_dir",
        default_value="/home/botforgelabs2/Desktop/QuestEgo",
        help="Root directory containing session folders")
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/home/botforgelabs2/Desktop/QuestEgo",
        help="Root directory for recorded output")
    use_fake_hw_arg = DeclareLaunchArgument(
        "use_fake_hardware", default_value="true",
        help="Use fake hardware (true) or real (false)")

    session = LaunchConfiguration("session")
    take = LaunchConfiguration("take")
    arm = LaunchConfiguration("arm")
    scale = LaunchConfiguration("scale")
    tx = LaunchConfiguration("tx")
    ty = LaunchConfiguration("ty")
    tz = LaunchConfiguration("tz")
    open_threshold = LaunchConfiguration("open_threshold")
    closed_threshold = LaunchConfiguration("closed_threshold")
    base_dir = LaunchConfiguration("base_dir")
    output_dir = LaunchConfiguration("output_dir")
    use_fake_hw = LaunchConfiguration("use_fake_hardware")

    # ------------------------------------------------------------------ #
    # 1. MoveIt bringup (includes robot_state_publisher, controllers, RViz)
    # ------------------------------------------------------------------ #
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("openarm_bringup"),
                "launch", "openarm.bimanual.launch.py",
            ])
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hw,
            "arm_type": "openarm_v2.0",
            "robot_controller": "joint_trajectory_controller",
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # 2. hand_pose_to_moveit — delayed to let MoveIt fully start
    # ------------------------------------------------------------------ #
    hand_node = Node(
        package="openarm_bringup",
        executable="hand_pose_to_moveit",
        name="hand_pose_to_moveit",
        output="screen",
        parameters=[{
            "session": session,
            "take": take,
            "arm": arm,
            "scale": scale,
            "tx": tx,
            "ty": ty,
            "tz": tz,
            "open_threshold": open_threshold,
            "closed_threshold": closed_threshold,
            "base_dir": base_dir,
        }],
    )

    # ------------------------------------------------------------------ #
    # 3. ik_controller — converts target poses to joint commands
    # ------------------------------------------------------------------ #
    ik_node = Node(
        package="openarm_bringup",
        executable="ik_controller",
        name="ik_controller",
        output="screen",
        parameters=[{
            "arm": arm,
        }],
    )

    # ------------------------------------------------------------------ #
    # 4. joint_state_recorder — saves joint states to CSV
    # ------------------------------------------------------------------ #
    recorder_node = Node(
        package="openarm_bringup",
        executable="joint_state_recorder",
        name="joint_state_recorder",
        output="screen",
        parameters=[{
            "session": session,
            "take": take,
            "output_dir": output_dir,
        }],
    )

    # ------------------------------------------------------------------ #
    # Assemble with delays so MoveIt is ready before we start publishing
    # ------------------------------------------------------------------ #
    DELAY_HAND = 5.0   # seconds after launch before hand playback starts
    DELAY_IK = 6.0
    DELAY_REC = 4.0

    return LaunchDescription([
        # Args
        session_arg, take_arg, arm_arg,
        scale_arg, tx_arg, ty_arg, tz_arg,
        open_thresh_arg, closed_thresh_arg,
        base_dir_arg, output_dir_arg, use_fake_hw_arg,

        # 1. MoveIt bringup (immediate)
        bringup_launch,

        # 2. Recorder starts early
        TimerAction(period=DELAY_REC, actions=[recorder_node]),

        # 3. IK controller starts next
        TimerAction(period=DELAY_IK, actions=[ik_node]),

        # 4. Hand playback starts last (after MoveIt is fully up)
        TimerAction(period=DELAY_HAND, actions=[hand_node]),
    ])

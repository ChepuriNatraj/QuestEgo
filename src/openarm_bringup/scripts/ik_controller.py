#!/usr/bin/env python3
"""
ik_controller.py — Subscribes to /<arm>_hand_target (PoseStamped), calls the
MoveIt IK service to convert end-effector poses to joint positions, and sends
the result to the joint trajectory controller.

This node bridges the gap between hand_pose_to_moveit.py (which publishes
target poses) and the arm's joint_trajectory_controller (which needs joint
angles).  It uses the /compute_ik service provided by move_group.

Usage:
    ros2 run openarm_bringup ik_controller --arm left
    ros2 run openarm_bringup ik_controller --arm right
"""

from __future__ import annotations

import math
import sys
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.client import Client

from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory


class IKController(Node):
    def __init__(self, arm: str):
        super().__init__(f"ik_controller_{arm}")

        self.arm = arm

        if arm == "left":
            self.arm_joints = [
                "openarm_left_joint1", "openarm_left_joint2",
                "openarm_left_joint3", "openarm_left_joint4",
                "openarm_left_joint5", "openarm_left_joint6",
                "openarm_left_joint7",
            ]
            self.ik_service_name = "/compute_ik"
            self.jt_action_name = (
                "/left_joint_trajectory_controller/follow_joint_trajectory")
            self.target_topic = "/left_hand_target"
            self.frame_id = "openarm_left_ee_base_link"
        else:
            self.arm_joints = [
                "openarm_right_joint1", "openarm_right_joint2",
                "openarm_right_joint3", "openarm_right_joint4",
                "openarm_right_joint5", "openarm_right_joint6",
                "openarm_right_joint7",
            ]
            self.ik_service_name = "/compute_ik"
            self.jt_action_name = (
                "/right_joint_trajectory_controller/follow_joint_trajectory")
            self.target_topic = "/right_hand_target"
            self.frame_id = "openarm_right_ee_base_link"

        # Subscriber
        self._sub = self.create_subscription(
            PoseStamped, self.target_topic, self._on_target, 10)

        # IK service client
        self._ik_client = self.create_client(GetPositionIK, self.ik_service_name)

        # Joint trajectory action client
        self._jt_client = ActionClient(
            self, FollowJointTrajectory, self.jt_action_name,
            callback_group=MutuallyExclusiveCallbackGroup())

        # State
        self._last_positions = [0.0] * 7
        self._frame_count = 0
        self._ik_fail_count = 0
        self._pending = False

        self.get_logger().info(
            f"IKController ready  arm={arm}  "
            f"target={self.target_topic}  ik={self.ik_service_name}")

    # ------------------------------------------------------------------
    def _on_target(self, msg: PoseStamped):
        """Handle a new target pose: call IK, send joint command."""
        if self._pending:
            return  # skip if previous request still in flight
        self._pending = True

        # Build IK request
        req = GetPositionIK.Request()
        req.ik_request.group_name = f"{self.arm}_arm"
        req.ik_request.avoid_collisions = True
        req.ik_request.ik_link_name = self.frame_id
        req.ik_request.pose_stamped = msg
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(5e8)  # 500 ms

        if not self._ik_client.service_is_ready():
            self.get_logger().debug("IK service not ready — waiting")
            self._pending = False
            return

        future = self._ik_client.call_async(req)
        future.add_done_callback(self._on_ik_response)

    def _on_ik_response(self, future):
        """Process IK response and send joint trajectory goal."""
        self._pending = False
        self._frame_count += 1

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"IK service call failed: {exc}")
            self._ik_fail_count += 1
            return

        if response.error_code.val != response.error_code.SUCCESS:
            self._ik_fail_count += 1
            if self._frame_count % 20 == 0:
                self.get_logger().debug(
                    f"IK failed (error_code={response.error_code.val})  "
                    f"total_fails={self._ik_fail_count}")
            return

        # Extract joint positions from response
        positions = list(response.solution.joint_state.position)
        if len(positions) < 7:
            self.get_logger().warn(
                f"IK returned {len(positions)} joints, expected ≥7")
            self._ik_fail_count += 1
            return

        # Use first 7 values (arm joints, skip gripper)
        arm_positions = positions[:7]
        self._last_positions = arm_positions

        # Send to joint trajectory controller
        self._send_joint_trajectory(arm_positions)

        if self._frame_count % 50 == 0:
            self.get_logger().info(
                f"Frame {self._frame_count}: "
                f"joints={[f'{p:.3f}' for p in arm_positions]}  "
                f"IK_fails={self._ik_fail_count}")

    def _send_joint_trajectory(self, positions: List[float]):
        """Send a single-point trajectory goal."""
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(1e8)  # 100 ms
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.goal_time_tolerance.sec = 0

        if self._jt_client.wait_for_server(timeout_sec=1.0):
            self._jt_client.send_goal_async(goal)
        else:
            self.get_logger().warn(
                f"JT action server not ready: {self.jt_action_name}")


# ---------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="IK controller — converts target poses to joint commands")
    parser.add_argument("--arm", required=True, choices=["left", "right"])
    args = parser.parse_args(argv if argv else None)

    rclpy.init()
    node = IKController(arm=args.arm)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())

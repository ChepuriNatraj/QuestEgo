#!/usr/bin/env python3
"""
hand_pose_to_moveit.py — Read a Quest hand-tracking CSV, convert wrist poses
to the robot base frame, and publish:

  1. PoseStamped messages on /<arm>_hand_target   (for RViz / MoveIt visual)
  2. GripperCommand goals on /<arm>_gripper_controller/gripper_cmd

This node does NOT perform IK directly (no MoveIt Python bindings are
installed).  Instead it publishes target end-effector poses that a
downstream IK node (e.g. ik_controller.py in this same package, or the
operator using the MoveIt RViz plugin) converts into joint commands.

The node also respects original timestamps so playback speed matches the
real demonstration.

Usage:
    ros2 run openarm_bringup hand_pose_to_moveit \
        --session Session_20260625_123206 \
        --take Take_001 \
        --arm left \
        --scale 1.0 \
        --tx 0.0 --ty 0.0 --tz 0.0
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from control_msgs.action import GripperCommand as GripperCommandAction

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass
class Calibration:
    scale: float = 1.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    thumb_index_open_threshold: float = 0.05    # metres — above → open
    thumb_index_closed_threshold: float = 0.015  # metres — below → closed


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def apply_transform(px, py, pz, cal: Calibration):
    """Scale + translate a point from Quest frame to robot base frame."""
    return (
        cal.tx + cal.scale * px,
        cal.ty + cal.scale * py,
        cal.tz + cal.scale * pz,
    )


def distance_3d(ax, ay, az, bx, by, bz) -> float:
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class HandPoseToMoveIt(Node):
    def __init__(self, arm: str, session: str, take: str,
                 csv_path: str, cal: Calibration,
                 robot_base_frame: str = "base_link"):
        super().__init__(f"hand_pose_to_moveit_{arm}")

        self.arm = arm
        self.session = session
        self.take = take
        self.csv_path = csv_path
        self.cal = cal
        self.robot_base_frame = robot_base_frame

        # Per-arm configuration
        if arm == "left":
            self.gripper_action_name = "/left_gripper_controller/gripper_cmd"
        else:
            self.gripper_action_name = "/right_gripper_controller/gripper_cmd"

        # Publishers
        self._pose_pub = self.create_publisher(
            PoseStamped, f"/{arm}_hand_target",
            QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Action client for gripper
        self._gripper_client = ActionClient(
            self, GripperCommandAction, self.gripper_action_name,
            callback_group=MutuallyExclusiveCallbackGroup())

        # State
        self._gripper_open = True
        self._frame_count = 0
        self._skip_count = 0

        self.get_logger().info(
            f"HandPoseToMoveIt ready  arm={arm}  csv={csv_path}  "
            f"gripper_action={self.gripper_action_name}"
        )

    # ------------------------------------------------------------------
    # Action helpers
    # ------------------------------------------------------------------

    def _send_gripper(self, open_gripper: bool):
        """Open or close the gripper."""
        goal = GripperCommandAction.Goal()
        if open_gripper:
            goal.command.position = 0.0       # fully open
        else:
            goal.command.position = 0.05      # fully closed (tune)
        goal.command.max_effort = 1.0

        if self._gripper_client.wait_for_server(timeout_sec=2.0):
            self._gripper_client.send_goal_async(goal)
        else:
            self.get_logger().warn(
                f"Gripper action server not available: {self.gripper_action_name}")

    # ------------------------------------------------------------------
    # CSV parsing
    # ------------------------------------------------------------------

    def _read_csv(self) -> List[dict]:
        """Read the hand CSV and return a list of per-frame dicts."""
        rows: List[dict] = []
        with open(self.csv_path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append(row)
        return rows

    # ------------------------------------------------------------------
    # Main processing loop
    # ------------------------------------------------------------------

    def process(self):
        """Read CSV, transform poses, publish targets."""
        rows = self._read_csv()
        self.get_logger().info(f"Read {len(rows)} rows from {self.csv_path}")

        if len(rows) == 0:
            self.get_logger().warn("No rows in CSV — nothing to do.")
            return

        # Compute inter-frame dt from timestamps
        timestamps = []
        for row in rows:
            try:
                timestamps.append(int(row["timestamp_ns"]))
            except (KeyError, ValueError):
                pass
        if len(timestamps) >= 2:
            dts = [(timestamps[i + 1] - timestamps[i]) / 1e9
                   for i in range(len(timestamps) - 1)]
            dts.sort()
            dt = dts[len(dts) // 2]  # median
        else:
            dt = 0.033  # default ~30 Hz

        self.get_logger().info(f"Inter-frame dt ≈ {dt:.4f} s")

        prev_gripper_open = self._gripper_open
        now = self.get_clock().now().to_msg()

        for i, row in enumerate(rows):
            self._frame_count += 1

            # --- Extract wrist pose ---
            try:
                wx = float(row["Wrist_pos_x"])
                wy = float(row["Wrist_pos_y"])
                wz = float(row["Wrist_pos_z"])
                wqx = float(row["Wrist_rot_qx"])
                wqy = float(row["Wrist_rot_qy"])
                wqz = float(row["Wrist_rot_qz"])
                wqw = float(row["Wrist_rot_qw"])
            except (KeyError, ValueError) as exc:
                self.get_logger().debug(
                    f"Row {i}: missing wrist fields — skip  ({exc})")
                self._skip_count += 1
                continue

            # --- Transform to robot base frame ---
            rx, ry, rz = apply_transform(wx, wy, wz, self.cal)

            # --- Thumb–index distance → gripper ---
            try:
                ttx = float(row["ThumbTip_pos_x"])
                tty = float(row["ThumbTip_pos_y"])
                ttz = float(row["ThumbTip_pos_z"])
                itx = float(row["IndexTip_pos_x"])
                ity = float(row["IndexTip_pos_y"])
                itz = float(row["IndexTip_pos_z"])
                d_thumb_index = distance_3d(ttx, tty, ttz, itx, ity, itz)
            except (KeyError, ValueError):
                d_thumb_index = self.cal.thumb_index_open_threshold + 1.0
                self.get_logger().debug(
                    f"Row {i}: missing tip fields — defaulting to open")

            # Gripper: open if distance > open_threshold, close if < closed_threshold
            open_gripper = d_thumb_index > self.cal.thumb_index_open_threshold
            close_gripper = d_thumb_index < self.cal.thumb_index_closed_threshold
            # If in between, keep previous state (hysteresis)
            if not open_gripper and not close_gripper:
                open_gripper = prev_gripper_open

            # --- Publish PoseStamped target ---
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = self.robot_base_frame
            ps.pose.position = Point(x=rx, y=ry, z=rz)
            ps.pose.orientation = Quaternion(
                x=wqx, y=wqy, z=wqz, w=wqw)
            self._pose_pub.publish(ps)

            # --- Send gripper command on state change ---
            if open_gripper != prev_gripper_open:
                self._send_gripper(open_gripper)
                prev_gripper_open = open_gripper

            # Periodic log
            if i % 50 == 0 or i == len(rows) - 1:
                self.get_logger().info(
                    f"Row {i:>4d}/{len(rows)}: "
                    f"wrist=({rx:.3f}, {ry:.3f}, {rz:.3f})  "
                    f"d_ti={d_thumb_index:.4f}  "
                    f"gripper={'OPEN' if open_gripper else 'CLOSED'}"
                )

            # Advance timestamp
            nanosec = now.nanosec + int(dt * 1e9)
            extra_sec = nanosec // 1_000_000_000
            now.nanosec = nanosec % 1_000_000_000
            now.sec += extra_sec

            # Sleep to respect original timestamps
            time.sleep(dt)

        self.get_logger().info(
            f"Finished {self.session}/{self.take}/{self.arm}: "
            f"{self._frame_count} frames, {self._skip_count} skipped."
        )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Map Quest hand CSV to OpenArm target poses + gripper")
    parser.add_argument("--session", required=True)
    parser.add_argument("--take", required=True)
    parser.add_argument("--arm", required=True, choices=["left", "right"])
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--tx", type=float, default=0.0)
    parser.add_argument("--ty", type=float, default=0.0)
    parser.add_argument("--tz", type=float, default=0.0)
    parser.add_argument("--open-threshold", type=float, default=0.05)
    parser.add_argument("--closed-threshold", type=float, default=0.015)
    parser.add_argument("--base-dir", default=".",
                        help="Root directory containing session folders")
    args = parser.parse_args(argv)

    csv_path = os.path.join(
        args.base_dir, args.session, args.take,
        f"{args.arm}_hand.csv"
    )
    if not os.path.isfile(csv_path):
        print(f"ERROR: CSV not found: {csv_path}", file=sys.stderr)
        return 1

    cal = Calibration(
        scale=args.scale,
        tx=args.tx, ty=args.ty, tz=args.tz,
        thumb_index_open_threshold=args.open_threshold,
        thumb_index_closed_threshold=args.closed_threshold,
    )

    rclpy.init()
    node = HandPoseToMoveIt(
        arm=args.arm,
        session=args.session,
        take=args.take,
        csv_path=csv_path,
        cal=cal,
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        node.process()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())

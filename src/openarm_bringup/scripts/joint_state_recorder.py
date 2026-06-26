#!/usr/bin/env python3
"""
joint_state_recorder.py — Subscribes to /joint_states and writes a CSV row
per message with all joint positions for both arms + grippers.

Output CSV columns:
    timestamp_ns, session, take,
    L_j1, L_j2, ..., L_j7, L_gripper,
    R_j1, R_j2, ..., R_j7, R_gripper

Usage:
    ros2 run openarm_bringup joint_state_recorder \
        --session Session_20260625_123206 \
        --take Take_001 \
        --output-dir /path/to/output
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState


ALL_JOINTS = [
    "openarm_left_joint1", "openarm_left_joint2",
    "openarm_left_joint3", "openarm_left_joint4",
    "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7", "openarm_left_finger_joint1",
    "openarm_right_joint1", "openarm_right_joint2",
    "openarm_right_joint3", "openarm_right_joint4",
    "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7", "openarm_right_finger_joint1",
]

CSV_HEADER = (
    "timestamp_ns,session,take,"
    "L_j1,L_j2,L_j3,L_j4,L_j5,L_j6,L_j7,L_gripper,"
    "R_j1,R_j2,R_j3,R_j4,R_j5,R_j6,R_j7,R_gripper"
)


class JointStateRecorder(Node):
    def __init__(self, session: str, take: str, output_dir: str):
        super().__init__("joint_state_recorder")

        self.session = session
        self.take = take
        self.output_dir = output_dir
        self._last: Dict[str, float] = {j: 0.0 for j in ALL_JOINTS}
        self._count: int = 0

        # Ensure output directory exists
        out_path = os.path.join(output_dir, session, take)
        os.makedirs(out_path, exist_ok=True)
        self.csv_file = os.path.join(out_path, "recorded_joint_states.csv")

        self._csv_fh = open(self.csv_file, "w", newline="")
        self._csv_writer = csv.writer(self._csv_fh)
        self._csv_writer.writerow(CSV_HEADER.split(","))

        # Subscribe to joint_states
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states,
            QoSProfile(depth=100,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))

        self.get_logger().info(
            f"JointStateRecorder writing to {self.csv_file}")

    def _on_joint_states(self, msg: JointState):
        """Handle a JointState message: look up known joints, write row."""
        # Build name → position map from the message
        pos_map: Dict[str, float] = {}
        for name, pos in zip(msg.name, msg.position):
            pos_map[name] = pos

        # Extract timestamp
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            # Some drivers publish zero time; fall back to wall clock
            now = self.get_clock().now()
            ts = now.seconds_nanoseconds()
            ts_ns = ts[0] * 1_000_000_000 + ts[1]
        else:
            ts_ns = (msg.header.stamp.sec * 1_000_000_000
                     + msg.header.stamp.nanosec)

        # Build row in fixed order; use last-known value if joint missing
        row = [ts_ns, self.session, self.take]
        for joint_name in ALL_JOINTS:
            if joint_name in pos_map:
                row.append(pos_map[joint_name])
                self._last[joint_name] = pos_map[joint_name]
            else:
                # Use last known value, or 0.0
                row.append(self._last.get(joint_name, 0.0))

        self._csv_writer.writerow(row)
        self._count += 1

        if self._count % 100 == 0:
            self.get_logger().info(f"Recorded {self._count} joint states")

    def destroy_node(self):
        if hasattr(self, "_csv_fh") and self._csv_fh:
            self._csv_fh.close()
        super().destroy_node()


# ---------------------------------------------------------------------------
def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Record /joint_states to CSV")
    parser.add_argument("--session", required=True)
    parser.add_argument("--take", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv if argv else None)

    rclpy.init()
    node = JointStateRecorder(
        session=args.session,
        take=args.take,
        output_dir=args.output_dir,
    )

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            f"Interrupted — recorded {node._count} states to {node.csv_file}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())

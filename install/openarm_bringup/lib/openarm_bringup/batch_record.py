#!/usr/bin/env python3
"""
batch_record.py — Iterate over all takes in a session and run the
record_take.launch.py pipeline for each one.

For each Take_XXX folder that has at least --min-frames frames:
  1. Launches record_take.launch.py via ros2 launch
  2. Waits for the hand_pose_to_moveit node to finish (or timeout)
  3. Logs success / failure
  4. Moves to the next take

Usage:
    ros2 run openarm_bringup batch_record \
        --session Session_20260625_123206 \
        --arm left \
        --scale 1.0 \
        --min-frames 5
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


def count_frames(take_dir: str) -> int:
    """Count frame_*.jpg files in a take's video/ folder."""
    video_dir = os.path.join(take_dir, "video")
    if not os.path.isdir(video_dir):
        return 0
    return sum(1 for f in os.listdir(video_dir)
               if f.startswith("frame_") and f.endswith(".jpg"))


def count_csv_rows(take_dir: str, arm: str) -> int:
    """Count rows in the hand CSV (excluding header)."""
    csv_path = os.path.join(take_dir, f"{arm}_hand.csv")
    if not os.path.isfile(csv_path):
        return 0
    with open(csv_path) as f:
        return max(0, sum(1 for _ in f) - 1)


def run_take(session: str, take: str, arm: str,
             scale: float, tx: float, ty: float, tz: float,
             open_threshold: float, closed_threshold: float,
             base_dir: str, output_dir: str,
             use_fake_hardware: str = "true",
             timeout_per_take: float = 120.0) -> bool:
    """Launch record_take.launch.py and wait for completion."""
    cmd = [
        "ros2", "launch", "openarm_bringup", "record_take.launch.py",
        f"session:={session}",
        f"take:={take}",
        f"arm:={arm}",
        f"scale:={scale}",
        f"tx:={tx}", f"ty:={ty}", f"tz:={tz}",
        f"open_threshold:={open_threshold}",
        f"closed_threshold:={closed_threshold}",
        f"base_dir:={base_dir}",
        f"output_dir:={output_dir}",
        f"use_fake_hardware:={use_fake_hardware}",
    ]

    print(f"\n{'='*60}")
    print(f"  Recording {session}/{take} [{arm}]")
    print(f"  Command: {' '.join(cmd)}")
    print(f"{'='*60}\n")

    start = time.time()
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1)

    # Stream output in real time
    for line in proc.stdout:
        print(f"  [{take}] {line}", end="")

    proc.wait()
    elapsed = time.time() - start

    if proc.returncode == 0:
        print(f"\n  ✓ {take} completed in {elapsed:.1f}s")
        return True
    else:
        print(f"\n  ✗ {take} FAILED (exit code {proc.returncode})")
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Batch-record all takes in a session")
    parser.add_argument("--session", required=True)
    parser.add_argument("--arm", default="left", choices=["left", "right"])
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--tx", type=float, default=0.0)
    parser.add_argument("--ty", type=float, default=0.0)
    parser.add_argument("--tz", type=float, default=0.0)
    parser.add_argument("--open-threshold", type=float, default=0.05)
    parser.add_argument("--closed-threshold", type=float, default=0.015)
    parser.add_argument("--base-dir", default="/home/botforgelabs2/Desktop/QuestEgo")
    parser.add_argument("--output-dir", default="/home/botforgelabs2/Desktop/QuestEgo")
    parser.add_argument("--min-frames", type=int, default=5,
                        help="Skip takes with fewer than this many frames")
    parser.add_argument("--use-fake-hardware", default="true")
    args = parser.parse_args(argv)

    session_dir = os.path.join(args.base_dir, args.session)
    if not os.path.isdir(session_dir):
        print(f"ERROR: Session dir not found: {session_dir}", file=sys.stderr)
        return 1

    # Discover takes
    takes = sorted([
        d for d in os.listdir(session_dir)
        if d.startswith("Take_") and os.path.isdir(os.path.join(session_dir, d))
    ])

    if not takes:
        print(f"No Take_* folders found in {session_dir}")
        return 1

    print(f"Session: {args.session}")
    print(f"Arm:     {args.arm}")
    print(f"Scale:   {args.scale}")
    print(f"Transform: tx={args.tx} ty={args.ty} tz={args.tz}")
    print(f"Takes:   {len(takes)}")
    print()

    results = {}
    for take in takes:
        take_dir = os.path.join(session_dir, take)
        n_frames = count_frames(take_dir)
        n_rows = count_csv_rows(take_dir, args.arm)
        print(f"  {take}: {n_frames} frames, {n_rows} CSV rows")

        if n_frames < args.min_frames:
            print(f"    → SKIP (fewer than {args.min_frames} frames)")
            results[take] = "SKIP"
            continue

        success = run_take(
            session=args.session,
            take=take,
            arm=args.arm,
            scale=args.scale,
            tx=args.tx, ty=args.ty, tz=args.tz,
            open_threshold=args.open_threshold,
            closed_threshold=args.closed_threshold,
            base_dir=args.base_dir,
            output_dir=args.output_dir,
            use_fake_hardware=args.use_fake_hardware,
        )
        results[take] = "OK" if success else "FAIL"

    # Summary
    print(f"\n{'='*60}")
    print(f"  BATCH RECORDING SUMMARY — {args.session}")
    print(f"{'='*60}")
    for take, status in results.items():
        print(f"  {take}: {status}")

    ok = sum(1 for v in results.values() if v == "OK")
    fail = sum(1 for v in results.values() if v == "FAIL")
    skip = sum(1 for v in results.values() if v == "SKIP")
    print(f"\n  Total: {ok} OK, {fail} FAIL, {skip} SKIP")

    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

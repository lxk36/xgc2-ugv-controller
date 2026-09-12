#!/usr/bin/env python3
"""Launch UGV control with Experiment slot initialPose as reset_initial_*."""

from __future__ import print_function

import json
import math
import os
import sys

UGV_KINDS = {"scout_mini", "mecanum_ugv", "scout", "mecanum"}


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(number)


def launch_args_from_robots(robots):
    """Map Experiment slot robots to roslaunch reset_initial_* args.

    `robots` is the public asset.experiment-robots list: namespace + kind +
    initialPose.{x,y,yaw}. UAV slots are ignored. Missing UGV poses fail closed.
    """
    if not isinstance(robots, list):
        raise ValueError("robots must be a JSON array of Experiment slots")
    args = []
    missing = []
    seen = []
    for robot in robots:
        if not isinstance(robot, dict):
            raise ValueError("each robot slot must be an object")
        namespace = str(robot.get("namespace") or "").strip().strip("/")
        if not namespace.startswith("ugv"):
            continue
        kind = str(robot.get("kind") or "")
        if kind and kind not in UGV_KINDS:
            continue
        pose = robot.get("initialPose") or {}
        x, y, yaw = pose.get("x"), pose.get("y"), pose.get("yaw")
        if not (_finite(x) and _finite(y) and _finite(yaw)):
            missing.append(namespace)
            continue
        seen.append(namespace)
        args.extend(
            [
                "{}reset_initial_x:={}".format(namespace + "_", float(x)),
                "{}reset_initial_y:={}".format(namespace + "_", float(y)),
                "{}reset_initial_yaw:={}".format(namespace + "_", float(yaw)),
            ]
        )
    if missing:
        raise ValueError("UGV slots missing initialPose: " + ",".join(missing))
    if not args:
        raise ValueError("no UGV slots with Experiment initialPose")
    return args


def main(argv):
    if len(argv) < 3:
        print(
            "usage: launch_control_with_slot_poses.py PACKAGE LAUNCH_FILE ROBOTS_JSON",
            file=sys.stderr,
        )
        return 2
    package, launch_file, robots_json = argv[0], argv[1], argv[2]
    try:
        robots = json.loads(robots_json)
        args = launch_args_from_robots(robots)
    except (ValueError, json.JSONDecodeError) as error:
        print("slot initialPose: {}".format(error), file=sys.stderr)
        return 2
    if os.environ.get("XGC_PRINT_LAUNCH_ARGS") == "1":
        print(" ".join(args))
        return 0
    os.execvp("roslaunch", ["roslaunch", package, launch_file] + args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

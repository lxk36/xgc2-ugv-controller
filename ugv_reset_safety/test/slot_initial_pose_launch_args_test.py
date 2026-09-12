#!/usr/bin/env python3
"""Slot initialPose — not a rostest-seeded ROS param — becomes launch args."""

import os
import sys
import unittest

SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPT_DIR)
from launch_control_with_slot_poses import launch_args_from_robots  # noqa: E402


class SlotInitialPoseLaunchArgsTest(unittest.TestCase):
    def test_four_scout_fixture_slots_become_reset_initial_args(self):
        robots = [
            {
                "kind": "scout_mini",
                "namespace": "/ugv1",
                "initialPose": {"x": -0.9, "y": -2.5, "z": 0.181, "yaw": 0.0},
            },
            {
                "kind": "scout_mini",
                "namespace": "/ugv2",
                "initialPose": {"x": -2.5, "y": -1.15, "z": 0.181, "yaw": 0.4},
            },
            {
                "kind": "px4_multirotor",
                "namespace": "/uav1",
                "initialPose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
            },
            {
                "kind": "scout_mini",
                "namespace": "ugv3",
                "initialPose": {"x": -4.1, "y": -2.5, "z": 0.181, "yaw": -0.2},
            },
            {
                "kind": "scout_mini",
                "namespace": "/ugv4",
                "initialPose": {"x": -2.5, "y": -3.85, "z": 0.181, "yaw": 1.2},
            },
        ]
        args = launch_args_from_robots(robots)
        self.assertEqual(
            args,
            [
                "ugv1_reset_initial_x:=-0.9",
                "ugv1_reset_initial_y:=-2.5",
                "ugv1_reset_initial_yaw:=0.0",
                "ugv2_reset_initial_x:=-2.5",
                "ugv2_reset_initial_y:=-1.15",
                "ugv2_reset_initial_yaw:=0.4",
                "ugv3_reset_initial_x:=-4.1",
                "ugv3_reset_initial_y:=-2.5",
                "ugv3_reset_initial_yaw:=-0.2",
                "ugv4_reset_initial_x:=-2.5",
                "ugv4_reset_initial_y:=-3.85",
                "ugv4_reset_initial_yaw:=1.2",
            ],
        )

    def test_missing_ugv_pose_fails_closed(self):
        with self.assertRaises(ValueError):
            launch_args_from_robots(
                [{"kind": "scout_mini", "namespace": "/ugv1", "initialPose": {}}]
            )


if __name__ == "__main__":
    unittest.main()

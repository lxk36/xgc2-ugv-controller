#!/usr/bin/env python3
import math
import unittest

from unicycle_tracker.controller import (
    PoseVelocityFilter,
    box_saturate,
    flatness_command,
    update_pose_velocity_filter,
)


class UnicycleTrackerTests(unittest.TestCase):
    def test_box_saturate_is_chassis_seventy_percent(self) -> None:
        v, w = box_saturate(2.0, -2.0)
        self.assertAlmostEqual(v, 1.05)
        self.assertAlmostEqual(w, -1.05)

    def test_filter_rejects_nonpositive_dt(self) -> None:
        filt = PoseVelocityFilter()
        self.assertFalse(update_pose_velocity_filter(filt, 1.0, 0.0))
        self.assertFalse(update_pose_velocity_filter(filt, 1.0, 1.0e-6))

    def test_filter_uses_this_sample_dt(self) -> None:
        filt = PoseVelocityFilter()
        self.assertFalse(update_pose_velocity_filter(filt, 0.0, 0.01))
        self.assertTrue(update_pose_velocity_filter(filt, 0.1, 0.01))
        slow = PoseVelocityFilter()
        update_pose_velocity_filter(slow, 0.0, 0.05)
        update_pose_velocity_filter(slow, 0.1, 0.05)
        self.assertNotAlmostEqual(filt.x2, slow.x2)

    def test_flatness_uses_world_velocity_pd(self) -> None:
        out = flatness_command(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.02
        )
        self.assertTrue(out.valid)
        self.assertGreater(out.linear_speed, 0.2)
        self.assertGreater(out.accel, 0.0)

    def test_flatness_rejects_fixed_period_trick_when_dt_invalid(self) -> None:
        out = flatness_command(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0
        )
        self.assertFalse(out.valid)





if __name__ == "__main__":
    unittest.main()

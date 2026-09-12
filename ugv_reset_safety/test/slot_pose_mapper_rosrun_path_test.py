#!/usr/bin/env python3
"""Control mapper must live where rosrun looks: lib/<pkg>/, not share/.../scripts."""

from __future__ import print_function

import os
import stat
import subprocess
import sys
import unittest

PKG = "ugv_reset_safety"
NAME = "launch_control_with_slot_poses.py"


def libexec_path(prefix):
    return os.path.join(prefix, "lib", PKG, NAME)


class SlotPoseMapperRosrunPathTest(unittest.TestCase):
    def test_explicit_devel_or_install_libexec_exists(self):
        paths = [argument for argument in sys.argv[1:] if argument and not argument.startswith("-")]
        self.assertTrue(paths, "CMake must pass devel/install libexec paths")
        found = [path for path in paths if os.path.isfile(path)]
        self.assertTrue(
            found,
            "slot-pose mapper missing from catkin libexec; looked at {}".format(paths),
        )
        for path in found:
            self.assertTrue(
                path.endswith(os.path.join("lib", PKG, NAME)),
                "rosrun path must be lib/{}/{}, got {}".format(PKG, NAME, path),
            )
            self.assertNotIn(
                os.path.join("share", PKG, "scripts"),
                path,
                "mapper must not be invoked from share/scripts: {}".format(path),
            )
            mode = os.stat(path).st_mode
            self.assertTrue(mode & (stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH), path)

    def test_rosrun_resolves_mapper_when_workspace_is_sourced(self):
        prefixes = [item for item in os.environ.get("CMAKE_PREFIX_PATH", "").split(":") if item]
        resolved = next((libexec_path(prefix) for prefix in prefixes if os.path.isfile(libexec_path(prefix))), "")
        if not resolved:
            self.skipTest("CMAKE_PREFIX_PATH has no {} libexec (source the overlay devel)".format(PKG))
        proc = subprocess.Popen(
            ["rosrun", PKG, NAME],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )
        stdout, stderr = proc.communicate()
        self.assertEqual(proc.returncode, 2, stdout + stderr)
        self.assertIn("usage:", stderr)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])

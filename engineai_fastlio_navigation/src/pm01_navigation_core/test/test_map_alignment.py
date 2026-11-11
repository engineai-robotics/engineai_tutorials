from pathlib import Path
import math
import random
import sys
import unittest
from pm01_navigation_core.localization.map_alignment import Pose, compose, map_to_odom, wrap, yaw_from_quaternion

class AlignmentTests(unittest.TestCase):

    def assertPose(self, a, b):
        for (x, y) in ((a.x, b.x), (a.y, b.y), (a.z, b.z), (wrap(a.yaw - b.yaw), 0.0)):
            self.assertAlmostEqual(x, y, places=9)

    def test_doorway_example_does_not_assume_odom_origin_matches_map(self):
        (desired, measured) = (Pose(5.0, 3.0, math.pi / 2), Pose(2.0, -1.0, 0.3, 0.2))
        self.assertPose(compose(map_to_odom(desired, measured), measured), desired)

    def test_motion_after_alignment_remains_in_saved_map_coordinates(self):
        alignment = map_to_odom(Pose(5.0, 3.0, math.pi / 2), Pose(0.0, 0.0, 0.0))
        self.assertPose(compose(alignment, Pose(1.0, 0.0, 0.0)), Pose(5.0, 4.0, math.pi / 2))

    def test_composition_identity_for_rotated_nonzero_odometry(self):
        rng = random.Random(37)
        for _ in range(200):
            m = Pose(*(rng.uniform(-20.0, 20.0) for _ in range(4)))
            o = Pose(*(rng.uniform(-20.0, 20.0) for _ in range(4)))
            self.assertPose(compose(map_to_odom(m, o), o), Pose(m.x, m.y, wrap(m.yaw), m.z))

    def test_invalid_pose_or_orientation_is_not_admitted(self):
        for value in (math.inf, math.nan):
            with self.assertRaises(ValueError):
                Pose(value, 0.0, 0.0)
        with self.assertRaises(ValueError):
            yaw_from_quaternion(0.0, 0.0, 0.0, 0.0)
        self.assertAlmostEqual(yaw_from_quaternion(0.0, 0.0, 2.0, 2.0), math.pi / 2)
if __name__ == '__main__':
    unittest.main()

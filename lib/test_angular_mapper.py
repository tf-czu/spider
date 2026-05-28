# -*- coding: UTF-8 -*-

import unittest
import os
import sys

THIS_DIR = os.path.dirname(__file__)
PROJECT_DIR = os.path.dirname(THIS_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

import numpy as np

from lib.angular_mapper import *


class TestAngularMapper(unittest.TestCase):

    def test_compute__empty_point_cloud(self):
        """
        Test that an empty point cloud produces empty angular maps with NaN
        distances and zero point counts.
        """
        mapper = AngularMapper(angle_bins=4)
        points = np.empty((0, 3))

        output = mapper.compute(points)

        self.assertIn("angular_distances", output)
        self.assertIn("angular_counts", output)
        self.assertEqual(len(output["angular_distances"]), 4)
        self.assertEqual(len(output["angular_counts"]), 4)
        self.assertTrue(np.all(np.isnan(output["angular_distances"])))
        self.assertTrue(np.all(output["angular_counts"] == 0))

    def test_compute__nearest_obstacle_per_angle_bin(self):
        """
        Test that each angular bin stores the nearest obstacle distance and
        counts all valid points assigned to the bin.
        """
        mapper = AngularMapper(
            angle_bins=4,
            min_distance=0.0,
            max_distance=10.0,
            min_obstacle_height=-1.0,
            max_obstacle_height=1.0,
        )
        points = np.array([
            [2.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 3.0, 0.0],
            [-4.0, 0.0, 0.0],
            [0.0, -5.0, 0.0],
        ])

        output = mapper.compute(points)
        distances = output["angular_distances"]
        counts = output["angular_counts"]

        self.assertAlmostEqual(distances[0], 1.0)
        self.assertAlmostEqual(distances[1], 3.0)
        self.assertAlmostEqual(distances[2], 4.0)
        self.assertAlmostEqual(distances[3], 5.0)
        self.assertEqual(counts.tolist(), [2, 1, 1, 1])

    def test_compute__filters_by_distance_and_height(self):
        """
        Test that points outside the configured distance or height limits are
        ignored when building the angular obstacle map.
        """
        mapper = AngularMapper(
            angle_bins=4,
            min_distance=1.0,
            max_distance=5.0,
            min_obstacle_height=0.0,
            max_obstacle_height=1.0,
        )
        points = np.array([
            [0.5, 0.0, 0.5],
            [6.0, 0.0, 0.5],
            [2.0, 0.0, -0.5],
            [3.0, 0.0, 1.5],
            [4.0, 0.0, 0.5],
        ])

        output = mapper.compute(points)
        distances = output["angular_distances"]
        counts = output["angular_counts"]

        self.assertAlmostEqual(distances[0], 4.0)
        self.assertEqual(counts[0], 1)
        self.assertTrue(np.all(np.isnan(distances[1:])))
        self.assertTrue(np.all(counts[1:] == 0))


if __name__ == '__main__':
    unittest.main()

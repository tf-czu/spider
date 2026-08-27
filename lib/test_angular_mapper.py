# -*- coding: UTF-8 -*-

import unittest
import numpy as np

from angular_mapper import *


class TestAngularMapper(unittest.TestCase):

    def test_nearest_point__single_point_is_detected(self):
        """Test that nearest_point mode detects a single valid obstacle point."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_point",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=0.0,
            max_distance=10.0,
        )
        points = np.array([
            [2.0, 0.0, 0.5],
        ])
        output = mapper.compute(points)
        self.assertAlmostEqual(output["angular_distances"][0], 2.0)
        self.assertEqual(output["angular_counts"][0], 1)

    def test_nearest_cluster__single_point_is_not_detected(self):
        """Test that nearest_cluster mode ignores an isolated single point."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_cluster",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=0.0,
            max_distance=10.0,
            cluster_distance_threshold=0.2,
            min_cluster_points=3,
        )
        points = np.array([
            [2.0, 0.0, 0.5],
        ])
        output = mapper.compute(points)
        self.assertTrue(np.isnan(output["angular_distances"][0]))
        self.assertEqual(output["angular_counts"][0], 0)

    def test_nearest_cluster__valid_cluster_is_detected(self):
        """Test that nearest_cluster mode detects a valid radial cluster."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_cluster",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=0.0,
            max_distance=10.0,
            cluster_distance_threshold=0.2,
            min_cluster_points=3,
        )
        points = np.array([
            [2.00, 0.0, 0.3],
            [2.10, 0.0, 0.8],
            [2.18, 0.0, 1.2],
        ])
        output = mapper.compute(points)
        self.assertAlmostEqual(output["angular_distances"][0], 2.00)
        self.assertEqual(output["angular_counts"][0], 3)

    def test_nearest_cluster__nearest_valid_cluster_is_selected(self):
        """Test that the nearest valid cluster is selected when multiple clusters exist."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_cluster",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=0.0,
            max_distance=10.0,
            cluster_distance_threshold=0.2,
            min_cluster_points=3,
        )
        points = np.array([
            [1.00, 0.0, 0.4],
            [1.10, 0.0, 0.4],
            [1.15, 0.0, 0.4],
            [4.00, 0.0, 0.4],
            [4.10, 0.0, 0.4],
            [4.15, 0.0, 0.4],
        ])
        output = mapper.compute(points)
        self.assertAlmostEqual(output["angular_distances"][0], 1.00)
        self.assertEqual(output["angular_counts"][0], 3)

    def test_nearest_cluster__near_noise_does_not_hide_far_cluster(self):
        """Test that near isolated noise does not hide a farther valid cluster."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_cluster",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=0.0,
            max_distance=10.0,
            cluster_distance_threshold=0.2,
            min_cluster_points=3,
        )
        points = np.array([
            [1.00, 0.0, 0.4],
            [3.00, 0.0, 0.4],
            [3.10, 0.0, 0.4],
            [3.15, 0.0, 0.4],
        ])
        output = mapper.compute(points)
        self.assertAlmostEqual(output["angular_distances"][0], 3.00)
        self.assertEqual(output["angular_counts"][0], 3)

    def test_nearest_cluster__separated_points_do_not_form_cluster(self):
        """Test that points separated by large radial gaps do not form a valid cluster."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_cluster",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=0.0,
            max_distance=10.0,
            cluster_distance_threshold=0.2,
            min_cluster_points=3,
        )
        points = np.array([
            [1.0, 0.0, 0.5],
            [1.4, 0.0, 0.5],
            [1.8, 0.0, 0.5],
        ])
        output = mapper.compute(points)
        self.assertTrue(np.isnan(output["angular_distances"][0]))
        self.assertEqual(output["angular_counts"][0], 0)

    def test_points_outside_height_range_are_ignored(self):
        """Test that points outside the configured height range are ignored."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_point",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=0.0,
            max_distance=10.0,
        )
        points = np.array([
            [1.0, 0.0, 0.05],
            [2.0, 0.0, 2.50],
        ])
        output = mapper.compute(points)
        self.assertTrue(np.isnan(output["angular_distances"][0]))
        self.assertEqual(output["angular_counts"][0], 0)

    def test_points_outside_distance_range_are_ignored(self):
        """Test that points outside the configured distance range are ignored."""
        mapper = AngularMapper(
            number_of_bins=360,
            obstacle_detection_method="nearest_point",
            min_obstacle_z=0.1,
            max_obstacle_z=2.0,
            min_distance=1.0,
            max_distance=5.0,
        )
        points = np.array([
            [0.5, 0.0, 0.5],
            [6.0, 0.0, 0.5],
        ])
        output = mapper.compute(points)
        self.assertTrue(np.isnan(output["angular_distances"][0]))
        self.assertEqual(output["angular_counts"][0], 0)

    def test_invalid_obstacle_detection_method_raises_error(self):
        """Test that an unknown obstacle detection method raises ValueError."""
        with self.assertRaises(ValueError):
            AngularMapper(obstacle_detection_method="unknown")


if __name__ == "__main__":
    unittest.main()

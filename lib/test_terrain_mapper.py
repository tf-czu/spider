# -*- coding: UTF-8 -*-

import unittest
import os
import sys

THIS_DIR = os.path.dirname(__file__)
PROJECT_DIR = os.path.dirname(THIS_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

import numpy as np

from lib.terrain_mapper import *


class TestTerrainMapper(unittest.TestCase):

    def test_build_percentile_height_maps__one_cell(self):
        """
        Test that all points falling into one grid cell produce expected lower
        and upper percentile height values.
        """
        mapper = TerrainMapper(
            x_min=0.0,
            x_max=1.0,
            y_min=0.0,
            y_max=1.0,
            resolution=1.0,
            lower_percentile=0,
            upper_percentile=100,
        )
        points = np.array([
            [0.1, 0.1, 1.0],
            [0.2, 0.2, 2.0],
            [0.3, 0.3, 3.0],
        ])

        min_map, max_map, count_map = mapper.build_percentile_height_maps(points)

        self.assertEqual(min_map.shape, (1, 1))
        self.assertAlmostEqual(min_map[0, 0], 1.0)
        self.assertAlmostEqual(max_map[0, 0], 3.0)
        self.assertEqual(count_map[0, 0], 3)

    def test_build_percentile_height_maps__ignores_points_outside_grid(self):
        """
        Test that points outside the configured XY grid are ignored when
        building percentile height maps.
        """
        mapper = TerrainMapper(
            x_min=0.0,
            x_max=1.0,
            y_min=0.0,
            y_max=1.0,
            resolution=1.0,
            lower_percentile=0,
            upper_percentile=100,
        )
        points = np.array([
            [0.5, 0.5, 1.0],
            [-0.1, 0.5, 2.0],
            [1.0, 0.5, 3.0],
            [0.5, -0.1, 4.0],
            [0.5, 1.0, 5.0],
        ])

        min_map, max_map, count_map = mapper.build_percentile_height_maps(points)

        self.assertAlmostEqual(min_map[0, 0], 1.0)
        self.assertAlmostEqual(max_map[0, 0], 1.0)
        self.assertEqual(count_map[0, 0], 1)

    def test_build_slope_map__plane_rising_in_x_direction(self):
        """
        Test that a height map representing a plane rising in the X direction
        produces the expected X gradient and slope magnitude.
        """
        mapper = TerrainMapper(resolution=1.0)
        z_map = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0],
        ])

        dzdx_map, dzdy_map, slope_map = mapper.build_slope_map(z_map)

        self.assertAlmostEqual(dzdx_map[1, 1], 1.0)
        self.assertAlmostEqual(dzdy_map[1, 1], 0.0)
        self.assertAlmostEqual(slope_map[1, 1], 1.0)

    def test_compute__returns_all_grid_maps(self):
        """
        Test that compute returns all expected terrain grid maps with the
        correct shape.
        """
        mapper = TerrainMapper(
            x_min=0.0,
            x_max=3.0,
            y_min=0.0,
            y_max=3.0,
            resolution=1.0,
            lower_percentile=0,
            upper_percentile=100,
        )
        points = np.array([
            [0.5, 0.5, 0.0],
            [1.5, 0.5, 1.0],
            [2.5, 0.5, 2.0],
            [0.5, 1.5, 0.0],
            [1.5, 1.5, 1.0],
            [2.5, 1.5, 2.0],
            [0.5, 2.5, 0.0],
            [1.5, 2.5, 1.0],
            [2.5, 2.5, 2.0],
        ])

        output = mapper.compute(points)

        for key in ["min_map", "max_map", "dif_map", "count_map", "dzdx_map", "dzdy_map", "slope_map"]:
            self.assertIn(key, output)
            self.assertEqual(output[key].shape, (3, 3))

        self.assertAlmostEqual(output["slope_map"][1, 1], 1.0)

    def test_build_percentile_height_maps__unknown_sampling_method(self):
        """
        Test that an unknown height sampling method is rejected with a
        ValueError.
        """
        mapper = TerrainMapper(height_sampling_method="unknown")
        points = np.array([[0.0, 0.0, 0.0]])

        with self.assertRaises(ValueError):
            mapper.build_percentile_height_maps(points)


if __name__ == '__main__':
    unittest.main()

# -*- coding: UTF-8 -*-

import numpy as np


class HoleDetector:
    """
    Detects hole candidates in a terrain height map.

    A hole candidate is detected when a grid cell is significantly lower
    than its local neighborhood.
    """

    def __init__(
        self,
        neighborhood_radius = 1,
        min_hole_depth = 0.25,
        min_valid_neighbors = 5,
    ):
        """
        Args:
            neighborhood_radius (int):
                Radius of the local neighborhood in grid cells.

            min_hole_depth (float):
                Minimal height difference [m] between the neighborhood
                and the center cell required to mark a hole candidate.

            min_valid_neighbors (int):
                Minimal number of valid neighboring cells required
                to evaluate the center cell.
        """
        self.neighborhood_radius = neighborhood_radius
        self.min_hole_depth = min_hole_depth
        self.min_valid_neighbors = min_valid_neighbors

    def compute(self, min_map):
        """
        Computes hole candidates from a height map.

        Args:
            min_map (numpy.array):
                2D height map. Unknown cells are represented by np.nan.

        Returns:
            dict:
                Dictionary containing:
                    "hole_depth_map":
                        2D map containing how much lower each cell is
                        compared to its local neighborhood.

                    "hole_mask":
                        Boolean 2D map. True means that the cell is
                        considered a hole candidate.

                    "hole_fuzzy_mask":
                        2D map with values from 0.0 to 1.0 describing
                        how strongly each cell resembles a hole candidate.
        """
        nx, ny = min_map.shape

        hole_depth_map = np.full((nx, ny), np.nan)
        hole_mask = np.zeros((nx, ny), dtype=bool)
        hole_fuzzy_mask = np.zeros((nx, ny), dtype=float)

        r = self.neighborhood_radius

        for ix in range(r, nx - r):
            for iy in range(r, ny - r):
                center_height = min_map[ix, iy]

                if np.isnan(center_height):
                    continue

                x0 = ix - r
                x1 = ix + r + 1
                y0 = iy - r
                y1 = iy + r + 1

                neighborhood = min_map[x0:x1, y0:y1].copy()
                neighborhood[r, r] = np.nan

                valid_neighbors = neighborhood[~np.isnan(neighborhood)]

                if len(valid_neighbors) < self.min_valid_neighbors:
                    continue

                neighbor_height = np.mean(valid_neighbors)
                hole_depth = neighbor_height - center_height

                hole_depth_map[ix, iy] = hole_depth

                if self.min_hole_depth > 0:
                    hole_fuzzy = hole_depth / self.min_hole_depth
                    hole_fuzzy = max(0.0, min(1.0, hole_fuzzy))
                else:
                    hole_fuzzy = 0.0

                hole_fuzzy_mask[ix, iy] = hole_fuzzy

                if hole_depth >= self.min_hole_depth:
                    hole_mask[ix, iy] = True

        return {
            "hole_depth_map": hole_depth_map,
            "hole_mask": hole_mask,
            "hole_fuzzy_mask": hole_fuzzy_mask,
        }

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
        hole_detection_method = "mean",
        neighborhood_height_percentile = 20,
        max_neighbor_roughness = 0.25,
        roughness_lower_percentile = 10,
        roughness_upper_percentile = 90,
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

            hole_detection_method (str):
                Hole detection method.

                Supported values:
                    "mean":
                        Uses the mean height of neighboring cells.

                    "percentile":
                        Uses a percentile-based robust estimate of the
                        neighboring terrain height and rejects rough
                        neighborhoods.

            neighborhood_height_percentile (float):
                Percentile used to estimate local terrain height around
                the center cell. Used only by the "percentile" method.

            max_neighbor_roughness (float):
                Maximal allowed height roughness [m] in the neighborhood.
                Cells with rougher neighborhoods are ignored. Used only
                by the "percentile" method.

            roughness_lower_percentile (float):
                Lower percentile used to estimate neighborhood roughness.
                Used only by the "percentile" method.

            roughness_upper_percentile (float):
                Upper percentile used to estimate neighborhood roughness.
                Used only by the "percentile" method.
        """

        self.neighborhood_radius = neighborhood_radius
        self.min_hole_depth = min_hole_depth
        self.min_valid_neighbors = min_valid_neighbors
        self.hole_detection_method = hole_detection_method
        self.neighborhood_height_percentile = neighborhood_height_percentile
        self.max_neighbor_roughness = max_neighbor_roughness
        self.roughness_lower_percentile = roughness_lower_percentile
        self.roughness_upper_percentile = roughness_upper_percentile

        if self.hole_detection_method not in ("mean", "percentile"):
            raise ValueError(
                f"Unknown hole_detection_method: {self.hole_detection_method}"
            )

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

                    "neighbor_roughness_map":
                        2D map containing local neighborhood roughness.
                        This map is mainly useful for the "percentile"
                        method.
        """

        nx, ny = min_map.shape

        hole_depth_map = np.full((nx, ny), np.nan)
        neighbor_roughness_map = np.full((nx, ny), np.nan)
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

                if self.hole_detection_method == "mean":
                    neighbor_height = np.mean(valid_neighbors)

                elif self.hole_detection_method == "percentile":
                    rough_low, rough_high = np.percentile(
                        valid_neighbors,
                        [
                            self.roughness_lower_percentile,
                            self.roughness_upper_percentile,
                        ],
                    )

                    neighbor_roughness = rough_high - rough_low
                    neighbor_roughness_map[ix, iy] = neighbor_roughness

                    if neighbor_roughness > self.max_neighbor_roughness:
                        continue

                    neighbor_height = np.percentile(
                        valid_neighbors,
                        self.neighborhood_height_percentile,
                    )

                else:
                    raise ValueError(
                        f"Unknown hole_detection_method: "
                        f"{self.hole_detection_method}"
                    )

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
            "neighbor_roughness_map": neighbor_roughness_map,
        }

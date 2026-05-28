# -*- coding: UTF-8 -*-

import numpy as np


class AngularMapper:
    """
    Builds angular obstacle maps from point clouds.

    The mapper projects points into polar coordinates around the sensor
    and estimates the nearest obstacle distance for each angular bin.

    Input:
        point cloud of shape N x 3

    Output:
        angular obstacle map with one distance value per angular bin.
    """

    def __init__(
        self,
        angle_bins = 360,
        min_distance = 0.2,
        max_distance = 20.0,
        min_obstacle_height = -0.2,
        max_obstacle_height = 2.0,
    ):
        """
        Args:
            angle_bins (int):
                Number of angular bins covering 360 degrees.

            min_distance (float):
                Minimum accepted point distance in meters.

            max_distance (float):
                Maximum accepted point distance in meters.

            min_obstacle_height (float):
                Minimum accepted point height in meters.

            max_obstacle_height (float):
                Maximum accepted point height in meters.
        """
        self.angle_bins = angle_bins
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.min_obstacle_height = min_obstacle_height
        self.max_obstacle_height = max_obstacle_height

    def compute(self, points):
        """
        Computes an angular obstacle-distance map.

        Args:
            points (numpy.array):
                Point cloud of shape N x 3.

        Returns:
            dict:
                Dictionary containing:
                    "angular_distances" nearest obstacle distance per bin
                    "angular_counts" number of obstacle points per bin
        """
        angular_distances = np.full(self.angle_bins, np.nan)
        angular_counts = np.zeros(self.angle_bins, dtype=int)

        if points is None or len(points) == 0:
            return {
                "angular_distances": angular_distances,
                "angular_counts": angular_counts,
            }

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        distances = np.sqrt(x ** 2 + y ** 2)
        angles = np.degrees(np.arctan2(y, x))
        angles = (angles + 360.0) % 360.0

        mask = (
            (distances >= self.min_distance) &
            (distances <= self.max_distance) &
            (z >= self.min_obstacle_height) &
            (z <= self.max_obstacle_height) &
            np.isfinite(distances) &
            np.isfinite(angles) &
            np.isfinite(z)
        )

        distances = distances[mask]
        angles = angles[mask]

        if len(distances) == 0:
            return {
                "angular_distances": angular_distances,
                "angular_counts": angular_counts,
            }

        bin_indices = np.floor(angles / 360.0 * self.angle_bins).astype(int)
        bin_indices = np.clip(bin_indices, 0, self.angle_bins - 1)

        for bin_index, distance in zip(bin_indices, distances):
            angular_counts[bin_index] += 1

            if np.isnan(angular_distances[bin_index]):
                angular_distances[bin_index] = distance
            elif distance < angular_distances[bin_index]:
                angular_distances[bin_index] = distance

        return {
            "angular_distances": angular_distances,
            "angular_counts": angular_counts,
        }

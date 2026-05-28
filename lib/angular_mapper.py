# -*- coding: UTF-8 -*-

import numpy as np


class AngularMapper:
    """
    Builds an angular obstacle map from a point cloud.

    The output is a polar obstacle representation:
        angle -> distance to the nearest obstacle

    The obstacle detection method can be selected by the
    obstacle_detection_method parameter.

    Supported methods:
        "nearest_point":
            The nearest obstacle candidate point in each angular bin
            is considered an obstacle.

        "nearest_cluster":
            The nearest radial cluster of obstacle candidate points
            in each angular bin is considered an obstacle. A single
            isolated point is not considered an obstacle unless
            min_cluster_points is set to 1.
    """

    def __init__(
        self,
        number_of_bins = 360,
        obstacle_detection_method = "nearest_cluster",
        min_obstacle_z = 0.15,
        max_obstacle_z = 2.0,
        min_distance = 0.2,
        max_distance = 15.0,
        cluster_distance_threshold = 0.25,
        min_cluster_points = 3,
    ):
        """
        Args:
            number_of_bins (int):
                Number of angular bins.

            obstacle_detection_method (str):
                Obstacle detection method.
                Supported values are "nearest_point" and "nearest_cluster".

            min_obstacle_z (float):
                Minimal Z coordinate [m] for a point to be considered
                an obstacle candidate.

            max_obstacle_z (float):
                Maximal Z coordinate [m] for a point to be considered
                an obstacle candidate.

            min_distance (float):
                Minimal horizontal distance [m] from the sensor.

            max_distance (float):
                Maximal horizontal distance [m] from the sensor.

            cluster_distance_threshold (float):
                Maximal radial gap [m] between consecutive points
                belonging to the same obstacle cluster.
                Used only by the "nearest_cluster" method.

            min_cluster_points (int):
                Minimal number of points required for a cluster to be
                considered a valid obstacle.
                Used only by the "nearest_cluster" method.
        """
        self.number_of_bins = number_of_bins
        self.obstacle_detection_method = obstacle_detection_method
        self.min_obstacle_z = min_obstacle_z
        self.max_obstacle_z = max_obstacle_z
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.cluster_distance_threshold = cluster_distance_threshold
        self.min_cluster_points = min_cluster_points

        if self.obstacle_detection_method not in ("nearest_point", "nearest_cluster"):
            raise ValueError(
                f"Unknown obstacle_detection_method: {self.obstacle_detection_method}"
            )

    def compute(self, points):
        """
        Computes angular obstacle map from a point cloud.

        Args:
            points (numpy.array):
                Point cloud of shape N x 3.

        Returns:
            dict:
                Dictionary containing:
                    "angular_distances":
                        numpy.array of size number_of_bins.
                        Each value contains distance [m] to the nearest
                        detected obstacle in the corresponding direction.
                        If no obstacle is detected, value is np.nan.

                    "angular_counts":
                        numpy.array of size number_of_bins.
                        For the "nearest_point" method, the value is the
                        number of obstacle candidate points in the bin.
                        For the "nearest_cluster" method, the value is the
                        number of points in the detected nearest obstacle
                        cluster.
        """
        points = np.asarray(points)

        angular_distances = np.full(self.number_of_bins, np.nan)
        angular_counts = np.zeros(self.number_of_bins, dtype=int)
        distances_per_bin = [[] for _ in range(self.number_of_bins)]

        for point in points:
            x, y, z = point

            if z < self.min_obstacle_z:
                continue
            if z > self.max_obstacle_z:
                continue

            distance = np.sqrt(x*x + y*y)

            if distance < self.min_distance:
                continue
            if distance > self.max_distance:
                continue

            angle = np.degrees(np.arctan2(y, x))
            angle = (angle + 360.0) % 360.0
            bin_index = int(angle / 360.0 * self.number_of_bins)

            if 0 <= bin_index < self.number_of_bins:
                distances_per_bin[bin_index].append(distance)

        if self.obstacle_detection_method == "nearest_point":
            self._compute_nearest_point_map(
                distances_per_bin,
                angular_distances,
                angular_counts,
            )
        elif self.obstacle_detection_method == "nearest_cluster":
            self._compute_nearest_cluster_map(
                distances_per_bin,
                angular_distances,
                angular_counts,
            )

        return {
            "angular_distances": angular_distances,
            "angular_counts": angular_counts,
        }

    def _compute_nearest_point_map(self, distances_per_bin, angular_distances, angular_counts):
        """
        Fills angular map using the nearest obstacle candidate point.

        Args:
            distances_per_bin (list of list of float):
                Candidate obstacle distances grouped by angular bin.

            angular_distances (numpy.array):
                Output array to be filled with nearest distances.

            angular_counts (numpy.array):
                Output array to be filled with candidate point counts.
        """
        for bin_index in range(self.number_of_bins):
            distances = distances_per_bin[bin_index]

            if not distances:
                continue

            angular_distances[bin_index] = min(distances)
            angular_counts[bin_index] = len(distances)

    def _compute_nearest_cluster_map(self, distances_per_bin, angular_distances, angular_counts):
        """
        Fills angular map using the nearest valid radial cluster.

        Args:
            distances_per_bin (list of list of float):
                Candidate obstacle distances grouped by angular bin.

            angular_distances (numpy.array):
                Output array to be filled with nearest cluster distances.

            angular_counts (numpy.array):
                Output array to be filled with nearest cluster sizes.
        """
        for bin_index in range(self.number_of_bins):
            distances = distances_per_bin[bin_index]

            if not distances:
                continue

            distances.sort()

            cluster_start_distance = distances[0]
            cluster_last_distance = distances[0]
            cluster_count = 1

            for distance in distances[1:]:
                if distance - cluster_last_distance <= self.cluster_distance_threshold:
                    cluster_last_distance = distance
                    cluster_count += 1
                else:
                    if cluster_count >= self.min_cluster_points:
                        angular_distances[bin_index] = cluster_start_distance
                        angular_counts[bin_index] = cluster_count
                        break

                    cluster_start_distance = distance
                    cluster_last_distance = distance
                    cluster_count = 1

            if np.isnan(angular_distances[bin_index]):
                if cluster_count >= self.min_cluster_points:
                    angular_distances[bin_index] = cluster_start_distance
                    angular_counts[bin_index] = cluster_count

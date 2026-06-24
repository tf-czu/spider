# -*- coding: UTF-8 -*-

import time
import numpy as np


class LidarInterpreter:
    """
    Interprets raw lidar range scans directly, without converting them
    to point clouds.

    Each incoming scan is processed independently and immediately produces
    angular obstacle and hole maps.

    The number of angular directions is derived directly from the number of
    columns in the lidar scan. Therefore, one scan column corresponds to one
    angular output element.

    Obstacle candidates are detected as vertical clusters of similar range
    values in one lidar column.

    Hole candidates are detected as large range jumps between vertically
    adjacent pixels in one lidar column.
    """

    def __init__(
        self,
        range_unit=0.001,
        min_range=0.3,
        max_range=15.0,
        obstacle_range_tolerance=0.25,
        min_obstacle_pixels=3,
        hole_min_near_range=0.5,
        hole_min_range_jump=1.0,
        hole_min_range_ratio=1.5,
        measure_execution_time=True,
    ):
        """
        Args:
            range_unit (float):
                Conversion factor from raw lidar range values to meters.

            min_range (float):
                Minimal valid range [m].

            max_range (float):
                Maximal valid range [m].

            obstacle_range_tolerance (float):
                Maximal range difference [m] between vertically adjacent
                pixels belonging to the same obstacle cluster.

            min_obstacle_pixels (int):
                Minimal number of vertically connected similar pixels required
                to detect an obstacle.

            hole_min_near_range (float):
                Minimal range [m] of the nearer pixel involved in a hole jump.
                Too near candidates for holes are more likely to be measurement
                errors.

            hole_min_range_jump (float):
                Minimal absolute range difference [m] between vertically
                adjacent pixels required to detect a hole candidate.
                Roughly corresponds to the "size of the hole".

            hole_min_range_ratio (float):
                Minimal ratio between far and near range required to detect
                a hole candidate.
                Makes the detector less sensitive to small relative range
                changes.

            measure_execution_time (bool):
                If True, execution times of scan computations are measured.
        """
        self.range_unit = range_unit
        self.min_range = min_range
        self.max_range = max_range

        self.obstacle_range_tolerance = obstacle_range_tolerance
        self.min_obstacle_pixels = min_obstacle_pixels

        self.hole_min_near_range = hole_min_near_range
        self.hole_min_range_jump = hole_min_range_jump
        self.hole_min_range_ratio = hole_min_range_ratio

        self.measure_execution_time = measure_execution_time
        self.execution_times = []

        self.scan_shape = None
        self.num_rows = None
        self.num_angles = None

    def update(self, timestamp, scan):
        """
        Processes one lidar range scan.

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the incoming lidar scan.

            scan (numpy.array):
                Lidar range scan of shape H x W.

        Returns:
            dict:
                Output maps computed from the given scan.
        """
        if scan is None:
            return None

        t0 = time.perf_counter()
        output = self.compute_scan(scan)
        dt = time.perf_counter() - t0

        if self.measure_execution_time:
            self.execution_times.append(dt)

        output["timestamp"] = timestamp

        return output


    def compute_scan(self, scan):
        """
        Computes obstacle and hole candidates from one lidar range scan.

        Args:
            scan (numpy.array):
                Lidar range scan of shape H x W.

        Returns:
            dict:
                Dictionary containing angular obstacle and hole maps computed
                from the given scan.
        """
        ranges = np.asarray(scan, dtype=float) * self.range_unit

        if ranges.ndim != 2:
            raise ValueError(f"Expected 2D lidar scan H x W, got shape {ranges.shape}")

        self.check_scan_shape(ranges)

        obstacle_output = self.compute_obstacle_map(ranges)
        hole_output = self.compute_hole_map(ranges)

        output = {}
        output.update(obstacle_output)
        output.update(hole_output)
        return output

    def check_scan_shape(self, ranges):
        """
        Initializes and validates lidar scan geometry.
        """
        if self.scan_shape is None:
            self.num_rows, self.num_angles = self.scan_shape = ranges.shape
            return
        if ranges.shape != self.scan_shape:
            raise ValueError(
                f"Unexpected lidar scan shape {ranges.shape}, expected {self.scan_shape}"
            )

    def compute_obstacle_map(self, ranges):
        """
        Computes angular obstacle map from one range image.

        Args:
            ranges (numpy.array):
                Lidar range image in meters.

        Returns:
            dict:
                Dictionary containing obstacle distance, count, strength,
                and pixel mask maps.
        """
        obstacle_distances = np.full(self.num_angles, np.nan)
        obstacle_counts = np.zeros(self.num_angles, dtype=int)
        obstacle_strength = np.zeros(self.num_angles, dtype=float)
        obstacle_pixel_mask = np.zeros((self.num_rows, self.num_angles), dtype=bool)

        for col in range(self.num_angles):
            cluster_start = None
            cluster_last_range = None
            cluster_count = 0
            cluster_min_range = None

            for row in range(self.num_rows):
                value = ranges[row, col]

                if (value < self.min_range) or (value > self.max_range):
                    # pokud pixel neni platny (vzdalenost mimo povoleny rozsah)
                    # uzavru cluster
                    if cluster_count >= self.min_obstacle_pixels:
                        self.store_obstacle_cluster(
                            col,
                            cluster_start,
                            row,
                            cluster_min_range,
                            cluster_count,
                            obstacle_distances,
                            obstacle_counts,
                            obstacle_strength,
                            obstacle_pixel_mask,
                        )
                    # pripravim novy cluster
                    cluster_start = None
                    cluster_last_range = None
                    cluster_count = 0
                    cluster_min_range = None
                elif cluster_start is None:
                    # pokud jeste zadny cluster nebyl zalozen,
                    # pripravim novy cluster s hodnotou, kterou ted vidim
                    cluster_start = row
                    cluster_last_range = value
                    cluster_count = 1
                    cluster_min_range = value
                elif abs(value - cluster_last_range) <= self.obstacle_range_tolerance:
                    # pokud je hodnota pixelu podobna hodnote predchoziho pixelu,
                    # pridam dalsi pixel do clusteru
                    cluster_last_range = value
                    cluster_count += 1
                    cluster_min_range = min(cluster_min_range, value)
                else:
                    # pokud je hodnota pixelu prilis vzdalena hodnote predchoziho pixelu,
                    # uzaviram cluster
                    if cluster_count >= self.min_obstacle_pixels:
                        self.store_obstacle_cluster(
                            col,
                            cluster_start,
                            row,
                            cluster_min_range,
                            cluster_count,
                            obstacle_distances,
                            obstacle_counts,
                            obstacle_strength,
                            obstacle_pixel_mask,
                        )
                    cluster_start = row
                    cluster_last_range = value
                    cluster_count = 1
                    cluster_min_range = value
                # end for
            # vyresim posledni mozny cluster, ktery mohl zustat neuzavreny,
            # kdyz for cyklus skoncil
            if cluster_count >= self.min_obstacle_pixels:
                self.store_obstacle_cluster(
                    col,
                    cluster_start,
                    self.num_rows,
                    cluster_min_range,
                    cluster_count,
                    obstacle_distances,
                    obstacle_counts,
                    obstacle_strength,
                    obstacle_pixel_mask,
                )
        return {
            "obstacle_distances": obstacle_distances,
            "obstacle_counts": obstacle_counts,
            "obstacle_strength": obstacle_strength,
            "obstacle_pixel_mask": obstacle_pixel_mask,
        }

    def compute_hole_map(self, ranges):
        """
        Computes angular hole-candidate map from one range image.

        Args:
            ranges (numpy.array):
                Lidar range image in meters.

        Returns:
            dict:
                Dictionary containing hole distance, count, strength,
                and pixel mask maps.
        """
        hole_distances = np.full(self.num_angles, np.nan)
        hole_counts = np.zeros(self.num_angles, dtype=int)
        hole_strength = np.zeros(self.num_angles, dtype=float)
        hole_pixel_mask = np.zeros((self.num_rows - 1, self.num_angles), dtype=bool)

        for row in range(self.num_rows - 1):
            r1 = ranges[row, :]
            r2 = ranges[row + 1, :]

            valid = (
                (r1 >= self.min_range) &
                (r1 <= self.max_range) &
                (r2 >= self.min_range) &
                (r2 <= self.max_range)
            )

            near = np.minimum(r1, r2)
            far = np.maximum(r1, r2)
            jump = far - near
            ratio = far / np.maximum(near, 1e-9)

            candidates = (
                valid &
                (near >= self.hole_min_near_range) &
                (jump >= self.hole_min_range_jump) &
                (ratio >= self.hole_min_range_ratio)
            )

            hole_pixel_mask[row, :] = candidates
            candidate_columns = np.nonzero(candidates)[0]

            for col in candidate_columns:
                distance = near[col]
                strength = min(
                    jump[col] / self.hole_min_range_jump,
                    ratio[col] / self.hole_min_range_ratio,
                )
                strength = max(0.0, min(1.0, strength))

                self.store_nearest_detection(
                    col,
                    distance,
                    1,
                    strength,
                    hole_distances,
                    hole_counts,
                    hole_strength,
                )

        return {
            "hole_distances": hole_distances,
            "hole_counts": hole_counts,
            "hole_strength": hole_strength,
            "hole_pixel_mask": hole_pixel_mask,
        }

    def store_obstacle_cluster(
        self,
        col,
        row_start,
        row_end,
        distance,
        count,
        obstacle_distances,
        obstacle_counts,
        obstacle_strength,
        obstacle_pixel_mask,
    ):
        """
        Stores one detected obstacle cluster into angular output maps.
        """
        strength = min(1.0, count / self.min_obstacle_pixels)

        self.store_nearest_detection(
            col,
            distance,
            count,
            strength,
            obstacle_distances,
            obstacle_counts,
            obstacle_strength,
        )

        obstacle_pixel_mask[row_start:row_end, col] = True

    def store_nearest_detection(
        self,
        col,
        distance,
        count,
        strength,
        obstacle_distances,
        obstacle_counts,
        obstacle_strength,
    ):
        """
        Stores a detection if it is closer than the current detection
        in the same angular direction.
        """
        if np.isnan(obstacle_distances[col]) or distance < obstacle_distances[col]:
            obstacle_distances[col] = distance
            obstacle_counts[col] = count
            obstacle_strength[col] = strength
        elif distance == obstacle_distances[col]:
            obstacle_counts[col] += count
            obstacle_strength[col] = max(obstacle_strength[col], strength)

    def get_num_angles(self):
        """
        Returns the number of angular directions used by the latest scan.
        """
        return self.num_angles

    def get_angle_step_degrees(self):
        """
        Returns angular resolution [deg] derived from the latest scan.
        """
        if self.num_angles is None:
            return None
        return 360.0 / self.num_angles

    def get_execution_times(self):
        """
        Returns execution time statistics in milliseconds.
        """
        if not self.execution_times:
            return "no timing data"

        values = np.asarray(self.execution_times)
        return (
            f"avg={1000*np.mean(values):.1f} ms "
            f"min={1000*np.min(values):.1f} ms "
            f"max={1000*np.max(values):.1f} ms"
        )

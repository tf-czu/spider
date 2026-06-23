# -*- coding: UTF-8 -*-

import time
import numpy as np


class LidarInterpreter:
    """
    Interprets raw lidar range scans directly, without converting them
    to point clouds.

    The interpreter periodically processes accumulated lidar scans and
    produces angular obstacle and hole maps.

    Obstacle candidates are detected as vertical clusters of similar range
    values in one lidar column.

    Hole candidates are detected as large range jumps between vertically
    adjacent pixels in one lidar column.
    """

    def __init__(
        self,
        output_frequency=1.0,
        range_unit=0.001,
        number_of_bins=360,
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
            output_frequency (float):
                Frequency [Hz] at which output maps are generated.

            range_unit (float):
                Conversion factor from raw lidar range values to meters.

            number_of_bins (int):
                Number of angular bins in output maps.

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
                To be more relaxed when detecting *distant* candidates for
                holes.

            measure_execution_time (bool):
                If True, execution times of output computations are measured.
        """
        self.output_frequency = output_frequency
        self.output_period = 1.0 / output_frequency
        self.last_output_time = None

        self.range_unit = range_unit
        self.number_of_bins = number_of_bins
        self.min_range = min_range
        self.max_range = max_range

        self.obstacle_range_tolerance = obstacle_range_tolerance
        self.min_obstacle_pixels = min_obstacle_pixels

        self.hole_min_near_range = hole_min_near_range
        self.hole_min_range_jump = hole_min_range_jump
        self.hole_min_range_ratio = hole_min_range_ratio

        self.measure_execution_time = measure_execution_time
        self.execution_times = []

        self.list_of_accumulated_scans = []

    def update(self, timestamp, scan):
        """
        Adds one lidar range scan to the accumulation buffer.

        Once enough time has elapsed since the previous output,
        the accumulated scans are processed.

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the incoming lidar scan.

            scan (numpy.array):
                Lidar range scan of shape H x W.

        Returns:
            dict:
                Output maps, if a new output was generated.

            None:
                If not enough time has elapsed yet.
        """
        self.list_of_accumulated_scans.append(scan)

        if self.last_output_time is None:
            self.last_output_time = timestamp
            return None

        if (timestamp - self.last_output_time).total_seconds() >= self.output_period:
            t0 = time.perf_counter()
            output = self.process_accumulated_scans(timestamp)
            dt = time.perf_counter() - t0

            if self.measure_execution_time:
                self.execution_times.append(dt)

            self.last_output_time = timestamp
            return output

        return None

    def process_accumulated_scans(self, timestamp):
        """
        Processes accumulated lidar range scans.

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the generated output maps.

        Returns:
            dict:
                Dictionary containing angular obstacle and hole maps.
        """
        if not self.list_of_accumulated_scans:
            return None

        obstacle_distances = np.full(self.number_of_bins, np.nan)
        obstacle_counts = np.zeros(self.number_of_bins, dtype=int)
        obstacle_strength = np.zeros(self.number_of_bins, dtype=float)

        hole_distances = np.full(self.number_of_bins, np.nan)
        hole_counts = np.zeros(self.number_of_bins, dtype=int)
        hole_strength = np.zeros(self.number_of_bins, dtype=float)

        latest_obstacle_pixel_mask = None
        latest_hole_pixel_mask = None

        for scan in self.list_of_accumulated_scans:
            scan_output = self.compute_scan(scan)

            self.merge_nearest_distances(
                obstacle_distances,
                obstacle_counts,
                obstacle_strength,
                scan_output["obstacle_distances"],
                scan_output["obstacle_counts"],
                scan_output["obstacle_strength"],
            )

            self.merge_nearest_distances(
                hole_distances,
                hole_counts,
                hole_strength,
                scan_output["hole_distances"],
                scan_output["hole_counts"],
                scan_output["hole_strength"],
            )

            latest_obstacle_pixel_mask = scan_output["obstacle_pixel_mask"]
            latest_hole_pixel_mask = scan_output["hole_pixel_mask"]

        output = {
            "timestamp": timestamp,
            "obstacle_distances": obstacle_distances,
            "obstacle_counts": obstacle_counts,
            "obstacle_strength": obstacle_strength,
            "hole_distances": hole_distances,
            "hole_counts": hole_counts,
            "hole_strength": hole_strength,
            "obstacle_pixel_mask": latest_obstacle_pixel_mask,
            "hole_pixel_mask": latest_hole_pixel_mask,
        }

        self.list_of_accumulated_scans.clear()

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

        obstacle_output = self.compute_obstacle_map(ranges)
        hole_output = self.compute_hole_map(ranges)

        output = {}
        output.update(obstacle_output)
        output.update(hole_output)
        return output

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
        h, w = ranges.shape

        obstacle_distances = np.full(self.number_of_bins, np.nan)
        obstacle_counts = np.zeros(self.number_of_bins, dtype=int)
        obstacle_strength = np.zeros(self.number_of_bins, dtype=float)
        obstacle_pixel_mask = np.zeros((h, w), dtype=bool)

        for col in range(w):
            column = ranges[:, col]
            valid = (column >= self.min_range) & (column <= self.max_range)

            cluster_start = None
            cluster_last_range = None
            cluster_count = 0
            cluster_min_range = None

            for row in range(h):
                value = column[row]

                if not valid[row]:
                    if cluster_count >= self.min_obstacle_pixels:
                        self.store_obstacle_cluster(
                            col,
                            w,
                            cluster_start,
                            row,
                            cluster_min_range,
                            cluster_count,
                            obstacle_distances,
                            obstacle_counts,
                            obstacle_strength,
                            obstacle_pixel_mask,
                        )
                    cluster_start = None
                    cluster_last_range = None
                    cluster_count = 0
                    cluster_min_range = None
                    continue

                if cluster_start is None:
                    cluster_start = row
                    cluster_last_range = value
                    cluster_count = 1
                    cluster_min_range = value
                    continue

                if abs(value - cluster_last_range) <= self.obstacle_range_tolerance:
                    cluster_last_range = value
                    cluster_count += 1
                    cluster_min_range = min(cluster_min_range, value)
                else:
                    if cluster_count >= self.min_obstacle_pixels:
                        self.store_obstacle_cluster(
                            col,
                            w,
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

            if cluster_count >= self.min_obstacle_pixels:
                self.store_obstacle_cluster(
                    col,
                    w,
                    cluster_start,
                    h,
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
        h, w = ranges.shape

        hole_distances = np.full(self.number_of_bins, np.nan)
        hole_counts = np.zeros(self.number_of_bins, dtype=int)
        hole_strength = np.zeros(self.number_of_bins, dtype=float)
        hole_pixel_mask = np.zeros((h - 1, w), dtype=bool)

        for row in range(h - 1):
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
                bin_index = self.column_to_bin(col, w)
                distance = near[col]
                strength = min(
                    jump[col] / self.hole_min_range_jump,
                    ratio[col] / self.hole_min_range_ratio,
                )
                strength = max(0.0, min(1.0, strength))

                self.store_nearest_detection(
                    bin_index,
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
        width,
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
        bin_index = self.column_to_bin(col, width)
        strength = min(1.0, count / self.min_obstacle_pixels)

        self.store_nearest_detection(
            bin_index,
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
        bin_index,
        distance,
        count,
        strength,
        distances,
        counts,
        strengths,
    ):
        """
        Stores a detection if it is closer than the current detection
        in the same angular bin.
        """
        if np.isnan(distances[bin_index]) or distance < distances[bin_index]:
            distances[bin_index] = distance
            counts[bin_index] = count
            strengths[bin_index] = strength
        elif distance == distances[bin_index]:
            counts[bin_index] += count
            strengths[bin_index] = max(strengths[bin_index], strength)

    def merge_nearest_distances(
        self,
        target_distances,
        target_counts,
        target_strength,
        source_distances,
        source_counts,
        source_strength,
    ):
        """
        Merges angular detections by keeping the nearest distance
        in each angular bin.
        """
        valid = np.isfinite(source_distances)

        update_empty = valid & np.isnan(target_distances)
        target_distances[update_empty] = source_distances[update_empty]
        target_counts[update_empty] = source_counts[update_empty]
        target_strength[update_empty] = source_strength[update_empty]

        update_closer = valid & np.isfinite(target_distances) & (source_distances < target_distances)
        target_distances[update_closer] = source_distances[update_closer]
        target_counts[update_closer] = source_counts[update_closer]
        target_strength[update_closer] = source_strength[update_closer]

    def column_to_bin(self, col, width):
        """
        Converts lidar image column index to angular bin index.
        """
        bin_index = int(col / width * self.number_of_bins)
        return max(0, min(self.number_of_bins - 1, bin_index))

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

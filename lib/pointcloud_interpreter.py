# -*- coding: UTF-8 -*-

import numpy as np


class PointCloudInterpreter:
    """
    Interprets incoming point clouds and produces terrain maps.

    The interpreter accumulates multiple point clouds over time and
    periodically processes them into higher-level representations such as:

        - traversability map
        - obstacle map
        - slope map
        - height map
        - percentile-based terrain maps

    Input:
        point cloud of shape N x 3

    Output:
        terrain-related maps computed at a lower frequency than the
        incoming lidar scans.
    """

    def __init__(self, output_frequency = 1.0):
        """
        Args:
            output_frequency (float):
                Frequency [Hz] at which output maps are generated.

                Example:
                    lidar input frequency  = 10 Hz
                    map output frequency   = 1 Hz

                In this case, approximately 10 incoming point clouds
                are accumulated before one terrain map update.
        """

        # desired output frequency [Hz]
        self.output_frequency = output_frequency

        # desired output period [s]
        self.output_period = 1.0 / output_frequency

        # timestamp of the last generated output map
        self.last_output_time = None  # [datetime.timedelta]

        # number of remembered outputs
        self.remembered_outputs = 3

        # FIFO storage for previously generated outputs
        self.fifo_of_outputs = []

        # accumulated point clouds waiting for processing
        self.list_of_accumulated_point_clouds = []

        # parameters to compute the maps
        self.x_min = -10.0
        self.x_max = 10.0
        self.y_min = -10.0
        self.y_max = 10.0
        self.resolution = 0.5
        self.lower_percentile = 10
        self.upper_percentile = 90

    def update(self, timestamp, points):
        """
        Adds one point cloud to the internal accumulation buffer.

        Once enough time has elapsed since the previous output,
        the accumulated point clouds are processed into terrain maps.

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the incoming point cloud.

            points (numpy.array):
                Point cloud of shape N x 3.

        Returns:
            (dict) ... if new maps were generated
            `None` ... if not enough time has elapsed yet
        """

        # store incoming point cloud
        self.list_of_accumulated_point_clouds.append(points)

        # initialize output timer
        if self.last_output_time is None:
            self.last_output_time = timestamp
            return None

        # process accumulated data periodically
        elif (timestamp - self.last_output_time).total_seconds() >= self.output_period:
            output = self.process_accumulated_point_clouds(timestamp)
            self.last_output_time = timestamp
            return output

        else:
            return None

    def process_accumulated_point_clouds(self, timestamp):
        """
        Processes accumulated point clouds and generates terrain maps.

        This method is expected to:
            - merge accumulated point clouds
            - compute height maps
            - compute percentile-based terrain statistics
            - compute slope maps
            - detect obstacles
            - generate traversability maps

        Args:
            timestamp (datetime.timedelta):
                Timestamp of the generated output maps.
        """

        if not self.list_of_accumulated_point_clouds:
            return None

        points = np.vstack(self.list_of_accumulated_point_clouds)

        min_map, max_map, count_map = self.build_percentile_height_maps(points)

        dif_map = max_map - min_map
        dif_map[count_map == 0] = np.nan

        output = {
            "timestamp": timestamp,
            "count_map": count_map,
            "min_map": min_map,
            "max_map": max_map,
            "dif_map": dif_map,
        }

        self.fifo_of_outputs.append(output)

        while len(self.fifo_of_outputs) > self.remembered_outputs:
            self.fifo_of_outputs.pop(0)

        self.list_of_accumulated_point_clouds.clear()

        return output

    def build_percentile_height_maps(self, points):
        nx = int((self.x_max - self.x_min) / self.resolution)
        ny = int((self.y_max - self.y_min) / self.resolution)

        z_values = [[[] for _ in range(ny)] for _ in range(nx)]

        for point in points:
            x, y, z = point

            if x < self.x_min or x >= self.x_max:
                continue
            if y < self.y_min or y >= self.y_max:
                continue

            ix = int((x - self.x_min) / self.resolution)
            iy = int((y - self.y_min) / self.resolution)

            if 0 <= ix < nx and 0 <= iy < ny:
                z_values[ix][iy].append(z)

        min_map = np.full((nx, ny), np.nan)
        max_map = np.full((nx, ny), np.nan)
        count_map = np.zeros((nx, ny), dtype=int)

        for ix in range(nx):
            for iy in range(ny):
                values = z_values[ix][iy]

                if not values:
                    continue

                count_map[ix, iy] = len(values)
                min_map[ix, iy] = np.percentile(
                    values,
                    self.lower_percentile,
                    method = "lower",
                )
                max_map[ix, iy] = np.percentile(
                    values,
                    self.upper_percentile,
                    method = "higher",
                )

        return min_map, max_map, count_map

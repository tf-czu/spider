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

        # sampling method to compute the percentiles in build_percentile_height_maps():
        #   "simple"    ... use all values
        #   "reservoir" ... online random reservoir sampling
        #   "first_n"   ... first N values observed in the cell
        #   "last_n"    ... last N values observed in the cell
        self.height_sampling_method = "simple"
        self.height_sample_size = 100

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
        dzdx_map, dzdy_map, slope_map = self.build_slope_map(min_map)

        dif_map = max_map - min_map
        dif_map[count_map == 0] = np.nan

        output = {
            "timestamp": timestamp,
            "min_map": min_map,
            "max_map": max_map,
            "dif_map": dif_map,
            "dzdx_map": dzdx_map,
            "dzdy_map": dzdy_map,
            "slope_map": slope_map,
        }

        self.fifo_of_outputs.append(output)

        while len(self.fifo_of_outputs) > self.remembered_outputs:
            self.fifo_of_outputs.pop(0)

        self.list_of_accumulated_point_clouds.clear()

        return output

    def build_percentile_height_maps(self, points):
        """
        Builds percentile-based height maps from a point cloud.

        The point cloud is projected into a 2D grid in the XY plane.
        For each grid cell, Z values are collected and robust lower
        and upper height estimates are computed using percentiles.

        The number of Z values used per grid cell can be controlled
        by self.height_sampling_method.

        Supported sampling methods:
            "simple":
                Use all Z values.

            "reservoir":
                Keep a random sample of at most self.height_sample_size
                values per cell using online reservoir sampling.

            "first_n":
                Keep only the first self.height_sample_size values
                observed in each cell.

            "last_n":
                Keep only the last self.height_sample_size values
                observed in each cell.

        Args:
            points (numpy.array):
                Point cloud of shape N x 3.

        Returns:
            min_map (numpy.array):
                2D map containing lower-percentile height estimates.

            max_map (numpy.array):
                2D map containing upper-percentile height estimates.

            count_map (numpy.array):
                2D map containing the total number of points observed
                in each grid cell.
        """
        nx = int((self.x_max - self.x_min) / self.resolution)
        ny = int((self.y_max - self.y_min) / self.resolution)

        z_values = [[[] for _ in range(ny)] for _ in range(nx)]
        z_seen_count = np.zeros((nx, ny), dtype=int)

        method = self.height_sampling_method
        sample_size = self.height_sample_size

        if method not in ("simple", "reservoir", "first_n", "last_n"):
            raise ValueError(f"Unknown height_sampling_method: {method}")

        for point in points:
            x, y, z = point

            if x < self.x_min or x >= self.x_max:
                continue
            if y < self.y_min or y >= self.y_max:
                continue

            ix = int((x - self.x_min) / self.resolution)
            iy = int((y - self.y_min) / self.resolution)

            if 0 <= ix < nx and 0 <= iy < ny:
                z_seen_count[ix, iy] += 1
                cell = z_values[ix][iy]

                if method == "simple":
                    cell.append(z)

                elif method == "first_n":
                    if len(cell) < sample_size:
                        cell.append(z)

                elif method == "last_n":
                    if len(cell) >= sample_size:
                        cell.pop(0)
                    cell.append(z)

                elif method == "reservoir":
                    seen = z_seen_count[ix, iy]

                    if len(cell) < sample_size:
                        cell.append(z)
                    else:
                        j = np.random.randint(0, seen)
                        if j < sample_size:
                            cell[j] = z

        min_map = np.full((nx, ny), np.nan)
        max_map = np.full((nx, ny), np.nan)
        count_map = np.zeros((nx, ny), dtype=int)

        for ix in range(nx):
            for iy in range(ny):
                values = z_values[ix][iy]

                if not values:
                    continue

                low, high = np.percentile(
                    values,
                    [self.lower_percentile, self.upper_percentile],
                    method = "linear",
                )

                min_map[ix, iy] = low
                max_map[ix, iy] = high
                count_map[ix, iy] = z_seen_count[ix, iy]

        return min_map, max_map, count_map

    def build_slope_map(self, z_map):
        """
        Computes terrain slope from a height map.

        The method estimates height gradients in X and Y directions
        using central differences between neighboring grid cells.
        The final slope map contains the magnitude of the local height
        gradient.

        Args:
            z_map (numpy.array):
                2D height map. Unknown cells are represented by np.nan.

        Returns:
            dzdx_map (numpy.array):
                2D map of height gradient in the X direction.

            dzdy_map (numpy.array):
                2D map of height gradient in the Y direction.

            slope_map (numpy.array):
                2D map containing the magnitude of the terrain gradient
                (tangent of the slope angle in the steepest direction)
        """

        nx, ny = z_map.shape

        dzdx_map = np.full((nx, ny), np.nan)
        dzdy_map = np.full((nx, ny), np.nan)
        slope_map = np.full((nx, ny), np.nan)

        for ix in range(1, nx - 1):
            for iy in range(1, ny - 1):
                zl = z_map[ix - 1, iy]
                zr = z_map[ix + 1, iy]
                zd = z_map[ix, iy - 1]
                zu = z_map[ix, iy + 1]

                if not np.isnan(zl) and not np.isnan(zr):
                    dzdx_map[ix, iy] = (zr - zl) / (2.0 * self.resolution)

                if not np.isnan(zd) and not np.isnan(zu):
                    dzdy_map[ix, iy] = (zu - zd) / (2.0 * self.resolution)

                if not np.isnan(dzdx_map[ix, iy]) and not np.isnan(dzdy_map[ix, iy]):
                    slope_map[ix, iy] = np.sqrt(
                        dzdx_map[ix, iy] ** 2 + dzdy_map[ix, iy] ** 2
                    )

        return dzdx_map, dzdy_map, slope_map

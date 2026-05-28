# -*- coding: UTF-8 -*-

import numpy as np


class TerrainMapper:
    """
    Builds grid-based terrain maps from accumulated point clouds.

    The mapper projects a point cloud into a 2D grid in the XY plane
    and computes terrain-related maps such as:

        - lower-percentile height map
        - upper-percentile height map
        - height difference map
        - slope map

    Input:
        point cloud of shape N x 3

    Output:
        dictionary containing grid-based terrain maps.
    """

    def __init__(
        self,
        x_min = -10.0,
        x_max = 10.0,
        y_min = -10.0,
        y_max = 10.0,
        resolution = 0.5,
        lower_percentile = 10,
        upper_percentile = 90,
        height_sampling_method = "simple",
        height_sample_size = 300,
    ):
        """
        Args:
            x_min, x_max, y_min, y_max (float):
                Boundaries of the local grid map in meters.

            resolution (float):
                Size of one grid cell in meters.

            lower_percentile (float):
                Percentile used for the lower height estimate.

            upper_percentile (float):
                Percentile used for the upper height estimate.

            height_sampling_method (str):
                Method used to limit the number of height samples per cell.

                Supported values:
                    "simple"     use all values
                    "reservoir"  online random reservoir sampling
                    "first_n"    first N values observed in the cell
                    "last_n"     last N values observed in the cell

            height_sample_size (int):
                Maximum number of samples used by sampling methods
                other than "simple".
        """
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution
        self.lower_percentile = lower_percentile
        self.upper_percentile = upper_percentile
        self.height_sampling_method = height_sampling_method
        self.height_sample_size = height_sample_size

    def compute(self, points):
        """
        Computes terrain maps from one accumulated point cloud.

        Args:
            points (numpy.array):
                Point cloud of shape N x 3.

        Returns:
            dict:
                Dictionary containing:
                    "min_map"   lower-percentile height map
                    "max_map"   upper-percentile height map
                    "dif_map"   height difference map
                    "count_map" number of observed points per grid cell
                    "dzdx_map"  height gradient in X direction
                    "dzdy_map"  height gradient in Y direction
                    "slope_map" magnitude of the terrain gradient
        """
        min_map, max_map, count_map = self.build_percentile_height_maps(points)
        dif_map = max_map - min_map
        dif_map[count_map == 0] = np.nan

        dzdx_map, dzdy_map, slope_map = self.build_slope_map(min_map)

        return {
            "min_map": min_map,
            "max_map": max_map,
            "dif_map": dif_map,
            "count_map": count_map,
            "dzdx_map": dzdx_map,
            "dzdy_map": dzdy_map,
            "slope_map": slope_map,
        }

    def build_percentile_height_maps(self, points):
        """
        Builds percentile-based height maps from a point cloud.

        The point cloud is projected into a 2D grid in the XY plane.
        For each grid cell, Z values are collected and robust lower
        and upper height estimates are computed using percentiles.

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
                    method="linear",
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
                (tangent of the slope angle in the steepest direction).
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

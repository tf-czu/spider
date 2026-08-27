# -*- coding: UTF-8 -*-

import numpy as np
import ouster.sdk as ous


class LidarToPointCloud:
    """
    Converts raw lidar scans into 3D point clouds.

    The class uses Ouster sensor metadata to initialize
    the sensor geometry and XYZ lookup table.

    Processing pipeline:

        lidar range scan (H x W)
            |
        XYZ lookup table
            |
        xyz image (H x W x 3)
            |
        reshape
            |
        point cloud (N x 3)

    Invalid points with zero range are automatically removed.
    """

    def __init__(self, metadata):
        """
        Initializes lidar geometry and XYZ conversion tables.

        Args:
            metadata (str):
                JSON string containing Ouster sensor metadata.

                The metadata contain:
                    - lidar geometry
                    - beam angles
                    - lidar mode
                    - scan dimensions
                    - calibration parameters
        """

        # sensor geometry description
        self.info = ous.core.SensorInfo(metadata)

        # XYZ lookup table used for scan -> point cloud conversion
        self.xyz_lut = ous.core.XYZLut(self.info)

    def convert(self, scan):
        """
        Converts one lidar scan into a 3D point cloud.

        Args:
            scan (numpy.array):
                Lidar range scan of size H x W.

                Example:
                    32 x 1024

                Values represent measured distances.

        Returns:
            numpy.array:
                Point cloud of size N x 3.

                There are N points, each represented as:
                    [x, y, z]

                Coordinates are expressed in meters
                in the sensor coordinate system.
        """

        # convert input to numpy array
        ranges = np.asarray(scan)

        # verify expected scan dimensionality
        if ranges.ndim != 2:
            raise ValueError(
                f"Expected 2D lidar scan H x W, got shape {ranges.shape}"
            )

        # scan dimensions
        h, w = ranges.shape

        # create temporary Ouster lidar scan structure
        lidar_scan = ous.core.LidarScan(h, w)

        # copy range image into the Ouster structure
        lidar_scan.field(ous.core.ChanField.RANGE)[:] = ranges

        # convert ranges to xyz coordinates
        # xyz shape:
        #     H x W x 3
        #
        # xyz[h, w] = [x, y, z]
        xyz = self.xyz_lut(lidar_scan)

        # convert:
        #     H x W x 3
        # to:
        #     N x 3
        points = xyz.reshape(-1, 3)

        # flatten range image:
        #     H x W
        # to:
        #     N
        ranges_flat = ranges.reshape(-1)

        # keep only valid points
        # points with range == 0 are invalid
        mask = ranges_flat > 0

        points = points[mask]

        return points

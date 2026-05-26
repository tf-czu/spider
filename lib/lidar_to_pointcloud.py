# -*- coding: UTF-8 -*-

import numpy as np
import ouster.sdk as ous


class LidarToPointCloud:
    """
        Converts lidar scans to 3D point clouds.
    """

    def __init__(self, metadata):
        """
            Args:
                metadata (str): JSON containing the lidar sensor geometry info
        """
        self.info = ous.core.SensorInfo(metadata) # geometrie senzoru
        self.xyz_lut = ous.core.XYZLut(self.info) # vytvoreni lookup tabulky

    def convert(self, scan):
        """
            Args:
                scan (numpy.array): lidar scan ... array of size H x W

            Returns:
                numpy.array: point cloud ... array of size N x 3
        """
        ranges = np.asarray(scan)

        if ranges.ndim != 2:
            raise ValueError(f"Expected 2D lidar scan H x W, got shape {ranges.shape}")

        h, w = ranges.shape

        lidar_scan = ous.core.LidarScan(h, w)
        lidar_scan.field(ous.core.ChanField.RANGE)[:] = ranges

        xyz = self.xyz_lut(lidar_scan) # shape HxWx3 ... xyz[h, w] = [x, y, z]
        points = xyz.reshape(-1, 3) # prevod HxWx3 -> Nx3 (-1 znamena automaticke dopocitani, tady: HxW -> N)

        # odfiltrovani bodu s nulovymi vzdalenostmi
        ranges_flat = ranges.reshape(-1) # prevod HxW -> N
        mask = ranges_flat > 0
        points = points[mask]

        return points

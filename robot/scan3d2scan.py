"""
  Convert 3D lidar scan to 2D scan.
  Input: numpy array (32, 1024) of distances in mm (Ouster lidar).
  Output: list of 1024 distances in mm.
"""
import numpy as np

from osgar.node import Node


class Scan3DToScan2D(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('scan2d')

        # Convert input values to mm.
        self.H = config.get('H', 1.2)*1000      # lidar height above ground (m)
        self.L = config.get('L', 15)*1000       # max horizontal range (m)
        self.min_dist = config.get('min_dist', 0.8)*1000  # ignore below limit (m)
        self.num_rays = config.get('num_rays', 16)   # number of bottom rays
        self.max_angle = config.get('max_angle', 22.5)  # max ray angle (deg)

        # angles for the bottom rays: row 16 -> 0 deg, row 31 -> 22.5 deg
        self.phi = np.radians(np.linspace(0.0, self.max_angle, self.num_rays))

        # max valid distance per ray
        self.max_range = self.H / (np.cos(self.phi) * (self.H / self.L + np.tan(self.phi)))

    def on_scan3d(self, data):
        assert data.shape == (32, 1024), data.shape

        # take bottom rays (rows 16-31)
        scan = data[16:16 + self.num_rays]

        # mask of valid measurements: within [min_dist, max_range] per ray
        valid = (scan >= self.min_dist) & (scan <= self.max_range[:, np.newaxis])

        # minimum valid distance per column, 0.0 if none valid
        masked = np.where(valid, scan, np.inf)
        result = masked.min(axis=0)
        result[np.isinf(result)] = 0.0

        self.publish('scan2d', result.tolist())


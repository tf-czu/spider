"""
  Convert 3D lidar scan to 2D scan.
  Calculation considerations.
        We calculate the distances at which the lidar can detect the ground, depending on the beam angle.
        If the lidar returns a shorter distance, this is interpreted as an obstacle.
        The calculation also takes into account the slope angle (the robot’s tilt is not considered)
        and obstacle_size to ignore minor obstacles.
"""
import numpy as np
import math

from osgar.node import Node


class Scan3DToScan2D(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('scan2d')

        # Convert input values to mm.
        self.lidar_height = config.get('lidar_height', 1.3)*1000    # lidar height above ground (m)
        self.slope = math.radians(config.get('slope', 0))           # possible terrain slope (deg)
        self.lidar_range = config.get('lidar_range', 15)*1000       # max horizontal range (m)
        self.obstacle_size = config.get('obstacle_size', 0.3)*1000  # bottom offset (m)
        self.blind_zone = config.get('blind_zone', 0.8)*1000        # ignore below limit (m)
        self.num_rays = config.get('num_rays', 16)                  # number of bottom rays
        self.max_angle = config.get('max_angle', 22.5)              # max ray angle (deg)
        self.flip_scan = config.get('flip_scan', False)             # we require a positive direction of rotation

        # angles for the bottom rays: row 16 -> 0 deg, row 31 -> 22.5 deg
        self.phi = np.radians(np.linspace(0.0, self.max_angle, self.num_rays))

        # max valid distance per ray
        # when denominator is 0 (e.g. slope=0 and phi=0), the ray is parallel
        # to the ground and never reaches it -> max_range = inf
        denominator = np.cos(self.phi) * (np.tan(self.slope) + np.tan(self.phi))
        self.max_range = np.full_like(denominator, np.inf)
        nonzero = ~np.isclose(denominator, 0.0)
        self.max_range[nonzero] = (self.lidar_height - self.obstacle_size) / denominator[nonzero]

    def on_scan3d(self, data):
        assert data.shape == (32, 1024), data.shape

        # take bottom rays (rows 16-31)
        scan = data[16:16 + self.num_rays]

        # mask of valid measurements: within [min_dist, max_range] per ray
        valid = (scan >= self.blind_zone) & (scan <= self.max_range[:, np.newaxis])

        # mask invalid measurements
        masked = np.where(valid, scan, np.inf)

        # project slanted distances to xy-plane (cos(phi) per ray)
        masked = masked * np.cos(self.phi)[:, np.newaxis]

        # minimum valid distance per column, 0.0 if none valid
        result = masked.min(axis=0)
        result[np.isinf(result)] = 0.0

        # clip too far obstacles
        result[result > self.lidar_range] = 0.0
        if self.flip_scan:
            result = result[::-1]

        self.publish('scan2d', result.tolist())


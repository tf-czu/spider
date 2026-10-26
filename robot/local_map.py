"""
  Local grid map centered on the robot.
  Input:  scan   - 2D scan from Scan3DToScan2D (distances in mm per column, 0 = no obstacle)
          pose3d - [[x, y, z] (mm), [qx, qy, qz, qw]]
  Output: grid   - square list of lists grid[i][j], values 1 = traversable, 0 = obstacle.
          Index i grows along the x axis (forward if heading_aligned, otherwise world x),
          index j grows along the y axis (left); the robot is in the middle of the map.
"""
import math

import numpy as np

from osgar.node import Node


def quaternion_yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class LocalMap(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('grid')

        self.size = config.get('size', 20.0)                        # map side (m)
        self.resolution = config.get('resolution', 0.2)             # cell size (m)
        self.n = int(round(self.size / self.resolution))
        self.lidar_range = config.get('lidar_range', 15) * 1000     # max range (mm)
        self.angle_offset = math.radians(config.get('angle_offset', 0))  # angle of first column (deg)
        self.fov = math.radians(config.get('fov', 360))             # field of view covered by the scan (deg)
        self.flip_scan = config.get('flip_scan', False)             # scan is in the negative direction of rotation
        self.heading_aligned = config.get('heading_aligned', True)  # x axis of the map = robot heading
        self.unknown_value = int(config.get('unknown_value', 1))    # value of cells without measurement
        self.accumulate = config.get('accumulate', False)           # keep older measurements

        self.pose = None  # (x mm, y mm, yaw rad)
        # world aligned memory (-1 unknown, 0 obstacle, 1 free), used only for accumulation
        self.memory = np.full((self.n, self.n), -1, dtype=np.int8)
        self.memory_origin = None  # integer world cell index of the memory center

    def on_pose3d(self, data):
        xyz, quat = data
        self.pose = (xyz[0], xyz[1], quaternion_yaw(quat))

    def scan_to_cells(self, scan, yaw):
        """Return a robot centered (world aligned) map: -1 unknown, 0 obstacle, 1 free."""
        scan = np.asarray(scan, dtype=float)
        count = len(scan)
        angles = self.angle_offset + np.arange(count) * self.fov / count
        if self.flip_scan:
            angles = -angles
        angles = angles + yaw

        dist = np.where(scan > 0, np.minimum(scan, self.lidar_range), self.lidar_range)
        hit = (scan > 0) & (scan <= self.lidar_range)

        step = self.resolution * 1000 / 2
        radii = np.arange(0, self.lidar_range + step, step)
        r = radii[np.newaxis, :]
        free = r < dist[:, np.newaxis]
        xs = r * np.cos(angles)[:, np.newaxis]
        ys = r * np.sin(angles)[:, np.newaxis]

        grid = np.full((self.n, self.n), -1, dtype=np.int8)
        cell = self.resolution * 1000
        half = self.n // 2

        def to_index(v):
            return np.floor(v / cell + 0.5).astype(int) + half

        def paint(mask, x, y, value):
            i, j = to_index(x[mask]), to_index(y[mask])
            ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
            grid[i[ok], j[ok]] = value

        paint(free, xs, ys, 1)
        hx = dist[hit] * np.cos(angles[hit])
        hy = dist[hit] * np.sin(angles[hit])
        i, j = to_index(hx), to_index(hy)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        grid[i[ok], j[ok]] = 0  # obstacle wins over free
        return grid

    def update_memory(self, new, x, y):
        """Shift the world aligned memory to the new robot cell and merge the new measurement."""
        cell = self.resolution * 1000
        origin = (int(round(x / cell)), int(round(y / cell)))
        if self.memory_origin is not None:
            di, dj = origin[0] - self.memory_origin[0], origin[1] - self.memory_origin[1]
            shifted = np.full_like(self.memory, -1)
            n = self.n
            if abs(di) < n and abs(dj) < n:
                shifted[max(0, -di):min(n, n - di), max(0, -dj):min(n, n - dj)] = \
                    self.memory[max(0, di):min(n, n + di), max(0, dj):min(n, n + dj)]
            self.memory = shifted
        self.memory_origin = origin
        known = new >= 0
        self.memory[known] = new[known]
        return self.memory

    def on_scan(self, data):
        if self.pose is None:
            if self.accumulate or not self.heading_aligned:
                return  # pose is required
            x = y = yaw = 0.0
        else:
            x, y, yaw = self.pose

        if self.accumulate:
            # world aligned scan, the memory is world aligned too
            world = self.update_memory(self.scan_to_cells(data, yaw), x, y)
            grid = self.rotate_to_robot(world, yaw) if self.heading_aligned else world
        elif self.heading_aligned:
            grid = self.scan_to_cells(data, 0.0)  # robot frame directly
        else:
            grid = self.scan_to_cells(data, yaw)

        out = np.where(grid < 0, self.unknown_value, grid)
        self.publish('grid', out.astype(int).tolist())

    def rotate_to_robot(self, world, yaw):
        """Resample world aligned map into the robot frame (nearest neighbour)."""
        half = self.n // 2
        idx = np.arange(self.n) - half
        ii, jj = np.meshgrid(idx, idx, indexing='ij')
        c, s = math.cos(yaw), math.sin(yaw)
        wi = np.rint(ii * c - jj * s).astype(int) + half
        wj = np.rint(ii * s + jj * c).astype(int) + half
        ok = (wi >= 0) & (wi < self.n) & (wj >= 0) & (wj < self.n)
        out = np.full((self.n, self.n), -1, dtype=np.int8)
        out[ok] = world[wi[ok], wj[ok]]
        return out

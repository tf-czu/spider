"""
  Local grid map centered on the robot.
  Input:  scan   - 2D scan from Scan3DToScan2D (distances in mm per column, 0 = no obstacle)
          pose3d - [[x, y, z] (m), [qx, qy, qz, qw]]
  Output: gridmap - square list of lists gridmap[i][j], values 0 = traversable, 1 = obstacle, -1 = unknown.
  The gridmap is a merge of layers: the current map from the lidar (optionally accumulated) and the
  permanent obstacles (stored in world cells, independent of the robot centered window).
  An obstacle in any layer is an obstacle in the result; permanent layer never changes free cells otherwise.
          Index i grows along the x axis (forward if heading_aligned, otherwise world x),
          index j grows along the y axis (left); the robot is in the middle of the map.
"""
import math

import numpy as np

from osgar.lib.quaternion import heading
from osgar.node import Node


def arrow_vector(heading, length):
    """Heading arrow (dx, dy) in plot data coordinates: the horizontal axis is y (growing to the left,
    the axis itself is reversed via imshow extent) and the vertical axis is x (growing up)."""
    return length * math.sin(heading), length * math.cos(heading)


class LocalMap(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('gridmap')

        self.size = config.get('size', 20.0)                        # map side (m)
        self.resolution = config.get('resolution', 0.2)             # cell size (m)
        self.n = int(round(self.size / self.resolution))
        self.lidar_range = config.get('lidar_range', 15) * 1000     # max range (mm)
        self.angle_offset = math.radians(config.get('angle_offset', 0))  # angle of first column (deg)
        self.fov = math.radians(config.get('fov', 360))             # field of view covered by the scan (deg)
        self.flip_scan = config.get('flip_scan', False)             # scan is in the negative direction of rotation
        self.heading_aligned = config.get('heading_aligned', True)  # x axis of the map = robot heading
        self.accumulate = config.get('accumulate', False)           # keep older measurements

        self.pose = None  # (x mm, y mm, yaw rad)
        # world aligned memory (-1 unknown, 0 free, 1 obstacle), used only for accumulation
        self.memory = np.full((self.n, self.n), -1, dtype=np.int8)
        self.memory_origin = None  # integer world cell index of the memory center
        # permanent obstacles: world cell index (round(position / cell size)) -> source of the obstacle
        self.permanent = {}
        self.permanent_cells = np.zeros((0, 2), dtype=int)
        self.history = []  # (grid, heading in map frame) collected only in verbose mode for draw()

    def on_pose3d(self, data):
        xyz, quat = data
        self.pose = (xyz[0] * 1000, xyz[1] * 1000, heading(quat))  # pose3d is in meters, scan in mm

    def scan_to_cells(self, scan, yaw):
        """Return a robot centered (world aligned) map: -1 unknown, 0 free, 1 obstacle."""
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

        paint(free, xs, ys, 0)
        hx = dist[hit] * np.cos(angles[hit])
        hy = dist[hit] * np.sin(angles[hit])
        i, j = to_index(hx), to_index(hy)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        grid[i[ok], j[ok]] = 1  # obstacle wins over free
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

    def add_permanent(self, cells, source):
        """Add permanent obstacles given as world cell indices (i, j) = round(x / cell), round(y / cell)."""
        for cell in cells:
            self.permanent[(int(cell[0]), int(cell[1]))] = source
        self.permanent_cells = np.array(list(self.permanent), dtype=int).reshape(-1, 2)

    def render_permanent(self, x, y, yaw):
        """Permanent obstacles (1) in the robot centered window, other cells are unknown (-1).
        The cells are transformed directly (not resampled), so thin obstacles cannot disappear."""
        grid = np.full((self.n, self.n), -1, dtype=np.int8)
        cell = self.resolution * 1000
        dx = self.permanent_cells[:, 0] * cell - x
        dy = self.permanent_cells[:, 1] * cell - y
        if self.heading_aligned:
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = c * dx + s * dy, -s * dx + c * dy
        i = np.floor(dx / cell + 0.5).astype(int) + self.n // 2
        j = np.floor(dy / cell + 0.5).astype(int) + self.n // 2
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        grid[i[ok], j[ok]] = 1
        return grid

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

        if len(self.permanent_cells) and self.pose is not None:
            grid = np.where(self.render_permanent(x, y, yaw) == 1, 1, grid)

        out = grid.astype(int)
        if self.verbose:
            self.history.append((out, 0.0 if self.heading_aligned else yaw))
        self.publish('gridmap', out.tolist())

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

    def draw(self):
        """
        Animate the published local maps (requires --verbose, otherwise nothing is recorded).
        Black = obstacle, white = traversable, gray = unknown; the robot is the red dot in the center,
        the arrow shows the robot heading, the x axis of the map points up and the y axis to the left.
        """
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap

        if not self.history:
            print("No grids to draw, run osgar.replay with --verbose")
            return
        half = self.size / 2
        fig, ax = plt.subplots()
        image = ax.imshow(self.history[0][0][:, ::-1], origin='lower', cmap=ListedColormap(['gray', 'white', 'black']), vmin=-1, vmax=1,
                          extent=(half, -half, -half, half))
        ax.plot([0], [0], 'ro')
        ax.set_xlabel('y (m)')
        ax.set_ylabel('x (m)')
        arrow = None
        length = self.size / 8
        for index, (grid, heading) in enumerate(self.history):
            if not plt.fignum_exists(fig.number):
                break
            if arrow is not None:
                arrow.remove()
            dx, dy = arrow_vector(heading, length)
            arrow = ax.arrow(0, 0, dx, dy,
                             color='red', width=length / 12, head_width=length / 3, length_includes_head=True)
            image.set_data(grid[:, ::-1])
            ax.set_title(f"local map {index + 1}/{len(self.history)}")
            plt.pause(0.05)
        if plt.fignum_exists(fig.number):
            plt.show()

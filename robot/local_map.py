"""
  Local grid map centered on the robot.
  Input:  scan    - 2D scan from Scan3DToScan2D (distances in mm per column, 0 = no obstacle)
          pose3d  - [[x, y, z] (m), [qx, qy, qz, qw]]
          bumpers - list of 0/1 flags, one per configured bumper segment (e.g. [left, right])
  Output: gridmap - square list of lists gridmap[i][j], values 0 = traversable, 1 = obstacle, -1 = unknown.
  The gridmap is a merge of layers: the current map from the lidar (optionally accumulated) and the
  permanent obstacles (stored in world cells, independent of the robot centered window).
  An obstacle in any layer is an obstacle in the result; permanent layer never changes free cells otherwise.
          Index i grows along the x axis (forward if heading_aligned, otherwise world x),
          index j grows along the y axis (left); the robot is in the middle of the map.
"""
import logging
import math

import numpy as np
from scipy import ndimage

from osgar.lib.quaternion import heading
from osgar.node import Node

g_logger = logging.getLogger(__name__)


def shift_array(array, di, dj, fill):
    """Shift array content by (-di, -dj) cells: the cell [i + di, j + dj] moves to [i, j]."""
    n = array.shape[0]
    shifted = np.full_like(array, fill)
    if abs(di) < n and abs(dj) < n:
        shifted[max(0, -di):min(n, n - di), max(0, -dj):min(n, n - dj)] = \
            array[max(0, di):min(n, n + di), max(0, dj):min(n, n + dj)]
    return shifted


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
        # bumper segments [x1, y1, x2, y2] in meters relative to the robot center (x forward, y left),
        # the order corresponds to the order of the flags in the bumpers message
        self.bumpers = np.array(config.get('bumpers', []), dtype=float).reshape(-1, 4) * 1000  # mm
        self.bumpers_mismatch_reported = False

        # detection of stable obstacles in the lidar data, promoted to the permanent layer
        self.detect_permanent = config.get('detect_permanent', False)
        self.min_hits = config.get('min_hits', 30)                # scans with an obstacle in the cell
        self.min_ratio = config.get('min_ratio', 0.8)             # hits / (hits + misses)
        self.min_duration = config.get('min_duration', 3.0)       # seconds since the first hit
        self.min_cells = config.get('min_cells', 5)               # size of the connected component
        self.analysis_period = config.get('analysis_period', 1.0)  # seconds between analyses
        self.clear_misses = max(1, config.get('clear_misses', 20))  # free observations in a row to forget
        shape = (self.n, self.n)
        self.hits = np.zeros(shape, dtype=np.int32)
        self.misses = np.zeros(shape, dtype=np.int32)
        self.free_streak = np.zeros(shape, dtype=np.int32)
        self.first_seen = np.full(shape, np.nan)
        self.stats_origin = None  # integer world cell index of the center of the statistics window
        self.last_analysis = None

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

    def scan_to_cells(self, scan, yaw, shift=(0.0, 0.0)):
        """Return a robot centered (world aligned) map: -1 unknown, 0 free, 1 obstacle.
        The shift (mm) is the position of the robot relative to the center of the window."""
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

        def to_index(v, offset):
            return np.floor((v + offset) / cell + 0.5).astype(int) + half

        def paint(mask, x, y, value):
            i, j = to_index(x[mask], shift[0]), to_index(y[mask], shift[1])
            ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
            grid[i[ok], j[ok]] = value

        paint(free, xs, ys, 0)
        hx = dist[hit] * np.cos(angles[hit])
        hy = dist[hit] * np.sin(angles[hit])
        i, j = to_index(hx, shift[0]), to_index(hy, shift[1])
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        grid[i[ok], j[ok]] = 1  # obstacle wins over free
        return grid

    def window_origin(self, x, y):
        """World cell index of the robot, the center of the world aligned windows."""
        cell = self.resolution * 1000
        return int(math.floor(x / cell + 0.5)), int(math.floor(y / cell + 0.5))

    def update_memory(self, new, x, y):
        """Shift the world aligned memory to the new robot cell and merge the new measurement."""
        origin = self.window_origin(x, y)
        if self.memory_origin is not None:
            di, dj = origin[0] - self.memory_origin[0], origin[1] - self.memory_origin[1]
            self.memory = shift_array(self.memory, di, dj, -1)
        self.memory_origin = origin
        known = new >= 0
        self.memory[known] = new[known]
        return self.memory

    def add_permanent(self, cells, source):
        """Add permanent obstacles given as world cell indices (i, j) = round(x / cell), round(y / cell)."""
        for cell in cells:
            self.permanent[(int(cell[0]), int(cell[1]))] = source
        self.permanent_cells = np.array(list(self.permanent), dtype=int).reshape(-1, 2)

    def remove_permanent(self, cells):
        for cell in cells:
            del self.permanent[cell]
        self.permanent_cells = np.array(list(self.permanent), dtype=int).reshape(-1, 2)

    def update_stats(self, obs, origin, now):
        """Update per cell counters from a world aligned observation (-1 unknown, 0 free, 1 obstacle)."""
        if self.stats_origin is not None and origin != self.stats_origin:
            di, dj = origin[0] - self.stats_origin[0], origin[1] - self.stats_origin[1]
            self.hits = shift_array(self.hits, di, dj, 0)
            self.misses = shift_array(self.misses, di, dj, 0)
            self.free_streak = shift_array(self.free_streak, di, dj, 0)
            self.first_seen = shift_array(self.first_seen, di, dj, np.nan)
        self.stats_origin = origin
        obstacle = obs == 1
        free = obs == 0
        self.first_seen[obstacle & np.isnan(self.first_seen)] = now
        self.hits[obstacle] += 1
        self.misses[free] += 1
        self.free_streak[free] += 1
        self.free_streak[obstacle] = 0
        forget = self.free_streak >= self.clear_misses  # the obstacle is gone, start from scratch
        self.hits[forget] = 0
        self.misses[forget] = 0
        self.first_seen[forget] = np.nan

    def analyse_stats(self, obs, now):
        """Promote large stable obstacles to the permanent layer, remove lidar ones confirmed free."""
        origin = self.stats_origin
        half = self.n // 2
        ratio = self.hits / np.maximum(self.hits + self.misses, 1)
        with np.errstate(invalid='ignore'):
            stable = (now - self.first_seen) >= self.min_duration  # False for NaN
        candidate = (obs == 1) & (self.hits >= self.min_hits) & (ratio >= self.min_ratio) & stable
        labels, _ = ndimage.label(candidate, structure=np.ones((3, 3)))
        keep = np.bincount(labels.ravel()) >= self.min_cells
        keep[0] = False
        cells = np.argwhere(keep[labels]) - half + origin
        self.add_permanent([c for c in cells if (int(c[0]), int(c[1])) not in self.permanent], 'lidar')

        cleared = []
        for (ci, cj), source in self.permanent.items():
            i, j = ci - origin[0] + half, cj - origin[1] + half
            if source == 'lidar' and 0 <= i < self.n and 0 <= j < self.n \
                    and self.free_streak[i, j] >= self.clear_misses:
                cleared.append((ci, cj))
        if cleared:
            self.remove_permanent(cleared)

    def on_bumpers(self, data):
        """Pressed bumpers create permanent obstacles along their segments at the current robot pose."""
        if len(data) != len(self.bumpers):
            if not self.bumpers_mismatch_reported:
                g_logger.warning("Bumpers message has %d items but %d bumpers are configured, ignoring",
                                 len(data), len(self.bumpers))
                self.bumpers_mismatch_reported = True
            return
        if self.pose is None or not any(data):
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        cell = self.resolution * 1000
        cells = []
        for pressed, (x1, y1, x2, y2) in zip(data, self.bumpers):
            if not pressed:
                continue
            count = int(math.ceil(math.hypot(x2 - x1, y2 - y1) / (cell / 2))) + 1
            bx = np.linspace(x1, x2, count)
            by = np.linspace(y1, y2, count)
            wx = x + c * bx - s * by
            wy = y + s * bx + c * by
            cells.extend(zip(np.floor(wx / cell + 0.5).astype(int), np.floor(wy / cell + 0.5).astype(int)))
        self.add_permanent(cells, 'bumper')

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

        now = self.time.total_seconds() if self.time is not None else 0.0
        observed = None
        if self.pose is not None and (self.accumulate or self.detect_permanent):
            origin = self.window_origin(x, y)
            cell = self.resolution * 1000
            observed = self.scan_to_cells(data, yaw, (x - origin[0] * cell, y - origin[1] * cell))
        if self.pose is not None and self.detect_permanent:
            self.update_stats(observed, origin, now)
            if self.last_analysis is None:
                self.last_analysis = now
            elif now - self.last_analysis >= self.analysis_period:
                self.last_analysis = now
                self.analyse_stats(observed, now)

        if self.accumulate:
            # world aligned scan, the memory is world aligned too
            world = self.update_memory(observed, x, y)
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

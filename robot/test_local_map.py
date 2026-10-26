import math
import unittest
from unittest.mock import MagicMock

import numpy as np

from osgar.bus import Bus

from robot.local_map import LocalMap


def make_app(config=None):
    bus = Bus(MagicMock())
    app = LocalMap(config=config or {}, bus=bus.handle('app'))
    tester = bus.handle('tester')
    bus.connect('app.grid', 'tester.grid')
    return app, tester


def grid_of(tester):
    _, channel, data = tester.queue.get_nowait()
    assert channel == 'grid'
    return np.array(data)


class TestLocalMap(unittest.TestCase):
    def test_size(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5})
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(grid.shape, (20, 20))
        self.assertTrue((grid == 1).all())

    def test_obstacle_ahead(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4})
        scan = [0] * 360
        scan[0] = 2000  # straight ahead, 2 m
        app.on_scan(scan)
        grid = grid_of(tester)
        self.assertEqual(grid[10 + 4, 10], 0)
        self.assertEqual(grid[10 + 2, 10], 1)
        self.assertEqual(grid.sum(), grid.size - 1)

    def test_obstacle_left(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4})
        scan = [0] * 360
        scan[90] = 2000
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[10, 14], 0)

    def test_flip_scan(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4, 'flip_scan': True})
        scan = [0] * 360
        scan[90] = 2000
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[10, 6], 0)

    def test_yaw(self):
        scan = [0] * 360
        scan[0] = 2000
        yaw = math.pi / 2
        pose = [[0, 0, 0], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]]

        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4})
        app.on_pose3d(pose)
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[14, 10], 0)  # robot frame

        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4,
                                'heading_aligned': False})
        app.on_pose3d(pose)
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[10, 14], 0)  # world frame

    def test_unknown_value(self):
        app, tester = make_app({'size': 4, 'resolution': 0.5, 'lidar_range': 1, 'unknown_value': 0})
        app.on_scan([0] * 4)
        grid = grid_of(tester)
        self.assertEqual(grid[0, 0], 0)

    def test_accumulate_shift(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4, 'accumulate': True,
                                'unknown_value': 1})
        app.on_scan([0] * 360)
        self.assertTrue(tester.queue.empty())  # no pose yet
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        scan = [0] * 360
        scan[0] = 2000
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[14, 10], 0)
        app.on_pose3d([[1000, 0, 0], [0, 0, 0, 1]])
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(grid[12, 10], 1)
        app.on_scan([0] * 360)  # same pose, nothing changes
        grid_of(tester)


if __name__ == '__main__':
    unittest.main()

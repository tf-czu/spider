import math
import unittest
from unittest.mock import MagicMock

import numpy as np

from osgar.bus import Bus

from robot.local_map import LocalMap, arrow_vector


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
        app.on_pose3d([[1.0, 0, 0], [0, 0, 0, 1]])
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(grid[12, 10], 1)
        app.on_scan([0] * 360)  # same pose, nothing changes
        grid_of(tester)

    def make_accumulating(self, size=10):
        app, tester = make_app({'size': size, 'resolution': 0.5, 'lidar_range': 4, 'accumulate': True})
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        scan = [0] * 360
        scan[0] = 2000  # obstacle 2 m ahead
        app.on_scan(scan)
        half = size * 2 // 2
        self.assertEqual(grid_of(tester)[half + 4, half], 0)
        return app, tester, half

    def test_obstacle_kept_when_hidden_behind_closer_obstacle(self):
        app, tester, half = self.make_accumulating()
        app.on_scan([1000] * 360)  # wall 1 m around the robot, the old obstacle is not visible
        grid = grid_of(tester)
        self.assertEqual(grid[half + 4, half], 0)
        self.assertEqual(grid[half + 2, half], 0)  # the wall itself

    def test_obstacle_kept_when_out_of_lidar_range(self):
        app, tester, half = self.make_accumulating(size=20)
        app.on_pose3d([[-3.0, 0, 0], [0, 0, 0, 1]])  # the obstacle is 5 m away, the range is 4 m
        app.on_scan([0] * 360)
        self.assertEqual(grid_of(tester)[half + 10, half], 0)
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.on_scan([0] * 360)  # robot returns and sees free space
        self.assertEqual(grid_of(tester)[half + 4, half], 1)

    def test_obstacle_removed_when_lidar_sees_free_space(self):
        app, tester, _ = self.make_accumulating()
        app.on_scan([0] * 360)  # lidar confirms free space in the whole range
        self.assertEqual(grid_of(tester)[14, 10], 1)

    def test_obstacle_removed_when_lidar_sees_farther(self):
        app, tester, _ = self.make_accumulating()
        scan = [0] * 360
        scan[0] = 3500  # the obstacle moved away, the beam passes through the old cell
        app.on_scan(scan)
        grid = grid_of(tester)
        self.assertEqual(grid[14, 10], 1)
        self.assertEqual(grid[17, 10], 0)

    def test_angle_offset(self):
        # Ouster column 0 points backwards, angle_offset 180 turns it into the rear of the robot
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4, 'angle_offset': 180})
        scan = [0] * 360
        scan[0] = 2000
        scan[180] = 3000
        app.on_scan(scan)
        grid = grid_of(tester)
        self.assertEqual(grid[10 - 4, 10], 0)  # column 0 is behind the robot
        self.assertEqual(grid[10 + 6, 10], 0)  # column 180 is in front of the robot

    def test_static_obstacle_approaches_when_driving_forward(self):
        # obstacle 5 m ahead of the robot at the origin, robot drives 2 m towards it
        app, tester = make_app({'size': 20, 'resolution': 0.5, 'lidar_range': 8})
        scan = [0] * 360
        scan[0] = 5000
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.on_scan(scan)
        first = np.argwhere(grid_of(tester) == 0)[0]
        scan[0] = 3000
        app.on_pose3d([[2.0, 0, 0], [0, 0, 0, 1]])
        app.on_scan(scan)
        second = np.argwhere(grid_of(tester) == 0)[0]
        self.assertEqual(first[0] - 20, 10)
        self.assertEqual(second[0] - 20, 6)  # closer to the robot (center) than before

    def test_static_obstacle_same_world_cell_for_any_heading(self):
        # world fixed obstacle at (4, 0) seen from (0, 0) heading 0, and from (4, -3) heading 90 deg (ahead) and heading 0 (to the left)
        config = {'size': 20, 'resolution': 0.5, 'lidar_range': 8, 'heading_aligned': False}
        yaw = math.pi / 2
        views = [([[0, 0, 0], [0, 0, 0, 1]], 0, 4000),
                 ([[4.0, -3.0, 0], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]], 0, 3000),
                 ([[4.0, -3.0, 0], [0, 0, 0, 1]], 90, 3000)]
        cells = []
        for pose, column, distance in views:
            app, tester = make_app(config)
            scan = [0] * 360
            scan[column] = distance
            app.on_pose3d(pose)
            app.on_scan(scan)
            i, j = np.argwhere(grid_of(tester) == 0)[0]
            cells.append(((i - 20) * 0.5 + pose[0][0], (j - 20) * 0.5 + pose[0][1]))
        for (x, y) in cells:
            self.assertAlmostEqual(x, 4.0, delta=0.5)
            self.assertAlmostEqual(y, 0.0, delta=0.5)

    def test_arrow_vector(self):
        # plot: horizontal axis = y (reversed axis, left is positive y), vertical axis = x
        dx, dy = arrow_vector(0, 1)
        self.assertAlmostEqual(dx, 0)
        self.assertAlmostEqual(dy, 1)
        dx, dy = arrow_vector(math.pi / 2, 1)  # heading +y: data y positive -> drawn to the left
        self.assertAlmostEqual(dx, 1)
        self.assertAlmostEqual(dy, 0)
        dx, dy = arrow_vector(-math.pi / 2, 1)  # heading -y -> drawn to the right
        self.assertAlmostEqual(dx, -1)


if __name__ == '__main__':
    unittest.main()

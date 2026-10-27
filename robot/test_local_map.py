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
    bus.connect('app.gridmap', 'tester.gridmap')
    return app, tester


def grid_of(tester):
    _, channel, data = tester.queue.get_nowait()
    assert channel == 'gridmap'
    return np.array(data)


class TestLocalMap(unittest.TestCase):
    def test_size(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5})
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(grid.shape, (20, 20))
        self.assertTrue((grid == 0).all())

    def test_obstacle_ahead(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4})
        scan = [0] * 360
        scan[0] = 2000  # straight ahead, 2 m
        app.on_scan(scan)
        grid = grid_of(tester)
        self.assertEqual(grid[10 + 4, 10], 1)
        self.assertEqual(grid[10 + 2, 10], 0)
        self.assertEqual((grid == 1).sum(), 1)
        self.assertEqual(grid[10 + 9, 10], -1)  # beyond the lidar range

    def test_obstacle_left(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4})
        scan = [0] * 360
        scan[90] = 2000
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[10, 14], 1)

    def test_flip_scan(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4, 'flip_scan': True})
        scan = [0] * 360
        scan[90] = 2000
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[10, 6], 1)

    def test_yaw(self):
        scan = [0] * 360
        scan[0] = 2000
        yaw = math.pi / 2
        pose = [[0, 0, 0], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]]

        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4})
        app.on_pose3d(pose)
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[14, 10], 1)  # robot frame

        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4,
                                'heading_aligned': False})
        app.on_pose3d(pose)
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[10, 14], 1)  # world frame

    def test_unknown_cells(self):
        app, tester = make_app({'size': 4, 'resolution': 0.5, 'lidar_range': 1})
        app.on_scan([0] * 4)
        grid = grid_of(tester)
        self.assertEqual(grid[0, 0], -1)  # corner out of reach of the lidar
        self.assertEqual(grid[4, 4], 0)
        self.assertEqual(set(np.unique(grid)), {-1, 0})

    def test_accumulate_shift(self):
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4, 'accumulate': True})
        app.on_scan([0] * 360)
        self.assertTrue(tester.queue.empty())  # no pose yet
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        scan = [0] * 360
        scan[0] = 2000
        app.on_scan(scan)
        self.assertEqual(grid_of(tester)[14, 10], 1)
        app.on_pose3d([[1.0, 0, 0], [0, 0, 0, 1]])
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(grid[12, 10], 0)
        app.on_scan([0] * 360)  # same pose, nothing changes
        grid_of(tester)

    def make_accumulating(self, size=10):
        app, tester = make_app({'size': size, 'resolution': 0.5, 'lidar_range': 4, 'accumulate': True})
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        scan = [0] * 360
        scan[0] = 2000  # obstacle 2 m ahead
        app.on_scan(scan)
        half = size * 2 // 2
        self.assertEqual(grid_of(tester)[half + 4, half], 1)
        return app, tester, half

    def test_obstacle_kept_when_hidden_behind_closer_obstacle(self):
        app, tester, half = self.make_accumulating()
        app.on_scan([1000] * 360)  # wall 1 m around the robot, the old obstacle is not visible
        grid = grid_of(tester)
        self.assertEqual(grid[half + 4, half], 1)
        self.assertEqual(grid[half + 2, half], 1)  # the wall itself

    def test_obstacle_kept_when_out_of_lidar_range(self):
        app, tester, half = self.make_accumulating(size=20)
        app.on_pose3d([[-3.0, 0, 0], [0, 0, 0, 1]])  # the obstacle is 5 m away, the range is 4 m
        app.on_scan([0] * 360)
        self.assertEqual(grid_of(tester)[half + 10, half], 1)
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.on_scan([0] * 360)  # robot returns and sees free space
        self.assertEqual(grid_of(tester)[half + 4, half], 0)

    def test_obstacle_removed_when_lidar_sees_free_space(self):
        app, tester, _ = self.make_accumulating()
        app.on_scan([0] * 360)  # lidar confirms free space in the whole range
        self.assertEqual(grid_of(tester)[14, 10], 0)

    def test_obstacle_removed_when_lidar_sees_farther(self):
        app, tester, _ = self.make_accumulating()
        scan = [0] * 360
        scan[0] = 3500  # the obstacle moved away, the beam passes through the old cell
        app.on_scan(scan)
        grid = grid_of(tester)
        self.assertEqual(grid[14, 10], 0)
        self.assertEqual(grid[17, 10], 1)

    def test_empty_permanent_layer_changes_nothing(self):
        scan = [0] * 360
        scan[0] = 2000
        for accumulate in (False, True):
            results = []
            for add in (False, True):
                app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4,
                                        'accumulate': accumulate})
                app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
                if add:
                    app.add_permanent([], 'test')
                app.on_scan(scan)
                results.append(grid_of(tester))
            self.assertTrue((results[0] == results[1]).all())

    def test_permanent_obstacle_outside_lidar_range(self):
        app, tester = make_app({'size': 20, 'resolution': 0.5, 'lidar_range': 4})
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.on_scan([0] * 360)
        self.assertEqual(grid_of(tester)[20 + 16, 20], -1)  # 8 m ahead, out of the 4 m range
        app.add_permanent([(16, 0)], 'test')
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(grid[20 + 16, 20], 1)
        self.assertEqual((grid == 1).sum(), 1)

    def test_permanent_obstacle_survives_window_shift(self):
        app, tester = make_app({'size': 20, 'resolution': 0.5, 'lidar_range': 4})
        app.add_permanent([(4, 0)], 'test')  # world x = 2 m
        for x, expected in ((0, 1), (-20.0, None), (0, 1)):
            app.on_pose3d([[x, 0, 0], [0, 0, 0, 1]])
            app.on_scan([0] * 360)
            grid = grid_of(tester)
            if expected is None:
                self.assertEqual((grid == 1).sum(), 0)  # 22 m away, outside of the window
            else:
                self.assertEqual(grid[20 + 4, 20], expected)

    def test_permanent_layer_does_not_overwrite_free_space(self):
        app, tester = make_app({'size': 20, 'resolution': 0.5, 'lidar_range': 4})
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.on_scan([0] * 360)
        free = grid_of(tester)
        app.add_permanent([(16, 0)], 'test')
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        changed = np.argwhere(grid != free)
        self.assertEqual(changed.tolist(), [[20 + 16, 20]])  # only the permanent cell, unknown -> obstacle
        self.assertEqual(grid[20 + 2, 20], 0)  # free space from the lidar stays free

    def test_permanent_obstacle_overrides_current_free_cell(self):
        app, tester = make_app({'size': 20, 'resolution': 0.5, 'lidar_range': 4})
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.add_permanent([(4, 0)], 'test')
        app.on_scan([0] * 360)  # the lidar sees free space, the permanent obstacle stays
        self.assertEqual(grid_of(tester)[20 + 4, 20], 1)

    def test_permanent_obstacle_frames(self):
        yaw = math.pi / 2
        pose = [[0, 0, 0], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]]
        for heading_aligned, expected in ((True, (20, 20 - 8)), (False, (20 + 8, 20))):
            app, tester = make_app({'size': 20, 'resolution': 0.5, 'lidar_range': 4,
                                    'heading_aligned': heading_aligned})
            app.on_pose3d(pose)
            app.add_permanent([(8, 0)], 'test')  # world x = 4 m, robot looks along world y
            app.on_scan([0] * 360)
            grid = grid_of(tester)
            self.assertEqual(np.argwhere(grid == 1).tolist(), [list(expected)])

    def make_bumper_app(self, **config):
        cfg = {'size': 20, 'resolution': 0.5, 'lidar_range': 4,
               'bumpers': [[1, 0, 1, 0.9], [1, -0.9, 1, 0]]}  # [left, right]
        cfg.update(config)
        app, tester = make_app(cfg)
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        return app, tester

    def test_bumper_creates_permanent_obstacle(self):
        app, tester = self.make_bumper_app()
        app.on_bumpers([1, 0])
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(sorted(np.argwhere(grid == 1).tolist()), [[22, 20], [22, 21], [22, 22]])  # left only
        self.assertEqual(set(app.permanent.values()), {'bumper'})

    def test_bumper_right_and_both(self):
        app, tester = self.make_bumper_app()
        app.on_bumpers([0, 1])
        app.on_scan([0] * 360)
        self.assertEqual(sorted(np.argwhere(grid_of(tester) == 1).tolist()), [[22, 18], [22, 19], [22, 20]])
        app.on_bumpers([1, 0])
        app.on_scan([0] * 360)
        self.assertEqual(len(np.argwhere(grid_of(tester) == 1)), 5)  # both are kept, [22, 20] is shared

    def test_bumper_released_does_not_remove_obstacle(self):
        app, tester = self.make_bumper_app()
        app.on_bumpers([1, 1])
        app.on_bumpers([0, 0])
        app.on_scan([0] * 360)
        self.assertEqual(len(np.argwhere(grid_of(tester) == 1)), 5)

    def test_bumper_repeated_messages(self):
        app, tester = self.make_bumper_app()
        for _ in range(3):
            app.on_bumpers([1, 0])
        self.assertEqual(len(app.permanent), 3)

    def test_bumper_at_more_positions(self):
        app, tester = self.make_bumper_app()
        app.on_bumpers([1, 0])
        app.on_pose3d([[5.0, 0, 0], [0, 0, 0, 1]])
        app.on_bumpers([1, 0])
        self.assertEqual(len(app.permanent), 6)
        app.on_scan([0] * 360)
        grid = grid_of(tester)
        self.assertEqual(len(np.argwhere(grid == 1)), 6)
        self.assertEqual(grid[20 + 2 - 10, 20], 1)  # the first hit is 5 m behind the robot (+ 1 m forward)

    def test_bumper_message_length_mismatch(self):
        app, tester = self.make_bumper_app()
        with self.assertLogs('robot.local_map', level='WARNING'):
            app.on_bumpers([1, 1, 1])
        app.on_bumpers([1])
        self.assertEqual(len(app.permanent), 0)

    def test_bumper_without_pose_or_config(self):
        app, tester = make_app({'bumpers': [[1, 0, 1, 0.9]]})
        app.on_bumpers([1])  # unknown position
        self.assertEqual(len(app.permanent), 0)
        app, tester = make_app()
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.on_bumpers([])  # no bumpers configured
        self.assertEqual(len(app.permanent), 0)

    def test_bumper_rotated_robot(self):
        yaw = math.pi / 2
        app, tester = self.make_bumper_app()
        app.on_pose3d([[0, 0, 0], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]])
        app.on_bumpers([1, 0])
        # the robot looks along world y, its left is world -x
        self.assertEqual(set(app.permanent), {(0, 2), (-1, 2), (-2, 2)})

    def test_bumper_in_robot_and_world_frame(self):
        yaw = math.pi / 2
        pose = [[0, 0, 0], [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]]
        for heading_aligned, expected in ((True, [[22, 20], [22, 21], [22, 22]]),
                                          (False, [[20, 22], [19, 22], [18, 22]])):
            app, tester = self.make_bumper_app(heading_aligned=heading_aligned)
            app.on_pose3d(pose)
            app.on_bumpers([1, 0])
            app.on_scan([0] * 360)
            self.assertEqual(sorted(np.argwhere(grid_of(tester) == 1).tolist()), sorted(expected))

    def test_angle_offset(self):
        # Ouster column 0 points backwards, angle_offset 180 turns it into the rear of the robot
        app, tester = make_app({'size': 10, 'resolution': 0.5, 'lidar_range': 4, 'angle_offset': 180})
        scan = [0] * 360
        scan[0] = 2000
        scan[180] = 3000
        app.on_scan(scan)
        grid = grid_of(tester)
        self.assertEqual(grid[10 - 4, 10], 1)  # column 0 is behind the robot
        self.assertEqual(grid[10 + 6, 10], 1)  # column 180 is in front of the robot

    def test_static_obstacle_approaches_when_driving_forward(self):
        # obstacle 5 m ahead of the robot at the origin, robot drives 2 m towards it
        app, tester = make_app({'size': 20, 'resolution': 0.5, 'lidar_range': 8})
        scan = [0] * 360
        scan[0] = 5000
        app.on_pose3d([[0, 0, 0], [0, 0, 0, 1]])
        app.on_scan(scan)
        first = np.argwhere(grid_of(tester) == 1)[0]
        scan[0] = 3000
        app.on_pose3d([[2.0, 0, 0], [0, 0, 0, 1]])
        app.on_scan(scan)
        second = np.argwhere(grid_of(tester) == 1)[0]
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
            i, j = np.argwhere(grid_of(tester) == 1)[0]
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

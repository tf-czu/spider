import unittest
from unittest.mock import MagicMock

import numpy as np

from osgar.bus import Bus

from robot.scan3d2scan import Scan3DToScan2D


def make_app(config=None):
    """Create a Scan3DToScan2D instance with a mock bus."""
    bus = Bus(MagicMock())
    app = Scan3DToScan2D(config=config or {}, bus=bus.handle('app'))
    tester = bus.handle('tester')
    bus.connect('app.scan2d', 'tester.scan2d')
    return app, tester


class TestScan3DToScan2D(unittest.TestCase):
    def _get_result(self, tester):
        """Read the published scan2d result from the tester queue."""
        timestamp, channel, data = tester.queue.get_nowait()
        self.assertEqual(channel, 'scan2d')
        return data

    def test_output_length(self):
        app, tester = make_app()
        data = np.full((32, 1024), 2000, dtype=np.float64)
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertEqual(len(result), 1024)

    def test_uses_bottom_rays(self):
        app, tester = make_app()
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        # top rows (0-15) have valid distances but should be ignored
        data[0:16, :] = 2000
        # bottom rows (16-31) have 0 -> invalid (below min_dist)
        data[16:32, :] = 0
        app.on_scan3d(data)
        result = self._get_result(tester)
        # all columns should be 0.0 because bottom rays are invalid
        self.assertTrue(all(v == 0.0 for v in result))

    def test_minimum_per_column(self):
        app, tester = make_app()
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        # bottom ray 0 (row 16) -> 2000 mm, bottom ray 1 (row 17) -> 5000 mm
        data[16, :] = 2000
        data[17, :] = 5000
        app.on_scan3d(data)
        result = self._get_result(tester)
        # minimum valid distance per column should be 2000
        self.assertTrue(all(v == 2000.0 for v in result))

    def test_zero_when_no_valid(self):
        app, tester = make_app()
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        # all bottom rays are 0 -> below min_dist -> no valid
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertTrue(all(v == 0.0 for v in result))

    def test_cos_projection_slant_distance(self):
        app, tester = make_app()
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        # bottom ray 15 (row 31, phi=22.5 deg) -> 1500 mm (valid: max_range ~1587 mm)
        data[31, :] = 1500
        app.on_scan3d(data)
        result = self._get_result(tester)
        # projected distance = 1500 * cos(22.5 deg)
        expected = 1500 * np.cos(np.radians(22.5))
        self.assertTrue(all(abs(v - expected) < 0.01 for v in result))

    def test_clip_above_lidar_range(self):
        # small lidar_range so that the projected value exceeds it
        app, tester = make_app(config={'slope': 5, 'lidar_range': 2.0})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        # row 17 (phi=1.5 deg), distance 3000 mm -> projection ~2999.7 mm > lidar_range=2000 mm
        data[17, :] = 3000
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertTrue(all(v == 0.0 for v in result))

    def test_wrong_shape_raises(self):
        app, tester = make_app()
        data = np.zeros((16, 1024), dtype=np.float64)
        with self.assertRaises(AssertionError):
            app.on_scan3d(data)


if __name__ == '__main__':
    unittest.main()
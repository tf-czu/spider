import warnings
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
    bus.connect('app.scan', 'tester.scan')
    return app, tester


class TestScan3DToScan2D(unittest.TestCase):
    def _get_result(self, tester):
        """Read the published scan result from the tester queue."""
        timestamp, channel, data = tester.queue.get_nowait()
        self.assertEqual(channel, 'scan')
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

    def test_slope_zero_no_warning(self):
        # slope=0, phi=0 -> denominator=0 -> max_range should be inf, no warning
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)  # osgar Node setDaemon
            warnings.simplefilter("error", RuntimeWarning)       # catch divide-by-zero
            app, tester = make_app(config={'slope': 0})
        # first ray (phi=0) should have max_range = inf
        self.assertTrue(np.isinf(app.max_range[0]))
        # other rays should have finite max_range
        self.assertTrue(np.all(np.isfinite(app.max_range[1:])))

    def test_flip_scan_single(self):
        # flip_scan=True, cluster_size=1 -> scan flipped exactly once
        app, tester = make_app(config={'flip_scan': True})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        data[16, 512] = 2000  # obstacle at front (index 512)
        app.on_scan3d(data)
        result = self._get_result(tester)
        # after flip, obstacle should be at index 1024-1-512 = 511
        self.assertEqual(result[511], 2000.0)
        self.assertEqual(result[512], 0.0)

    def test_flip_scan_cluster(self):
        # flip_scan=True, cluster_size=2 -> scan flipped exactly once
        app, tester = make_app(config={'flip_scan': True, 'cluster_size': 2})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        data[16, 512] = 2000  # obstacle at front (index 512)
        data[16, 513] = 2000  # same block (512,513) -> valid cluster
        app.on_scan3d(data)
        result = self._get_result(tester)
        # cluster_size=2 -> output length 512; obstacle at 512 -> block 256
        # after flip -> index 512-1-256 = 255
        self.assertEqual(len(result), 512)
        self.assertEqual(result[255], 2000.0)

    def test_cluster_size_1_no_change(self):
        # cluster_size=1 -> output unchanged (length 1024)
        app, tester = make_app(config={'cluster_size': 1})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        data[16, :] = 2000
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertEqual(len(result), 1024)
        self.assertTrue(all(v == 2000.0 for v in result))

    def test_cluster_size_2_downsample(self):
        # cluster_size=2 -> output length 512
        app, tester = make_app(config={'cluster_size': 2})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        data[16, :] = 2000
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertEqual(len(result), 512)

    def test_cluster_keeps_valid_block(self):
        # block with similar values -> kept (min)
        app, tester = make_app(config={'cluster_size': 2})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        data[16, 0] = 2000
        data[16, 1] = 2010  # diff 10 < 2% of 2010 = 40.2 -> valid
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertEqual(result[0], 2000.0)

    def test_cluster_rejects_noisy_block(self):
        # block with large difference -> 0.0
        app, tester = make_app(config={'cluster_size': 2})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        data[16, 0] = 2000
        data[16, 1] = 3000  # diff 1000 > 2% of 3000 = 60 -> invalid
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertEqual(result[0], 0.0)

    def test_cluster_rejects_block_with_zero(self):
        # block containing 0 -> 0.0
        app, tester = make_app(config={'cluster_size': 2})
        data = np.full((32, 1024), 0.0, dtype=np.float64)
        data[16, 0] = 2000
        data[16, 1] = 0  # zero -> invalid
        app.on_scan3d(data)
        result = self._get_result(tester)
        self.assertEqual(result[0], 0.0)

    def test_wrong_shape_raises(self):
        app, tester = make_app()
        data = np.zeros((16, 1024), dtype=np.float64)
        with self.assertRaises(AssertionError):
            app.on_scan3d(data)


if __name__ == '__main__':
    unittest.main()

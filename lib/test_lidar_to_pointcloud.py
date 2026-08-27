# -*- coding: UTF-8 -*-

import unittest
import os
import sys

THIS_DIR = os.path.dirname(__file__)
PROJECT_DIR = os.path.dirname(THIS_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from unittest.mock import MagicMock, patch
import numpy as np

try:
    import ouster.sdk
except ModuleNotFoundError:
    import types
    ouster_module = types.ModuleType("ouster")
    sdk_module = types.ModuleType("ouster.sdk")
    core_module = types.SimpleNamespace(
        SensorInfo=object,
        XYZLut=object,
        LidarScan=object,
        ChanField=types.SimpleNamespace(RANGE="RANGE"),
    )
    sdk_module.core = core_module
    ouster_module.sdk = sdk_module
    sys.modules["ouster"] = ouster_module
    sys.modules["ouster.sdk"] = sdk_module

from lib.lidar_to_pointcloud import *


class TestLidarToPointCloud(unittest.TestCase):

    def test_convert__returns_only_points_with_nonzero_range(self):
        """
        Test that conversion keeps only XYZ points whose corresponding range
        value is nonzero.
        """
        with patch("lib.lidar_to_pointcloud.ous.core.SensorInfo") as sensor_info_mock, \
             patch("lib.lidar_to_pointcloud.ous.core.XYZLut") as xyz_lut_mock, \
             patch("lib.lidar_to_pointcloud.ous.core.LidarScan") as lidar_scan_mock:

            sensor_info_mock.return_value = MagicMock()

            lidar_scan = MagicMock()
            range_field = np.zeros((2, 2), dtype=int)
            lidar_scan.field.return_value = range_field
            lidar_scan_mock.return_value = lidar_scan

            xyz_lut = MagicMock()
            xyz_lut.return_value = np.array([
                [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                [[3.0, 0.0, 0.0], [4.0, 0.0, 0.0]],
            ])
            xyz_lut_mock.return_value = xyz_lut

            converter = LidarToPointCloud("{}")
            scan = np.array([
                [10, 0],
                [20, 0],
            ])

            points = converter.convert(scan)

            self.assertEqual(points.shape, (2, 3))
            self.assertTrue(np.allclose(points, np.array([
                [1.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ])))
            lidar_scan_mock.assert_called_once_with(2, 2)
            self.assertTrue(np.array_equal(range_field, scan))

    def test_convert__rejects_non_2d_scan(self):
        """
        Test that conversion rejects inputs that are not 2D lidar range scans.
        """
        with patch("lib.lidar_to_pointcloud.ous.core.SensorInfo"), \
             patch("lib.lidar_to_pointcloud.ous.core.XYZLut"):

            converter = LidarToPointCloud("{}")

            with self.assertRaises(ValueError):
                converter.convert(np.zeros((2, 2, 2)))


if __name__ == '__main__':
    unittest.main()

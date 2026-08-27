# -*- coding: UTF-8 -*-

import os
import sys
import types
import unittest
from unittest.mock import MagicMock, patch
from datetime import timedelta
import numpy as np

PROJECT_DIR = os.path.dirname(__file__)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

try:
    from osgar.bus import Bus
except ModuleNotFoundError:
    osgar_module = types.ModuleType("osgar")
    node_module = types.ModuleType("osgar.node")
    bus_module = types.ModuleType("osgar.bus")

    class Node:
        def __init__(self, config, bus):
            self.config = config
            self.bus = bus
            self.time = None
            self.verbose = False

        def publish(self, channel, data):
            pass

    class Bus:
        def __init__(self, logger):
            self.logger = logger

        def handle(self, name):
            return MagicMock()

    node_module.Node = Node
    bus_module.Bus = Bus
    osgar_module.node = node_module
    osgar_module.bus = bus_module
    sys.modules["osgar"] = osgar_module
    sys.modules["osgar.node"] = node_module
    sys.modules["osgar.bus"] = bus_module

    from osgar.bus import Bus

try:
    import ouster.sdk
except ModuleNotFoundError:
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

from mapper import *


class TestMapper(unittest.TestCase):

    def test_on_lidar_metadata__initializes_converter_once(self):
        """
        Test that lidar metadata initializes the lidar-to-point-cloud converter
        only once.
        """
        bus = Bus(MagicMock())
        mapper = Mapper({}, bus.handle('abc'))

        with patch('mapper.LidarToPointCloud') as l2pc_class:
            mapper.on_lidar_metadata('metadata 1')
            mapper.on_lidar_metadata('metadata 2')

            l2pc_class.assert_called_once_with('metadata 1')
            self.assertIsNotNone(mapper.l2pc)

    def test_on_lidar_scan3d__ignored_before_metadata(self):
        """
        Test that lidar scans are ignored until lidar metadata has initialized
        the converter.
        """
        bus = Bus(MagicMock())
        mapper = Mapper({}, bus.handle('abc'))

        mapper.interpreter = MagicMock()
        mapper.on_lidar_scan3d(np.zeros((2, 2)))

        mapper.interpreter.update.assert_not_called()

    def test_on_lidar_scan3d__converts_scan_and_updates_interpreter(self):
        """
        Test that a lidar scan is converted to a point cloud and passed to the
        point cloud interpreter.
        """
        bus = Bus(MagicMock())
        mapper = Mapper({}, bus.handle('abc'))
        mapper.time = timedelta(seconds=1)

        points = np.array([[1.0, 0.0, 0.0]])
        mapper.l2pc = MagicMock()
        mapper.l2pc.convert.return_value = points
        mapper.interpreter = MagicMock()
        mapper.interpreter.update.return_value = None

        scan = np.zeros((2, 2))
        mapper.on_lidar_scan3d(scan)

        mapper.l2pc.convert.assert_called_once_with(scan)
        mapper.interpreter.update.assert_called_once_with(mapper.time, points)

    def test_on_lidar_scan3d__stores_draw_data_in_verbose_mode(self):
        """
        Test that generated maps are stored for drawing when verbose mode is
        enabled.
        """
        bus = Bus(MagicMock())
        mapper = Mapper({}, bus.handle('abc'))
        mapper.verbose = True
        mapper.time = timedelta(seconds=1)

        points = np.array([[1.0, 0.0, 0.0]])
        mapper.l2pc = MagicMock()
        mapper.l2pc.convert.return_value = points

        output = {
            'timestamp': mapper.time,
            'min_map': np.zeros((2, 2)),
            'max_map': np.ones((2, 2)),
            'dif_map': np.ones((2, 2)) * 0.5,
            'slope_map': np.ones((2, 2)) * 0.1,
            'angular_distances': np.array([1.0, np.nan, 2.0, np.nan]),
            'angular_counts': np.array([1, 0, 1, 0]),
        }
        mapper.interpreter = MagicMock()
        mapper.interpreter.update.return_value = output

        mapper.on_lidar_scan3d(np.zeros((2, 2)))

        self.assertEqual(mapper.draw_timestamps, [mapper.time])
        self.assertEqual(len(mapper.draw_dif_maps), 1)
        self.assertEqual(len(mapper.draw_angular_distances), 1)
        self.assertTrue(np.array_equal(mapper.draw_dif_maps[0], output['dif_map']))
        self.assertTrue(np.array_equal(mapper.draw_angular_distances[0], output['angular_distances'], equal_nan=True))

    def test_on_lidar_reflectivity__does_not_crash(self):
        """
        Test that the reflectivity callback accepts reflectivity scans without
        raising an exception.
        """
        bus = Bus(MagicMock())
        mapper = Mapper({}, bus.handle('abc'))
        mapper.l2pc = MagicMock()

        mapper.on_lidar_reflectivity(np.zeros((2, 2)))


if __name__ == '__main__':
    unittest.main()

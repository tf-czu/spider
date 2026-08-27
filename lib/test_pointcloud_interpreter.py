# -*- coding: UTF-8 -*-

import unittest
import os
import sys

THIS_DIR = os.path.dirname(__file__)
PROJECT_DIR = os.path.dirname(THIS_DIR)
if PROJECT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_DIR)

from datetime import timedelta
import numpy as np

from lib.pointcloud_interpreter import *


class TestPointCloudInterpreter(unittest.TestCase):

    def test_update__returns_none_before_output_period(self):
        """
        Test that update only accumulates point clouds and returns None before
        the configured output period elapses.
        """
        interpreter = PointCloudInterpreter(output_frequency=1.0)
        points = np.array([[1.0, 0.0, 0.0]])

        output = interpreter.update(timedelta(seconds=0.0), points)
        self.assertIsNone(output)

        output = interpreter.update(timedelta(seconds=0.5), points)
        self.assertIsNone(output)

        self.assertEqual(len(interpreter.list_of_accumulated_point_clouds), 2)

    def test_update__returns_output_after_output_period(self):
        """
        Test that update processes accumulated point clouds and returns map
        outputs once the configured output period elapses.
        """
        interpreter = PointCloudInterpreter(output_frequency=1.0)
        points_1 = np.array([[1.0, 0.0, 0.0]])
        points_2 = np.array([[2.0, 0.0, 0.0]])

        self.assertIsNone(interpreter.update(timedelta(seconds=0.0), points_1))
        output = interpreter.update(timedelta(seconds=1.0), points_2)

        self.assertIsNotNone(output)
        self.assertIn("timestamp", output)
        self.assertIn("min_map", output)
        self.assertIn("max_map", output)
        self.assertIn("dif_map", output)
        self.assertIn("slope_map", output)
        self.assertIn("angular_distances", output)
        self.assertEqual(output["timestamp"], timedelta(seconds=1.0))
        self.assertEqual(len(interpreter.list_of_accumulated_point_clouds), 0)
        self.assertEqual(len(interpreter.fifo_of_outputs), 1)

    def test_process_accumulated_point_clouds__empty_buffer(self):
        """
        Test that processing an empty accumulation buffer returns None and does
        not create a new output.
        """
        interpreter = PointCloudInterpreter(output_frequency=1.0)

        output = interpreter.process_accumulated_point_clouds(timedelta(seconds=1.0))

        self.assertIsNone(output)

    def test_fifo_of_outputs__keeps_only_remembered_outputs(self):
        """
        Test that the output FIFO keeps only the configured number of most
        recent map outputs.
        """
        interpreter = PointCloudInterpreter(output_frequency=1.0)
        interpreter.remembered_outputs = 2
        points = np.array([[1.0, 0.0, 0.0]])

        for i in range(4):
            interpreter.list_of_accumulated_point_clouds.append(points)
            interpreter.process_accumulated_point_clouds(timedelta(seconds=i))

        self.assertEqual(len(interpreter.fifo_of_outputs), 2)
        self.assertEqual(interpreter.fifo_of_outputs[0]["timestamp"], timedelta(seconds=2))
        self.assertEqual(interpreter.fifo_of_outputs[1]["timestamp"], timedelta(seconds=3))


if __name__ == '__main__':
    unittest.main()

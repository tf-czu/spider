# -*- coding: UTF-8 -*-

import unittest
import math
from datetime import timedelta

from rmsd import *

class TestRootMeanSquareDeviationCounter(unittest.TestCase):

    def test_interpolate_xyz_in_time__normal_run(self):
        xyz_1 = [2, 1, 0]
        xyz_2 = [3, 6, 7]
        for k in range(11):
            xyz = interpolate_xyz_in_time(timedelta(seconds = 5), xyz_1, timedelta(seconds = 6), xyz_2, timedelta(seconds = 5 + k/10))
            alpha = k / 10
            expected_xyz = [(1 - alpha)*xyz_1[i] + alpha*xyz_2[i] for i in range(len(xyz_1))]
            for i in range(len(xyz_1)):
                self.assertAlmostEqual(xyz[i], expected_xyz[i])

    def test_interpolate_xyz_in_time__error_run(self):
        xyz_1 = [2, 1, 0]
        xyz_2 = [3, 6, 7]
        with self.assertRaises(AssertionError):
            interpolate_xyz_in_time(timedelta(seconds = 6), xyz_1, timedelta(seconds = 5), xyz_2, timedelta(seconds = 5.5))
        with self.assertRaises(AssertionError):
            interpolate_xyz_in_time(timedelta(seconds = 5), xyz_1, timedelta(seconds = 6), xyz_2, timedelta(seconds = 6.5))

    def test_1(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(10):
            rmsd_counter.add_X(timedelta(seconds = t), [t, t, 0])
            rmsd_counter.add_Y(timedelta(seconds = t), [t+1, t, 0])
        print(rmsd_counter.get_rmsd())




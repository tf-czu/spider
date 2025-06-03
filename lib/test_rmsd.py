# -*- coding: UTF-8 -*-

import unittest
import math
from datetime import timedelta

from rmsd import *

class TestRootMeanSquareDeviationCounter(unittest.TestCase):

    def test_compute_rmsd__parallel_lines__without_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(10):
            rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            rmsd_counter.add_B(timedelta(seconds = t), [t+2, t, 0])
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), 2)

    def test_compute_rmsd__parallel_lines__with_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(11):
            if t % 2 == 0:
                rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            else:
                rmsd_counter.add_B(timedelta(seconds = t), [t+2, t, 0])
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), 2)

    def test_compute_rmsd__non_parallel_lines__without_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(10):
            rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            rmsd_counter.add_B(timedelta(seconds = t), [t, 0, 0])
        expected_rmsd = math.sqrt(sum([i**2 for i in range(10)]) / 10)
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), expected_rmsd)

    def test_compute_rmsd__non_parallel_lines__with_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(11):
            if t % 2 == 0:
                rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            else:
                rmsd_counter.add_B(timedelta(seconds = t), [t, 0, 0])
        expected_rmsd = math.sqrt(sum([i**2 for i in range(2, 10, 2)]) / 4)
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), expected_rmsd)

    def test_compute_rmsd__two_coords__without_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(10):
            rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            rmsd_counter.add_B(timedelta(seconds = t), [t+1, t+1, 0])
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), math.sqrt(2))

    def test_compute_rmsd__two_coords__with_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(11):
            if t % 2 == 0:
                rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            else:
                rmsd_counter.add_B(timedelta(seconds = t), [t+1, t+1, 0])
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), math.sqrt(2))




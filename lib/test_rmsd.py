# -*- coding: UTF-8 -*-

import unittest
import math
from datetime import timedelta

from rmsd import *

class TestRootMeanSquareDeviationCounter(unittest.TestCase):

    def test_compute_rmsd__without_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(10):
            rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            rmsd_counter.add_B(timedelta(seconds = t), [t+2, t, 0])
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), 2)

    def test_compute_rmsd__with_interpolation(self):
        rmsd_counter = RootMeanSquareDeviationCounter()
        for t in range(11):
            if t % 2 == 0:
                rmsd_counter.add_A(timedelta(seconds = t), [t, t, 0])
            else:
                rmsd_counter.add_B(timedelta(seconds = t), [t+2, t, 0])
        self.assertAlmostEqual(rmsd_counter.compute_rmsd(), 2)




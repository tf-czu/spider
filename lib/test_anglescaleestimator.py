# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for anglescaleestimator.py
"""

import unittest
import datetime
import random
import math
import numpy as np
from lib.anglescaleestimator import AngleScaleEstimator

class TestAngleScaleEstimator(unittest.TestCase):

    def test_distance_computation(self):
        ase = AngleScaleEstimator()
        self.assertEqual(ase.get_distance_from_origin([3,4]), 5)

    def test_weight_by_distance(self):
        ase = AngleScaleEstimator()
        d = (ase.distance_with_full_weight + ase.minimal_distance_to_accept)/2
        self.assertEqual(ase.weight_by_distance(d),1/2)


    def test_calculate_angle(self):
        ase = AngleScaleEstimator()
        c1 = np.complex128(1+1*1j)
        c2 = np.complex128(0+1*1j)
        self.assertAlmostEqual(np.angle(c2/c1), np.pi/4)

    def test_update(self):
        ase = AngleScaleEstimator()
        gps = [100,100]
        imu = [100,0]
        ase.update(gps, imu)
        ase.update(gps, imu)
        gps = [0,100]
        imu = [100,0]
        ase.update(gps, imu)
        self.assertEqual(ase.number_of_measurements, 3)
        self.assertAlmostEqual(ase.get_angle(), np.arctan((math.sqrt(2)+1)/math.sqrt(2)))
        self.assertAlmostEqual(ase.get_scale(), (2*math.sqrt(2)+1)/3)






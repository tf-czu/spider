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

    def test_distance_computation_2(self):
        ase = AngleScaleEstimator()
        ase.set_origin([1,1])
        self.assertEqual(ase.get_distance_from_origin([4,5]), 5)

    def test_weight_by_distance(self):
        ase = AngleScaleEstimator()
        d = (ase.distance_with_full_weight + ase.minimal_distance_to_accept)/2
        self.assertEqual(ase.weight_by_distance(d),1/2)


    def test_calculate_angle(self):
        ase = AngleScaleEstimator()
        p1 = [1,1]
        p2 = [0,1]
        #self.assertAlmostEqual(np.angle(c2/c1), np.pi/4)
        self.assertAlmostEqual(ase.calculate_angle(p2,p1), np.pi/4)

    def test_calculate_angle_2(self):
        ase = AngleScaleEstimator()
        ase.set_origin([1,1])
        p1 = [2,2]
        p2 = [0,2]
        self.assertAlmostEqual(ase.calculate_angle_wrt_origin(p2,p1), np.pi/2)

    def test_rotate_and_scale(self):
        ase = AngleScaleEstimator()
        ase.set_origin([1,1])
        ase.angle = np.pi/2
        ase.scale = 1
        point = [2,1]
        rotated_point = ase.rotate_and_scale(point)
        self.assertAlmostEqual(rotated_point[0],1)
        self.assertAlmostEqual(rotated_point[1],2)

    def test_update(self):
        ase = AngleScaleEstimator()
        gps = [100,100]
        imu = [100,0]
        ase.update(gps, imu)
        ase.update(gps, imu)
        gps = [0,100]
        imu = [100,0]
        ase.update(gps, imu)
        self.assertAlmostEqual(ase.get_angle(), np.arctan((math.sqrt(2)+1)/math.sqrt(2)))
        self.assertAlmostEqual(ase.get_scale(), (2*math.sqrt(2)+1)/3)

    def test_update_2(self):
        ase = AngleScaleEstimator()
        ase.set_origin([100,100])
        gps = [200,200]
        imu = [200,100]
        ase.update(gps, imu)
        ase.update(gps, imu)
        gps = [100,200]
        imu = [200,100]
        ase.update(gps, imu)
        self.assertAlmostEqual(ase.get_angle(), np.arctan((math.sqrt(2)+1)/math.sqrt(2)))
        self.assertAlmostEqual(ase.get_scale(), (2*math.sqrt(2)+1)/3)





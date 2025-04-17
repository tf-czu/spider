# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for anglescaleestimator.py
"""

import unittest
import datetime
import random
import math
import numpy as np
from lib.kalman_1d import KalmanFilter1D

class TestKalman1D(unittest.TestCase):

    def test_kalman(self):
        number_of_steps = 100
        speed = 2
        kf = KalmanFilter1D()
        self.assertEqual(kf.get_last_x()[1], 0.0)
        for i in range(1,number_of_steps+1):
            kf.input(speed*i, 1*i, 0.9)
        self.assertEqual(kf.get_last_x()[0], number_of_steps)
        self.assertTrue(abs(kf.get_last_x()[1][0]- number_of_steps*speed) < 0.01)
        self.assertTrue(abs(kf.get_velocity_estimate(kf.get_last_x()[0]) - speed) < 0.01)


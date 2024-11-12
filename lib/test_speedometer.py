# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for speedometer.py
"""

import unittest
import datetime
import random
import math
from lib.speedometer import Speedometer

class TestSpeedometer(unittest.TestCase):

    def test_object_creation(self):
        t = datetime.timedelta(seconds = 0)
        dt = datetime.timedelta(seconds = 1)
        ds = 2
        speedometer = Speedometer()
        for i in range(10):
            speedometer.update(t, ds)
            t += dt
        self.assertAlmostEqual(speedometer.get_distance_travelled(), 20.0)
        self.assertAlmostEqual(speedometer.get_speed(), 2.0)


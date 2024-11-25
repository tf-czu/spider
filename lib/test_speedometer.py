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

    def test_is_moving_1(self):
        # robot stoji a pak se rozjede
        speedometer = Speedometer()
        path = [( 0, 0.000, None ),
                ( 1, 0.000, None ),
                ( 2, 0.000, False),
                ( 3, 0.000, False),
                ( 4, 0.000, False),
                ( 5, 0.001, False),
                ( 6, 0.000, False),
                ( 7, 0.001, False),
                ( 8, 0.002, False),
                ( 9, 0.000, False),
                (10, 0.001, False),
                (11, 0.005, False),
                (12, 0.010, True ),
                (13, 0.015, True ),
                (14, 0.020, True ),
                (15, 0.025, True ),
                (16, 0.030, True ),
                (17, 0.035, True ),
                (18, 0.040, True ),
                (19, 0.050, True ),
                (20, 0.060, True )]
        for seconds, distance, expected in path:
            speedometer.update(datetime.timedelta(seconds = seconds), distance)
            self.assertEqual(speedometer.is_moving(), expected)

    def test_is_moving_2(self):
        # robot stoji, rozjede se a zastavi
        speedometer = Speedometer()
        path = [(0.0, 0.0000, None ),
                (0.1, 0.0000, None ),
                (0.2, 0.0000, None ),
                (0.3, 0.0000, None ),
                (0.4, 0.0000, None ),
                (0.5, 0.0001, None ),
                (0.6, 0.0000, None ),
                (0.7, 0.0001, None ),
                (0.8, 0.0002, None ),
                (0.9, 0.0000, None ),
                (1.0, 0.0001, False),
                (1.1, 0.0005, False),
                (1.2, 0.0010, False),
                (1.3, 0.0015, False),
                (1.4, 0.0020, False),
                (1.5, 0.0025, False),
                (1.6, 0.0030, True ),
                (1.7, 0.0035, True ),
                (1.8, 0.0030, True ),
                (1.9, 0.0040, True ),
                (2.0, 0.0030, True ),
                (2.1, 0.0025, True ),
                (2.2, 0.0010, True ),
                (2.3, 0.0015, True ),
                (2.4, 0.0005, True ),
                (2.5, 0.0002, True ),
                (2.6, 0.0000, True ),
                (2.7, 0.0005, True ),
                (2.8, 0.0001, True ),
                (2.9, 0.0000, False),
                (3.0, 0.0000, False),
                (3.1, 0.0025, False),
                (3.2, 0.0010, False),
                (3.3, 0.0015, False),
                (3.4, 0.0005, False),
                (3.5, 0.0002, False),
                (3.6, 0.0000, False),
                (3.7, 0.0005, False),
                (3.8, 0.0001, False),
                (3.9, 0.0000, False),
                (4.0, 0.0000, False)]
        for seconds, distance, expected in path:
            speedometer.update(datetime.timedelta(seconds = seconds), distance)
            #print(speedometer.is_moving())
            self.assertEqual(speedometer.is_moving(), expected)

if __name__ == '__main__':
    unittest.main()


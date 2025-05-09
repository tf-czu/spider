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
        speedometer = Speedometer(period = 0.5, threshold = 0.001)
        path = [( 0, 0.0000, None),
                ( 1, 0.0000, None),
                ( 2, 0.0000, False),
                ( 3, 0.0000, False),
                ( 4, 0.0000, False),
                ( 5, 0.0001, False),
                ( 6, 0.0000, False),
                ( 7, 0.0001, False),
                ( 8, 0.0002, False),
                ( 9, 0.0000, False),
                (10, 0.0001, False),
                (11, 0.0005, False),
                (12, 0.0010, True),
                (13, 0.0015, True),
                (14, 0.0020, True),
                (15, 0.0025, True),
                (16, 0.0030, True),
                (17, 0.0035, True),
                (18, 0.0040, True),
                (19, 0.0050, True),
                (20, 0.0060, True)]
        for seconds, distance, expected in path:
            speedometer.update(datetime.timedelta(seconds = seconds), distance)
            self.assertEqual(speedometer.is_moving(), expected)

    def test_is_moving_2(self):
        # robot stoji, rozjede se a zastavi
        speedometer = Speedometer(period = 0.5, threshold = 0.001)
        path = [(0.0, 0.0000, None),
                (0.1, 0.0000, None),
                (0.2, 0.0000, None),
                (0.3, 0.0000, None),
                (0.4, 0.0000, None),
                (0.5, 0.0001, False),
                (0.6, 0.0000, False),
                (0.7, 0.0001, False),
                (0.8, 0.0002, False),
                (0.9, 0.0000, False),
                (1.0, 0.0001, False),
                (1.1, 0.0005, True),
                (1.2, 0.0010, True),
                (1.3, 0.0015, True),
                (1.4, 0.0020, True),
                (1.5, 0.0025, True),
                (1.6, 0.0030, True),
                (1.7, 0.0035, True),
                (1.8, 0.0030, True),
                (1.9, 0.0040, True),
                (2.0, 0.0030, True),
                (2.1, 0.0025, True),
                (2.2, 0.0010, True),
                (2.3, 0.0015, True),
                (2.4, 0.0005, True),
                (2.5, 0.0002, True),
                (2.6, 0.0000, True),
                (2.7, 0.0005, True),
                (2.8, 0.0001, True),
                (2.9, 0.0000, True),
                (3.0, 0.0000, True),
                (3.1, 0.0025, True),
                (3.2, 0.0010, True),
                (3.3, 0.0015, True),
                (3.4, 0.0005, True),
                (3.5, 0.0002, True),
                (3.6, 0.0000, True),
                (3.7, 0.0005, True),
                (3.8, 0.0001, True),
                (3.9, 0.0000, True),
                (4.0, 0.0000, True)]
        for seconds, distance, expected in path:
            speedometer.update(datetime.timedelta(seconds = seconds), distance)
            self.assertEqual(speedometer.is_moving(), expected)

    def test_is_moving_3(self):
        # testovani metoda compute_average_velocity_over_last_period()
        # robot stoji, rozjede se a zastavi
        path = [(0.0, 0.0000, None,   None),
                (0.1, 0.0000, None,   None),
                (0.2, 0.0000, None,   None),
                (0.3, 0.0000, None,   None),
                (0.4, 0.0000, None,   None),
                (0.5, 0.0001, 0.0002, False),
                (0.6, 0.0000, 0.0002, False),
                (0.7, 0.0001, 0.0004, False),
                (0.8, 0.0002, 0.0008, False),
                (0.9, 0.0000, 0.0008, False),
                (1.0, 0.0001, 0.0008, False),
                (1.1, 0.0005, 0.0018, True),
                (1.2, 0.0010, 0.0036, True),
                (1.3, 0.0015, 0.0062, True),
                (1.4, 0.0020, 0.0102, True),
                (1.5, 0.0025, 0.0150, True),
                (1.6, 0.0030, 0.0200, True),
                (1.7, 0.0035, 0.0250, True),
                (1.8, 0.0030, 0.0280, True),
                (1.9, 0.0040, 0.0320, True),
                (2.0, 0.0030, 0.0330, True),
                (2.1, 0.0025, 0.0320, True),
                (2.2, 0.0010, 0.0270, True),
                (2.3, 0.0015, 0.0240, True),
                (2.4, 0.0005, 0.0170, True),
                (2.5, 0.0002, 0.0114, True),
                (2.6, 0.0000, 0.0064, True),
                (2.7, 0.0005, 0.0054, True),
                (2.8, 0.0001, 0.0026, True),
                (2.9, 0.0000, 0.0016, True),
                (3.0, 0.0000, 0.0012, True),
                (3.1, 0.0025, 0.0062, True),
                (3.2, 0.0010, 0.0072, True),
                (3.3, 0.0015, 0.0100, True),
                (3.4, 0.0005, 0.0110, True),
                (3.5, 0.0002, 0.0114, True),
                (3.6, 0.0000, 0.0064, True),
                (3.7, 0.0005, 0.0054, True),
                (3.8, 0.0001, 0.0026, True),
                (3.9, 0.0000, 0.0016, True),
                (4.0, 0.0000, 0.0012, True),
                (4.1, 0.0005, 0.0022, True),
                (4.2, 0.0000, 0.0012, True),
                (4.3, 0.0005, 0.0020, True),
                (4.4, 0.0005, 0.0030, True),
                (4.5, 0.0002, 0.0034, True),
                (4.6, 0.0000, 0.0024, True),
                (4.7, 0.0005, 0.0034, True),
                (4.8, 0.0001, 0.0026, True),
                (4.9, 0.0000, 0.0016, True),
                (5.0, 0.0000, 0.0012, True),
                (5.1, 0.0000, 0.0012, True),
                (5.2, 0.0000, 0.0002, False),
                (5.3, 0.0000, 0.0000, False),
                (5.4, 0.0000, 0.0000, False),
                (5.5, 0.0000, 0.0000, False),
                (5.6, 0.0000, 0.0000, False),
                (5.7, 0.0000, 0.0000, False),
                (5.8, 0.0000, 0.0000, False),
                (5.9, 0.0000, 0.0000, False),
                (6.0, 0.0000, 0.0000, False)]
        speedometer = Speedometer(period = 0.5, threshold = 0.001)
        for seconds, distance, expected_avg_vel, expected_moving in path:
            speedometer.update(datetime.timedelta(seconds = seconds), distance)
            avg_vel = speedometer.compute_average_velocity_over_last_period()
            if avg_vel is not None:
                avg_vel = round(avg_vel, 4)
            moving = speedometer.is_moving()
            self.assertEqual(avg_vel, expected_avg_vel)
            self.assertEqual(moving, expected_moving)

if __name__ == '__main__':
    unittest.main()


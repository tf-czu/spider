# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for kalman_localization.py
"""

import unittest
import datetime
import math
import lib.kalman as kalman

class TestKalmanLocalization(unittest.TestCase):

    def test_convergence_to_const_point(self):
        # tests if the Kalman filter converges after 20 steps to the value
        # `[10, 10, 10]` provided that the first input value is `[0, 0, 0]`
        # followed by the series of the values `[10, 10, 10]`
        #
        # otestuje, zda kalmanuv filtr dokonverguje po 20ti krocich k hodnote
        # `[10, 10, 10]`, pokud mu bude zadana najdriv hodnota `[0, 0, 0]` a
        # potom hodnoty `[10, 10, 10]`
        kf = kalman.KalmanFilterLocalization()
        time = datetime.timedelta(seconds = 0)
        xyz = [0, 0, 0]
        xyz_err = [1, 1, 1]
        freq = 10 # Hz
        period = 1 / freq
        period_timedelta = datetime.timedelta(seconds = period)
        kf.input(xyz, time.total_seconds(), xyz_err)
        for k in range(20):
            xyz = [10, 10, 10]
            kf.input(xyz, time.total_seconds(), xyz_err)
            time += period_timedelta
        __, last_xyz = kf.get_last_xyz()
        for i in range(3):
            self.assertAlmostEqual(last_xyz[i], 10.0, places = 0)

    def test_uniform_linear_motion(self):
        # simulates uniform linear motion in the direction of the `velocity`
        # vector;
        # tests if, after 20 steps, the filtered coordinates are close (5%) to
        # the input coordinates
        #
        # simuluje rovnomerny primocary pohyb ve smeru vektoru `velocity`
        # testuje, zda jsou po 20ti krocich filtrovane souradnice blizke (5%)
        # vstupnim souradnicim
        kf = kalman.KalmanFilterLocalization()
        time = datetime.timedelta(seconds = 0)
        xyz = [0, 0, 0]
        xyz_err = [1, 1, 1]
        kf.input(xyz, time.total_seconds(), xyz_err)
        velocity = [1, 2, 0]
        freq = 10 # Hz
        period = 1 / freq
        period_timedelta = datetime.timedelta(seconds = period)
        for k in range(20):
            time += period_timedelta
            new_xyz = [xyz[i] + velocity[i]*period for i in range(3)]
            xyz = new_xyz
            kf.input(xyz, time.total_seconds(), xyz_err)
            __, last_xyz = kf.get_last_xyz()
        relative_error = 3 * [None]
        for i in range(3):
            num = abs(xyz[i] - last_xyz[i])
            den = xyz[i]
            if den == 0 and num == 0:
                relative_error[i] = 0
            elif den == 0:
                relative_error[i] = 1
            else:
                relative_error[i] = num / den
            self.assertLess(relative_error[i], 0.05) # the relative error is supposed to be less than 5%


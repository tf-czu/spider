# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for kalman_localization.py
"""

import unittest
import datetime
import random
import math
import kalman_localization as kalloc

class TestKalmanLocalization(unittest.TestCase):

    def test_convergence_to_const_point(self):
        # otestuje, zda kalmanuv filtr dokonverguje po 20ti krocich k hodnote
        # `[10, 10, 10]`, pokud mu bude zadana najdriv hodnota `[0, 0, 0]` a
        # potom hodnoty `[10, 10, 10]`
        kf = kalloc.KalmanFilterLocalization()
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




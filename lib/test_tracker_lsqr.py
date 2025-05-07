# -*- coding: UTF-8 -*-

import unittest
from unittest.mock import MagicMock

#import random
#import math
from datetime import timedelta
#
#import osgar.lib.quaternion as quaternion
#
#from osgar.bus import Bus
#
#import lsqr_localization as loc

from lib.tracker_lsqr import *

class TestTrackerLeastSquares(unittest.TestCase):

    def test_compute_rotation_and_scale_45deg(self):
        # GPS se pohybuje rovnomerne primocare po primce, ktera svira uhel 45 stupnu s osou x;
        #   v kazdem kroku ujede 1 metr ve smeru osy x a 1 metr ve smeru osy y
        # odometrie se pohybuje rovnomerne primocare ve smeru osy x;
        #   v kazdem kroku ujede 1 metr ve smeru osy x
        sync_gps_odo = []
        for k in range(10):
            time = timedelta(seconds = 0.1 * k)
            gps = [k, k, 0]
            odo = [k, 0, 0]
            ori = [0, 0, 0, 1]
            tra = k
            sync_gps_odo.append(SyncGpsOdo(time = time, gps = gps, odo = odo, ori = ori, tra = tra))
        rot, sca = compute_rotation_and_scale(sync_gps_odo)
        self.assertAlmostEqual(rot[0], 0.0)
        self.assertAlmostEqual(rot[1], 0.0)
        self.assertAlmostEqual(rot[2], math.sin(math.pi / 8))
        self.assertAlmostEqual(rot[3], math.cos(math.pi / 8))
        self.assertAlmostEqual(sca, math.sqrt(2))

    def test_compute_rotation_and_scale_90deg(self):
        # GPS se pohybuje rovnomerne primocare ve smeru osy y;
        #   v kazdem kroku ujede 1 metr ve smeru osy y
        # odometrie se pohybuje rovnomerne primocare ve smeru osy x;
        #   v kazdem kroku ujede 1 metr ve smeru osy x
        sync_gps_odo = []
        for k in range(10):
            time = timedelta(seconds = 0.1 * k)
            gps = [0, k, 0]
            odo = [k, 0, 0]
            ori = [0, 0, 0, 1]
            tra = k
            sync_gps_odo.append(SyncGpsOdo(time = time, gps = gps, odo = odo, ori = ori, tra = tra))
        rot, sca = compute_rotation_and_scale(sync_gps_odo)
        self.assertAlmostEqual(rot[0], 0.0)
        self.assertAlmostEqual(rot[1], 0.0)
        self.assertAlmostEqual(rot[2], math.sin(math.pi / 4))
        self.assertAlmostEqual(rot[3], math.cos(math.pi / 4))
        self.assertAlmostEqual(sca, 1)

    def test_compute_rotation_and_scale_square(self):
        # GPS postupne nabyva hodnot (0, 0), (0, 1), (1, 0), (1, 1)
        # odometrie se pohybuje rovnomerne primocare ve smeru osy x;
        #   v kazdem kroku ujede 1 metr ve smeru osy x
        sync_gps_odo = []
        sync_gps_odo.append(SyncGpsOdo(time = timedelta(seconds = 0.0),
                                       gps = [0, 0, 0],
                                       odo = [0, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 0))
        sync_gps_odo.append(SyncGpsOdo(time = timedelta(seconds = 0.1),
                                       gps = [0, 1, 0],
                                       odo = [1, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 1))
        sync_gps_odo.append(SyncGpsOdo(time = timedelta(seconds = 0.2),
                                       gps = [1, 0, 0],
                                       odo = [2, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 2))
        sync_gps_odo.append(SyncGpsOdo(time = timedelta(seconds = 0.3),
                                       gps = [1, 1, 0],
                                       odo = [3, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 3))
        rot, sca = compute_rotation_and_scale(sync_gps_odo)
        self.assertAlmostEqual(rot[0], 0.0)
        self.assertAlmostEqual(rot[1], 0.0)
        self.assertAlmostEqual(rot[2], 0.3310069414355004)
        self.assertAlmostEqual(rot[3], 0.9436283191604177)
        self.assertAlmostEqual(sca, 0.4573660169594892)

    def test_rotate_and_scale(self):
        n = 20
        for vector in [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]]:
            list_of_params = [
                        [[0, 0, 0], [0, 0, 0], 1],
                        [[1, 2, 3], [0, 0, 0], 1],
                        [[0, 0, 0], [1, 2, 3], 1],
                        [[1, 2, 3], [4, 5, 6], 1],
                        [[0, 0, 0], [0, 0, 0], 5],
                        [[1, 2, 3], [0, 0, 0], 5],
                        [[0, 0, 0], [1, 2, 3], 5],
                        [[1, 2, 3], [4, 5, 6], 5],
                        [[-1, 2, 3], [0, 0, 0], 5],
                        [[0, 0, 0], [-1, -2, 3], 5],
                        [[1, -2, 3], [4, 5, -6], 5],
                    ]
            for input_origin, output_origin, scale in list_of_params:
                shifted = [vector[i] + input_origin[i] for i in range(3)]
                for k in range(n):
                    angle = k * math.pi / n
                    cos_angle = math.cos(angle)
                    sin_angle = math.sin(angle)
                    if vector == [1, 0, 0]:
                        expected = [ scale*cos_angle + output_origin[0],  scale*sin_angle + output_origin[1], output_origin[2]]
                    elif vector == [0, 1, 0]:
                        expected = [-scale*sin_angle + output_origin[0],  scale*cos_angle + output_origin[1], output_origin[2]]
                    elif vector == [-1, 0, 0]:
                        expected = [-scale*cos_angle + output_origin[0], -scale*sin_angle + output_origin[1], output_origin[2]]
                    elif vector == [0, -1, 0]:
                        expected = [ scale*sin_angle + output_origin[0], -scale*cos_angle + output_origin[1], output_origin[2]]
                    cos_angle_half = math.cos(angle / 2)
                    sin_angle_half = math.sin(angle / 2)
                    rot_qua = [0.0, 0.0, sin_angle_half, cos_angle_half]
                    result_by_qua = rotate_and_scale(rot_qua, scale, shifted, input_origin, output_origin)
                    self.assertIsInstance(result_by_qua, list)
                    self.assertEqual(len(result_by_qua), 3)
                    for i in range(3):
                        self.assertAlmostEqual(result_by_qua[i], expected[i])

if __name__ == '__main__':
    unittest.main()


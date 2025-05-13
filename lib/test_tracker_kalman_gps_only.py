# -*- coding: UTF-8 -*-

import unittest

import math

import osgar.lib.quaternion as quaternion

from lib.tracker_kalman_gps_only import *

class TestTrackerKalmanGPSOnly(unittest.TestCase):

    def test_get_quaternion_from_difference(self):
        A = [1, 1, 1]
        B = [2, 2, 2]
        qua = get_quaternion_from_difference(A, B)
        self.assertIsInstance(qua, list)
        self.assertEqual(len(qua), 4)
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], math.sin(math.pi/8))
        self.assertAlmostEqual(qua[3], math.cos(math.pi/8))
        #
        v = quaternion.rotate_vector([1, 0, 0], qua)
        d = [B[0]-A[0], B[1]-A[1], 0]
        norm_d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        for i in range(len(d)):
            d[i] /= norm_d
            self.assertAlmostEqual(v[i], d[i])
        #
        A = [2, 2, 2]
        B = [1, 1, 1]
        qua = get_quaternion_from_difference(A, B)
        self.assertIsInstance(qua, list)
        self.assertEqual(len(qua), 4)
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], math.sin(5*math.pi/8))
        self.assertAlmostEqual(qua[3], math.cos(5*math.pi/8))
        #
        v = quaternion.rotate_vector([1, 0, 0], qua)
        d = [B[0]-A[0], B[1]-A[1], 0]
        norm_d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        for i in range(len(d)):
            d[i] /= norm_d
            self.assertAlmostEqual(v[i], d[i])
        #
        A = [2, 1, 1]
        B = [1, 2, 2]
        qua = get_quaternion_from_difference(A, B)
        self.assertIsInstance(qua, list)
        self.assertEqual(len(qua), 4)
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], math.sin(3*math.pi/8))
        self.assertAlmostEqual(qua[3], math.cos(3*math.pi/8))
        #
        v = quaternion.rotate_vector([1, 0, 0], qua)
        d = [B[0]-A[0], B[1]-A[1], 0]
        norm_d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        for i in range(len(d)):
            d[i] /= norm_d
            self.assertAlmostEqual(v[i], d[i])
        #
        A = [1, 2, 2]
        B = [2, 1, 1]
        qua = get_quaternion_from_difference(A, B)
        self.assertIsInstance(qua, list)
        self.assertEqual(len(qua), 4)
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], math.sin(7*math.pi/8))
        self.assertAlmostEqual(qua[3], math.cos(7*math.pi/8))
        #
        v = quaternion.rotate_vector([1, 0, 0], qua)
        d = [B[0]-A[0], B[1]-A[1], 0]
        norm_d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        for i in range(len(d)):
            d[i] /= norm_d
            self.assertAlmostEqual(v[i], d[i])

if __name__ == '__main__':
    unittest.main()


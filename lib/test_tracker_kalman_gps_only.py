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
        #
        A = [2, 1, 0]
        B = [2, 2, 0]
        qua = get_quaternion_from_difference(A, B)
        self.assertIsInstance(qua, list)
        self.assertEqual(len(qua), 4)
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], math.sin(math.pi/4))
        self.assertAlmostEqual(qua[3], math.cos(math.pi/4))
        #
        v = quaternion.rotate_vector([1, 0, 0], qua)
        d = [B[0]-A[0], B[1]-A[1], 0]
        norm_d = math.sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2])
        for i in range(len(d)):
            d[i] /= norm_d
            self.assertAlmostEqual(v[i], d[i])

    def test_TrackerKalmanGPSOnly_1(self):
        list_of_gps_xyz = [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [2.0, 1.0, 0.0],
                [2.0, 2.0, 0.0],
                [3.0, 2.0, 0.0],
                [3.0, 3.0, 0.0],
                [4.0, 3.0, 0.0],
                [4.0, 4.0, 0.0],
            ]
        #
        cos_45 = 1 / math.sqrt(2)
        sin_45 = 1 / math.sqrt(2)
        list_of_expected_pose3d = [
                [[0.0, 0.0, 0.0], None],
                [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[1.0, 1.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
                [[2.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[2.0, 2.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
                [[3.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[3.0, 3.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
                [[4.0, 3.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[4.0, 4.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
            ]
        tracker = TrackerKalmanGPSOnly(gps_err = [0.0, 0.0, 0.0])
        for k in range(len(list_of_gps_xyz)):
            gps_xyz = list_of_gps_xyz[k]
            time = timedelta(seconds = 0.1 * k)
            tracker.input_gps_xyz(time, gps_xyz)
            pose3d = tracker.get_pose3d()
            expected_pose3d = list_of_expected_pose3d[k]
            if expected_pose3d is None:
                self.assertIsNone(pose3d)
            else:
                expected_xyz, expected_ori = expected_pose3d
                self.assertIsInstance(pose3d, list)
                self.assertEqual(len(pose3d), 2)
                xyz, ori = pose3d
                if expected_xyz is None:
                    self.assertIsNone(xyz)
                else:
                    self.assertIsInstance(xyz, list)
                    self.assertEqual(len(xyz), 3)
                    for i in range(3):
                        self.assertAlmostEqual(xyz[i], expected_xyz[i])
                if expected_ori is None:
                    self.assertIsNone(ori)
                else:
                    self.assertIsInstance(ori, list)
                    self.assertEqual(len(ori), 4)
                    for i in range(3):
                        self.assertAlmostEqual(ori[i], expected_ori[i])
        #
        list_of_expected_pose3d = [
                [[0.0, 0.0, 0.0], None],
                [[0.4464167336435452, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[0.6691194307407189, 0.3794984694921996, 0.0], [0.0, 0.0, 0.49692950750594594, 0.8677909106287632]],
                [[1.1812372041407557, 0.6288268250221619, 0.0], [0.0, 0.0, 0.2246065010940608, 0.9744495470091222]],
                [[1.5493653294882441, 1.1773272145077054, 0.0], [0.0, 0.0, 0.47049074039899297, 0.882404931535861]],
                [[2.190917059132903, 1.5765737243684206, 0.0], [0.0, 0.0, 0.2747528736135407, 0.9615148768693607]],
                [[2.6472979122063203, 2.242324116549911, 0.0], [0.0, 0.0, 0.46614607012864545, 0.8847077716984405]],
                [[3.3475775749675587, 2.7072184142459808, 0.0], [0.0, 0.0, 0.28885610589439786, 0.9573725241970987]],
                [[3.8256579047087507, 3.410259879102113, 0.0], [0.0, 0.0, 0.4678034948488529, 0.8838325012168308]],
            ]
        tracker = TrackerKalmanGPSOnly(gps_err = [1.0, 1.0, 1.0])
        for k in range(len(list_of_gps_xyz)):
            gps_xyz = list_of_gps_xyz[k]
            time = timedelta(seconds = 0.1 * k)
            tracker.input_gps_xyz(time, gps_xyz)
            pose3d = tracker.get_pose3d()
            expected_pose3d = list_of_expected_pose3d[k]
            if expected_pose3d is None:
                self.assertIsNone(pose3d)
            else:
                expected_xyz, expected_ori = expected_pose3d
                self.assertIsInstance(pose3d, list)
                self.assertEqual(len(pose3d), 2)
                xyz, ori = pose3d
                if expected_xyz is None:
                    self.assertIsNone(xyz)
                else:
                    self.assertIsInstance(xyz, list)
                    self.assertEqual(len(xyz), 3)
                    for i in range(3):
                        self.assertAlmostEqual(xyz[i], expected_xyz[i])
                if expected_ori is None:
                    self.assertIsNone(ori)
                else:
                    self.assertIsInstance(ori, list)
                    self.assertEqual(len(ori), 4)
                    for i in range(3):
                        self.assertAlmostEqual(ori[i], expected_ori[i])

    def test_TrackerKalmanGPSOnly_2(self):
        tracker = TrackerKalmanGPSOnly(gps_err = [0.0, 0.0, 0.0])
        list_of_gps_xyz = [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [2.0, 1.0, 0.0],
                [2.0, 2.0, 0.0],
                [3.0, 2.0, 0.0],
                [3.0, 3.0, 0.0],
                [4.0, 3.0, 0.0],
                [4.0, 4.0, 0.0],
            ]
        cos_45 = 1 / math.sqrt(2)
        sin_45 = 1 / math.sqrt(2)
        list_of_expected_pose3d = [
                [[0.0, 0.0, 0.0], None],
                [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[1.0, 1.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
                [[2.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[2.0, 2.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
                [[3.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[3.0, 3.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
                [[4.0, 3.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
                [[4.0, 4.0, 0.0], [0.0, 0.0, cos_45, sin_45]],
            ]
        for k in range(len(list_of_gps_xyz)):
            gps_xyz = list_of_gps_xyz[k]
            time = timedelta(seconds = 0.1 * k)
            tracker.input_gps_xyz(time, gps_xyz)
            pose3d = tracker.get_pose3d()
            #print(k, gps_xyz, pose3d)
            expected_pose3d = list_of_expected_pose3d[k]
            if expected_pose3d is None:
                self.assertIsNone(pose3d)
            else:
                expected_xyz, expected_ori = expected_pose3d
                self.assertIsInstance(pose3d, list)
                self.assertEqual(len(pose3d), 2)
                xyz, ori = pose3d
                if expected_xyz is None:
                    self.assertIsNone(xyz)
                else:
                    self.assertIsInstance(xyz, list)
                    self.assertEqual(len(xyz), 3)
                    for i in range(3):
                        self.assertAlmostEqual(xyz[i], expected_xyz[i])
                if expected_ori is None:
                    self.assertIsNone(ori)
                else:
                    self.assertIsInstance(ori, list)
                    self.assertEqual(len(ori), 4)
                    for i in range(3):
                        self.assertAlmostEqual(ori[i], expected_ori[i])

if __name__ == '__main__':
    unittest.main()


# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for least_squares_localization
"""

import unittest
from unittest.mock import MagicMock

import random
import math
from datetime import timedelta

import osgar.lib.quaternion as quaternion

from osgar.bus import Bus

import lsqr_localization as loc

class TestLeastSquaresLocalization(unittest.TestCase):

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
            sync_gps_odo.append(loc.SyncGpsOdo(time = time, gps = gps, odo = odo, ori = ori, tra = tra))
        rot, qua, sca = loc.compute_rotation_and_scale(sync_gps_odo)
        self.assertAlmostEqual(rot[0][0], math.cos(math.pi / 4))
        self.assertAlmostEqual(rot[0][1], -math.sin(math.pi / 4))
        self.assertAlmostEqual(rot[1][0], math.sin(math.pi / 4))
        self.assertAlmostEqual(rot[1][1], math.cos(math.pi / 4))
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], math.sin(math.pi / 8))
        self.assertAlmostEqual(qua[3], math.cos(math.pi / 8))
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
            sync_gps_odo.append(loc.SyncGpsOdo(time = time, gps = gps, odo = odo, ori = ori, tra = tra))
        rot, qua, sca = loc.compute_rotation_and_scale(sync_gps_odo)
        self.assertAlmostEqual(rot[0][0], math.cos(math.pi / 2))
        self.assertAlmostEqual(rot[0][1], -math.sin(math.pi / 2))
        self.assertAlmostEqual(rot[1][0], math.sin(math.pi / 2))
        self.assertAlmostEqual(rot[1][1], math.cos(math.pi / 2))
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], math.sin(math.pi / 4))
        self.assertAlmostEqual(qua[3], math.cos(math.pi / 4))
        self.assertAlmostEqual(sca, 1)

    def test_compute_rotation_and_scale_square(self):
        # GPS postupne nabyva hodnot (0, 0), (0, 1), (1, 0), (1, 1)
        # odometrie se pohybuje rovnomerne primocare ve smeru osy x;
        #   v kazdem kroku ujede 1 metr ve smeru osy x
        sync_gps_odo = []
        sync_gps_odo.append(loc.SyncGpsOdo(time = timedelta(seconds = 0.0),
                                       gps = [0, 0, 0],
                                       odo = [0, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 0))
        sync_gps_odo.append(loc.SyncGpsOdo(time = timedelta(seconds = 0.1),
                                       gps = [0, 1, 0],
                                       odo = [1, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 1))
        sync_gps_odo.append(loc.SyncGpsOdo(time = timedelta(seconds = 0.2),
                                       gps = [1, 0, 0],
                                       odo = [2, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 2))
        sync_gps_odo.append(loc.SyncGpsOdo(time = timedelta(seconds = 0.3),
                                       gps = [1, 1, 0],
                                       odo = [3, 0, 0],
                                       ori = [0, 0, 0, 1],
                                       tra = 3))
        rot, qua, sca = loc.compute_rotation_and_scale(sync_gps_odo)
        self.assertAlmostEqual(rot[0][0], 0.78086881)
        self.assertAlmostEqual(rot[0][1], -0.62469505)
        self.assertAlmostEqual(rot[1][0], 0.62469505)
        self.assertAlmostEqual(rot[1][1], 0.78086881)
        self.assertAlmostEqual(qua[0], 0.0)
        self.assertAlmostEqual(qua[1], 0.0)
        self.assertAlmostEqual(qua[2], 0.3310069414355004)
        self.assertAlmostEqual(qua[3], 0.9436283191604177)
        self.assertAlmostEqual(sca, 0.4573660169594892)

    def test_compute_rotation_and_scale__totally_random(self):
        # GPS se pohybuje rovnomerne primocare v nahodnem smeru
        # odometrie se pohybuje rovnomerne primocare v nahodnem smeru
        for step in range(20):
            angle_gps = 2 * math.pi * random.random()
            len_dir_gps = 5 * (random.random() + 0.5)
            dir_gps = [len_dir_gps * math.cos(angle_gps), len_dir_gps * math.sin(angle_gps), 0.0]
            orig_gps = [100 * (random.random() - 0.5) for i in range(3)]
            err_gps = 2.0
            #
            angle_odo = 2 * math.pi * random.random()
            len_dir_odo = 5 * (random.random() + 0.5)
            dir_odo = [len_dir_odo * math.cos(angle_odo), len_dir_odo * math.sin(angle_odo), 0.0]
            orig_odo = [100 * (random.random() - 0.5) for i in range(3)]
            err_odo = 0.01
            #
            sync_gps_odo = []
            gps = 3 * [0.0]
            odo = 3 * [0.0]
            for k in range(1000):
                time = timedelta(seconds = 0.1 * k)
                t = time.total_seconds()
                for i in range(2):
                    gps[i] = orig_gps[i] + t*dir_gps[i] + 2*err_gps*(random.random() - 0.5)
                    odo[i] = orig_odo[i] + t*dir_odo[i] + 2*err_odo*(random.random() - 0.5)
                ori = [0, 0, 0, 1]
                tra = len_dir_odo * t
                sync_gps_odo.append(loc.SyncGpsOdo(time = time, gps = gps, odo = odo, ori = ori, tra = tra))
            rot, qua, sca = loc.compute_rotation_and_scale(sync_gps_odo,
                                                       or_g = orig_gps,
                                                       or_o = orig_odo,
                                                       first_index = 0,
                                                       period = None,
                                                       prune = 1)
            self.assertIsInstance(rot, list)
            self.assertEqual(len(rot), 2)
            for row in rot:
                self.assertIsInstance(row, list)
                self.assertEqual(len(row), 2)
                for value in row:
                    self.assertIsInstance(value, float)
            self.assertIsInstance(qua, list)
            self.assertEqual(len(qua), 4)
            for value in qua:
                self.assertIsInstance(value, float)
            self.assertIsInstance(sca, float)
            # test, zda kvaternion odpovida matici rotace
            cos_angle = rot[0][0]
            sin_angle = rot[1][0]
            angle = math.acos(cos_angle)
            if sin_angle < 0.0:
                angle = 2*math.pi - angle
            self.assertAlmostEqual(cos_angle, math.cos(angle))
            self.assertAlmostEqual(sin_angle, math.sin(angle))
            cos_angle_half = math.cos(angle / 2)
            sin_angle_half = math.sin(angle / 2)
            self.assertAlmostEqual(rot[0][0], cos_angle)
            self.assertAlmostEqual(rot[0][1], -sin_angle)
            self.assertAlmostEqual(rot[1][0], sin_angle)
            self.assertAlmostEqual(rot[1][1], cos_angle)
            self.assertAlmostEqual(qua[0], 0.0)
            self.assertAlmostEqual(qua[1], 0.0)
            self.assertAlmostEqual(qua[2], sin_angle_half)
            self.assertAlmostEqual(qua[3], cos_angle_half)
            # pokud je dost vzorku (mame jich 1000), tak predpokladam, ze lze
            # scale odhadnout na 1 desetinne misto
            expected_scale = len_dir_gps / len_dir_odo
            self.assertAlmostEqual(sca, expected_scale, 1)

    def test_rotate_and_scale(self):
        n = 20
        for vector in [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]]:
            for step in range(20):
                input_origin = [10*(random.random() - 0.5), 10*(random.random() - 0.5), 0.0]
                output_origin = [10*(random.random() - 0.5), 10*(random.random() - 0.5), 0.0]
                shifted = [vector[i] + input_origin[i] for i in range(3)]
                scale = 10*random.random()
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
                    #rot_mat = [[cos_angle, -sin_angle], [sin_angle, cos_angle]]
                    #result_by_mat = loc.rotate_and_scale_by_matrix(rot_mat, scale, shifted, input_origin, output_origin)
                    #self.assertIsInstance(result_by_mat, list)
                    #self.assertEqual(len(result_by_mat), 3)
                    #for i in range(3):
                    #    self.assertAlmostEqual(result_by_mat[i], expected[i])
                    cos_angle_half = math.cos(angle / 2)
                    sin_angle_half = math.sin(angle / 2)
                    rot_qua = [0.0, 0.0, sin_angle_half, cos_angle_half]
                    result_by_qua = loc.rotate_and_scale(rot_qua, scale, shifted, input_origin, output_origin)
                    self.assertIsInstance(result_by_qua, list)
                    self.assertEqual(len(result_by_qua), 3)
                    for i in range(3):
                        self.assertAlmostEqual(result_by_qua[i], expected[i])

    def test_rotate_and_scale__quaternion_computed_from_rotation_matrix(self):
        n = 20 # number of angles tested
        for vector in [[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]]:
            for step in range(20):
                input_origin = [10*(random.random() - 0.5), 10*(random.random() - 0.5), 0.0]
                output_origin = [10*(random.random() - 0.5), 10*(random.random() - 0.5), 0.0]
                shifted = [vector[i] + input_origin[i] for i in range(3)]
                scale = 10*random.random()
                for k in range(n):
                    angle = 2 * math.pi * (k / n)
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
                    #rot_mat = [[cos_angle, -sin_angle], [sin_angle, cos_angle]]
                    #result_by_mat = loc.rotate_and_scale_by_matrix(rot_mat, scale, shifted, input_origin, output_origin)
                    #self.assertIsInstance(result_by_mat, list)
                    #self.assertEqual(len(result_by_mat), 3)
                    #for i in range(3):
                    #    self.assertAlmostEqual(result_by_mat[i], expected[i])
                    #cos_angle_half = math.cos(angle / 2)
                    #sin_angle_half = math.sin(angle / 2)
                    sin_angle_half = math.sqrt((1 - cos_angle) / 2)
                    cos_angle_half = math.sqrt((1 + cos_angle) / 2)
                    if sin_angle < 0:
                        cos_angle_half *= -1
                    rot_qua = [0.0, 0.0, sin_angle_half, cos_angle_half]
                    result_by_qua = loc.rotate_and_scale(rot_qua, scale, shifted, input_origin, output_origin)
                    self.assertIsInstance(result_by_qua, list)
                    self.assertEqual(len(result_by_qua), 3)
                    for i in range(3):
                        self.assertAlmostEqual(result_by_qua[i], expected[i])

    def test_rotate_and_scale__totally_random(self):
        for step in range(20):
            angle = 100 * (random.random() - 0.5)
            cos_angle = math.cos(angle)
            sin_angle = math.sin(angle)
            cos_angle_half = math.cos(angle / 2)
            sin_angle_half = math.sin(angle / 2)
            rot = [[cos_angle, -sin_angle], [sin_angle, cos_angle]]
            qua = [0.0, 0.0, sin_angle_half, cos_angle_half]
            scale = 100 * (random.random() + 0.001)
            vector = [100 * (random.random() + 0.5) for i in range(3)]
            input_origin = [100 * (random.random() + 0.5) for i in range(3)]
            output_origin = [100 * (random.random() + 0.5) for i in range(3)]
            #by_mat = loc.rotate_and_scale_by_matrix(rot, scale, vector, input_origin, output_origin)
            by_qua = loc.rotate_and_scale(qua, scale, vector, input_origin, output_origin)
            #self.assertIsInstance(by_mat, list)
            self.assertIsInstance(by_qua, list)
            #self.assertEqual(len(by_mat), 3)
            self.assertEqual(len(by_qua), 3)
            # testuju pouze prvni dve souradnice
            #for i in range(2):
            #    self.assertAlmostEqual(by_mat[i], by_qua[i])

    def test_NMEAParser_1(self):
        # vzal jsem pet po sobe jdoucich hodnot z logu:
        #   01_spider-dynamic-meas-240906_210505.log
        # a spocital ocekavane hodnoty
        nmea_parser = loc.NMEAParser()
        list_of_nmea = [
                {'identifier': '$GNGGA', 'lon': 15.569438022333333, 'lon_dir': 'E', 'lat': 50.3699768605, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.569438040833333, 'lon_dir': 'E', 'lat': 50.369976887, 'lat_dir': 'N', 'utc_time': '205207.95', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2308, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.569438059, 'lon_dir': 'E', 'lat': 50.36997691133333, 'lat_dir': 'N', 'utc_time': '205208.00', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2285, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.569438077333333, 'lon_dir': 'E', 'lat': 50.36997693633333, 'lat_dir': 'N', 'utc_time': '205208.05', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2261, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.5694380965, 'lon_dir': 'E', 'lat': 50.369976963166664, 'lat_dir': 'N', 'utc_time': '205208.10', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2237, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
            ]
        expected_results = [
                [0.00000000, 0.00000000, 0.0],
                [0.00131413, 0.00295126, 0.0],
                [0.00260457, 0.00566122, 0.0],
                [0.00390686, 0.00844543, 0.0],
                [0.00526834, 0.01143381, 0.0],
            ]
        for k in range(len(list_of_nmea)):
            nmea = list_of_nmea[k]
            expected = expected_results[k]
            nmea_parser.parse(nmea)
            xyz = nmea_parser.get_xyz()
            if expected is None:
                self.assertIsNone(xyz)
            else:
                self.assertEqual(len(xyz), len(expected))
                for i in range(len(xyz)):
                    self.assertAlmostEqual(xyz[i], expected[i])

    def test_NMEAParser_2(self):
        # zde jsem si hodnoty GPS vymyslel tak, aby to slo priblizne po jednom
        # metru
        nmea_parser = loc.NMEAParser()
        list_of_nmea = [
                {'identifier': '$GNGGA', 'lon': 15.56940, 'lon_dir': 'E', 'lat': 50.36990, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56941, 'lon_dir': 'E', 'lat': 50.36991, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56942, 'lon_dir': 'E', 'lat': 50.36992, 'lat_dir': 'N', 'utc_time': '205207.95', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2308, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56943, 'lon_dir': 'E', 'lat': 50.36993, 'lat_dir': 'N', 'utc_time': '205208.00', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2285, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56944, 'lon_dir': 'E', 'lat': 50.36994, 'lat_dir': 'N', 'utc_time': '205208.05', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2261, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56945, 'lon_dir': 'E', 'lat': 50.36995, 'lat_dir': 'N', 'utc_time': '205208.10', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2237, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56946, 'lon_dir': 'E', 'lat': 50.36996, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56947, 'lon_dir': 'E', 'lat': 50.36997, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56948, 'lon_dir': 'E', 'lat': 50.36998, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.56949, 'lon_dir': 'E', 'lat': 50.36999, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
            ]
        expected_results = [
                [0.00000000,  0.0000000,  0.0],
                [0.71033916,  1.1136833,  0.0],
                [1.42067831,  2.2273666,  0.0],
                [2.13101747,  3.3410499,  0.0],
                [2.84135662,  4.4547332,  0.0],
                [3.55169578,  5.5684165,  0.0],
                [4.26203493,  6.6820998,  0.0],
                [4.97237409,  7.79578309, 0.0],
                [5.68271325,  8.90946639, 0.0],
                [6.3930524,  10.02314969, 0.0],
            ]
        for k in range(len(list_of_nmea)):
            nmea = list_of_nmea[k]
            expected = expected_results[k]
            nmea_parser.parse(nmea)
            xyz = nmea_parser.get_xyz()
            if expected is None:
                self.assertIsNone(xyz)
            else:
                self.assertEqual(len(xyz), len(expected))
                for i in range(len(xyz)):
                    self.assertAlmostEqual(xyz[i], expected[i])

    def test_OdometryParser_0deg(self):
        # simulace rovne jizdy ve smeru osy x
        # v kazdem kroku ujede 1 metr, takze hodnota `k` odpovida ujete
        # vzdalenosti
        odo_parser = loc.OdometryParser()
        for k in range(10):
            # pohybuje se ve smeru osy x
            odo_parser.parse_odometry([1000*k, 0, 0])
            # nulovy odklon od osy x (prvni euleruv uhel)
            odo_parser.parse_orientation(quaternion.euler_to_quaternion(0, 0, 0))
            raw = odo_parser.get_raw_odometry()
            self.assertEqual(len(raw), 2)
            self.assertAlmostEqual(raw[0], k)
            self.assertAlmostEqual(raw[1], 0)
            xyz = odo_parser.get_xyz()
            self.assertEqual(len(xyz), 3)
            self.assertAlmostEqual(xyz[0], k)
            self.assertAlmostEqual(xyz[1], 0)
            self.assertAlmostEqual(xyz[2], 0)
            tra = odo_parser.get_distance_travelled()
            self.assertAlmostEqual(tra, k)

    def test_OdometryParser_45deg(self):
        # simulace rovne jizdy ve smeru primky, ktera svira uhel 45 stupnu s osou x
        # v kazdem kroku ujede 1 metr ve smeru osy x a 1 metr ve smeru osy y;
        # takze hodnota `k` vynasobena odmocninou ze 2 odpovida ujete
        # vzdalenosti
        odo_parser = loc.OdometryParser()
        for k in range(10):
            # pohybuje se po primce, ktera svira uhel 45 stupnu s osou x
            odo_parser.parse_odometry([1000*k, 1000*k, 0])
            # prvni euleruv uhel je 45 stupnu
            odo_parser.parse_orientation(quaternion.euler_to_quaternion(math.pi/4, 0, 0))
            raw = odo_parser.get_raw_odometry()
            self.assertEqual(len(raw), 2)
            self.assertAlmostEqual(raw[0], k)
            self.assertAlmostEqual(raw[1], k)
            xyz = odo_parser.get_xyz()
            self.assertEqual(len(xyz), 3)
            self.assertAlmostEqual(xyz[0], k)
            self.assertAlmostEqual(xyz[1], k)
            self.assertAlmostEqual(xyz[2], 0)
            tra = odo_parser.get_distance_travelled()
            self.assertAlmostEqual(tra, math.sqrt(2)*k)

    def test_OdometryParser_90deg(self):
        # simulace rovne jizdy ve smeru osy y
        # v kazdem kroku ujede 1 metr, takze hodnota `k` odpovida ujete
        # vzdalenosti
        odo_parser = loc.OdometryParser()
        for k in range(10):
            # pohybuje se ve smeru osy y
            odo_parser.parse_odometry([0, 1000*k, 0])
            # prvni euleruv uhel je 90 stupnu
            odo_parser.parse_orientation(quaternion.euler_to_quaternion(math.pi/2, 0, 0))
            raw = odo_parser.get_raw_odometry()
            self.assertEqual(len(raw), 2)
            self.assertAlmostEqual(raw[0], 0)
            self.assertAlmostEqual(raw[1], k)
            xyz = odo_parser.get_xyz()
            self.assertEqual(len(xyz), 3)
            self.assertAlmostEqual(xyz[0], 0)
            self.assertAlmostEqual(xyz[1], k)
            self.assertAlmostEqual(xyz[2], 0)
            tra = odo_parser.get_distance_travelled()
            self.assertAlmostEqual(tra, k)

    def test_OdometryParser_30deg(self):
        # simulace rovne jizdy ve smeru primky, ktera svira uhel 30 stupnu s osou x
        # v kazdem kroku ujede sqrt(3) metru ve smeru osy x a 1 metr ve smeru osy y;
        # takze hodnota `k` vynasobena 2 odpovida ujete vzdalenosti
        odo_parser = loc.OdometryParser()
        for k in range(10):
            # pohybuje se po primce, ktera svira uhel 30 stupnu s osou x
            odo_parser.parse_odometry([math.sqrt(3)*1000*k, 1000*k, 0])
            # prvni euleruv uhel je 30 stupnu
            odo_parser.parse_orientation(quaternion.euler_to_quaternion(math.pi/6, 0, 0))
            raw = odo_parser.get_raw_odometry()
            self.assertEqual(len(raw), 2)
            self.assertAlmostEqual(raw[0], math.sqrt(3)*k)
            self.assertAlmostEqual(raw[1], k)
            xyz = odo_parser.get_xyz()
            self.assertEqual(len(xyz), 3)
            self.assertAlmostEqual(xyz[0], math.sqrt(3)*k)
            self.assertAlmostEqual(xyz[1], k)
            self.assertAlmostEqual(xyz[2], 0)
            tra = odo_parser.get_distance_travelled()
            self.assertAlmostEqual(tra, 2*k)

    def test_OdometryParser_60deg(self):
        # simulace rovne jizdy ve smeru primky, ktera svira uhel 60 stupnu s osou x
        # v kazdem kroku ujede 1 metr ve smeru osy x a sqrt(3) metru ve smeru osy y;
        # takze hodnota `k` vynasobena 2 odpovida ujete vzdalenosti
        odo_parser = loc.OdometryParser()
        for k in range(10):
            # pohybuje se po primce, ktera svira uhel 60 stupnu s osou x
            odo_parser.parse_odometry([1000*k, math.sqrt(3)*1000*k, 0])
            # prvni euleruv uhel je 60 stupnu
            odo_parser.parse_orientation(quaternion.euler_to_quaternion(math.pi/3, 0, 0))
            raw = odo_parser.get_raw_odometry()
            self.assertEqual(len(raw), 2)
            self.assertAlmostEqual(raw[0], k)
            self.assertAlmostEqual(raw[1], math.sqrt(3)*k)
            xyz = odo_parser.get_xyz()
            self.assertEqual(len(xyz), 3)
            self.assertAlmostEqual(xyz[0], k)
            self.assertAlmostEqual(xyz[1], math.sqrt(3)*k)
            self.assertAlmostEqual(xyz[2], 0)
            tra = odo_parser.get_distance_travelled()
            self.assertAlmostEqual(tra, 2*k)

    def test_LeastSquaresLocalization(self):
        bus = Bus(MagicMock())
        config = {
            'window': 5,
            'post_window': 5,
            'prune': 1,   
            'initial_scale':  1.0,   
            'initial_window': 1.0,   
            'initial_angle': -75,   
        }
        localization = loc.LeastSquaresLocalization(config, bus.handle('abc'))
        list_of_nmea = [
                {'identifier': '$GNGGA', 'lon': 15.00000, 'lon_dir': 'E', 'lat': 50.00000, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00001, 'lon_dir': 'E', 'lat': 50.00001, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00002, 'lon_dir': 'E', 'lat': 50.00002, 'lat_dir': 'N', 'utc_time': '205207.95', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2308, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00003, 'lon_dir': 'E', 'lat': 50.00003, 'lat_dir': 'N', 'utc_time': '205208.00', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2285, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00004, 'lon_dir': 'E', 'lat': 50.00004, 'lat_dir': 'N', 'utc_time': '205208.05', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2261, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00005, 'lon_dir': 'E', 'lat': 50.00005, 'lat_dir': 'N', 'utc_time': '205208.10', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2237, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00006, 'lon_dir': 'E', 'lat': 50.00006, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00007, 'lon_dir': 'E', 'lat': 50.00007, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00008, 'lon_dir': 'E', 'lat': 50.00008, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
                {'identifier': '$GNGGA', 'lon': 15.00009, 'lon_dir': 'E', 'lat': 50.00009, 'lat_dir': 'N', 'utc_time': '205207.90', 'quality': 1, 'sats': 19, 'hdop': 0.6, 'alt': 292.2333, 'a_units': 'M', 'undulation': 43.7576, 'u_units': 'M', 'age': 0.0, 'stn_id': '0000'},
            ]
        expected_gps = [
                None,
                [0.71033916,  1.1136833,  0.0],
                [1.42067831,  2.2273666,  0.0],
                [2.13101747,  3.3410499,  0.0],
                [2.84135662,  4.4547332,  0.0],
                [3.55169578,  5.5684165,  0.0],
                [4.26203493,  6.6820998,  0.0],
                [4.97237409,  7.79578309, 0.0],
                [5.68271325,  8.90946639, 0.0],
                [6.3930524,  10.02314969, 0.0],
            ]
        expected_pose3d = [
                None,
                [[0.0, 0.0, 0.0], [0.0, 0.0, -0.6087614290087207, 0.7933533402912352]],
                [[0.7158618258318257, 1.1136832995820294, 0.0], [0.0, 0.0, 0.47920966031320716, 0.8777004622663136]],
                [[1.4317236516636507, 2.2273665985310025, 0.0], [0.0, 0.0, 0.47920966025647393, 0.8777004622972889]],
                [[2.1475854775772234, 3.3410498978982455, 0.0], [0.0, 0.0, 0.4792096602549543, 0.8777004622981187]],
                [[2.863447303445984, 4.454733196956496, 0.0], [0.0, 0.0, 0.4792096602434726, 0.8777004623043874]],
                [[3.5793091292978496, 5.568416496255569, 0.0], [0.0, 0.0, 0.47920966024615885, 0.8777004623029209]],
                [[3.5793091292978496, 5.568416496255569, 0.0], [0.0, 0.0, 0.47920966024615885, 0.8777004623029209]],
                [[5.011032780977889, 7.795783094599852, 0.0], [0.0, 0.0, 0.4792096602442248, 0.8777004623039769]],
                [[5.7268946068583695, 8.909466393664081, 0.0], [0.0, 0.0, 0.47920966023713296, 0.8777004623078488]],
            ]
        for k in range(len(list_of_nmea)):
            timestamp = timedelta(seconds = k)
            localization.on_odometry([1000*k, 0, 0])
            localization.on_orientation(quaternion.euler_to_quaternion(0, 0, 0))
            localization.on_nmea(list_of_nmea[k])
            pose3d = localization.get_pose3d()
            if expected_pose3d[k] is None:
                self.assertIsNone(pose3d)
            else:
                self.assertIsInstance(pose3d, list)
                self.assertEqual(len(pose3d), 2)
                xyz, ori = pose3d
                self.assertIsInstance(xyz, list)
                self.assertIsInstance(ori, list)
                self.assertEqual(len(xyz), 3)
                self.assertEqual(len(ori), 4)
                expected_xyz, expected_ori = expected_pose3d[k]
                for i in range(3):
                    self.assertAlmostEqual(xyz[i], expected_xyz[i])
                for i in range(4):
                    self.assertAlmostEqual(ori[i], expected_ori[i])

    def test_slerp__border_cases(self):
        A = [random.random() for i in range(4)]
        B = [random.random() for i in range(4)]
        norm_A = math.sqrt(sum(value**2 for value in A))
        norm_B = math.sqrt(sum(value**2 for value in B))
        for i in range(4):
            A[i] /= norm_A
            B[i] /= norm_B
        C = loc.slerp(A, B, 0.0)
        D = loc.slerp(A, B, 1.0)
        for i in range(4):
            self.assertAlmostEqual(A[i], C[i], 3)
            self.assertAlmostEqual(B[i], D[i], 3)

    def test_slerp__various_angles(self):
        sqrt_3 = math.sqrt(3)
        sqrt_1_7 = math.sqrt(1/7)
        sqrt_2_7 = math.sqrt(2/7)
        sqrt_4_7 = math.sqrt(4/7)
        for a in range(40):
            angle_A = 10*math.pi*(a - 20)/40
            sin_A_half = math.sin(angle_A/2)
            cos_A_half = math.cos(angle_A/2)
            for b in range(10):
                angle_B = angle_A + math.pi*(b - 5)/10
                sin_B_half = math.sin(angle_B/2)
                cos_B_half = math.cos(angle_B/2)
                # around x
                A = [sin_A_half, 0.0, 0.0, cos_A_half]
                B = [sin_B_half, 0.0, 0.0, cos_B_half]
                for c in range(10):
                    t = c / 9
                    result = loc.slerp(A, B, t)
                    expected_angle = (1 - t)*angle_A + t*angle_B
                    expected = [math.sin(expected_angle/2), 0.0, 0.0, math.cos(expected_angle/2)]
                    for i in range(4):
                        self.assertAlmostEqual(result[i], expected[i], 3)
                # around y
                A = [0.0, sin_A_half, 0.0, cos_A_half]
                B = [0.0, sin_B_half, 0.0, cos_B_half]
                for c in range(10):
                    t = c / 9
                    result = loc.slerp(A, B, t)
                    expected_angle = (1 - t)*angle_A + t*angle_B
                    expected = [0.0, math.sin(expected_angle/2), 0.0, math.cos(expected_angle/2)]
                    for i in range(4):
                        self.assertAlmostEqual(result[i], expected[i], 3)
                # around z
                A = [0.0, 0.0, sin_A_half, cos_A_half]
                B = [0.0, 0.0, sin_B_half, cos_B_half]
                for c in range(10):
                    t = c / 9
                    result = loc.slerp(A, B, t)
                    expected_angle = (1 - t)*angle_A + t*angle_B
                    expected = [0.0, 0.0, math.sin(expected_angle/2), math.cos(expected_angle/2)]
                    for i in range(4):
                        self.assertAlmostEqual(result[i], expected[i], 3)
                # around (1/sqrt(3), 1/sqrt(3), 1/sqrt(3))
                A = [sin_A_half/sqrt_3, sin_A_half/sqrt_3, sin_A_half/sqrt_3, cos_A_half]
                B = [sin_B_half/sqrt_3, sin_B_half/sqrt_3, sin_B_half/sqrt_3, cos_B_half]
                for c in range(10):
                    t = c / 9
                    result = loc.slerp(A, B, t)
                    expected_angle = (1 - t)*angle_A + t*angle_B
                    exp_crd = math.sin(expected_angle/2) / sqrt_3
                    expected = [exp_crd, exp_crd, exp_crd, math.cos(expected_angle/2)]
                    for i in range(4):
                        self.assertAlmostEqual(result[i], expected[i], 3)
                # around (sqrt(1/7), sqrt(2/7), sqrt(4/7))
                A = [sin_A_half*sqrt_1_7, sin_A_half*sqrt_2_7, sin_A_half*sqrt_4_7, cos_A_half]
                B = [sin_B_half*sqrt_1_7, sin_B_half*sqrt_2_7, sin_B_half*sqrt_4_7, cos_B_half]
                for c in range(10):
                    t = c / 9
                    result = loc.slerp(A, B, t)
                    expected_angle = (1 - t)*angle_A + t*angle_B
                    exp_sin = math.sin(expected_angle/2)
                    exp_cos = math.cos(expected_angle/2)
                    expected = [exp_sin*sqrt_1_7, exp_sin*sqrt_2_7, exp_sin*sqrt_4_7, exp_cos]
                    for i in range(4):
                        self.assertAlmostEqual(result[i], expected[i], 3)

    def test_slerp__around_z__totally_random(self):
        for i in range(20):
            t = random.random()
            angle_A = 10*math.pi*(random.random() - 0.5)
            angle_B = angle_A + math.pi*(random.random() - 0.5)
            expected_angle = (1 - t)*angle_A + t*angle_B
            A = [0.0, 0.0, math.sin(angle_A/2), math.cos(angle_A/2)]
            B = [0.0, 0.0, math.sin(angle_B/2), math.cos(angle_B/2)]
            expected = [0.0, 0.0, math.sin(expected_angle/2), math.cos(expected_angle/2)]
            result = loc.slerp(A, B, t)
            for i in range(4):
                self.assertAlmostEqual(result[i], expected[i], 3)

#    def test_get_pose3d(self):
#        #TODO
#        pass

if __name__ == '__main__':
    unittest.main()


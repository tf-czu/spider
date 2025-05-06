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

from localization import *

class TestLocalization(unittest.TestCase):

    def test_pose2d_encoders_collision(self):
        bus = Bus(MagicMock())
        config = {
            'window': 5,
            'post_window': 5,
            'prune': 1,   
            'initial_scale':  1.0,   
            'initial_window': 1.0,   
            'initial_angle': -75,   
            'enc_scale': 0.00218
        }
        #
        localization = Localization(config, bus.handle('abc'))
        localization.on_pose2d([0, 0, 0])
        with self.assertRaises(AssertionError):
            localization.on_encoders([0, 0])
        #
        localization = Localization(config, bus.handle('abc'))
        localization.on_encoders([0, 0])
        with self.assertRaises(AssertionError):
            localization.on_pose2d([0, 0, 0])
        
    def test_LeastSquaresLocalization(self):
        return
        bus = Bus(MagicMock())
        config = {
            'window': 5,
            'post_window': 5,
            'prune': 1,   
            'initial_scale':  1.0,   
            'initial_window': 1.0,   
            'initial_angle': -75,   
            'enc_scale': 0.00218
        }
        localization = Localization(config, bus.handle('abc'))
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
            localization.on_pose2d([1000*k, 0, 0])
            localization.on_orientation(quaternion.euler_to_quaternion(0, 0, 0))
            localization.on_nmea(list_of_nmea[k])
            print(localization.pose3d)
            #pose3d = localization.get_pose3d()
            #if expected_pose3d[k] is None:
            #    self.assertIsNone(pose3d)
            #else:
            #    self.assertIsInstance(pose3d, list)
            #    self.assertEqual(len(pose3d), 2)
            #    xyz, ori = pose3d
            #    self.assertIsInstance(xyz, list)
            #    self.assertIsInstance(ori, list)
            #    self.assertEqual(len(xyz), 3)
            #    self.assertEqual(len(ori), 4)
            #    expected_xyz, expected_ori = expected_pose3d[k]
            #    for i in range(3):
            #        self.assertAlmostEqual(xyz[i], expected_xyz[i])
            #    for i in range(4):
            #        self.assertAlmostEqual(ori[i], expected_ori[i])

if __name__ == '__main__':
    unittest.main()


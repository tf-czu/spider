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
        
    def test_on_pose2d(self):
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
        expected_gps_xyz = [
                [0.0, 0.0, 0.0],
                [0.7158618258318254, 1.1136832995820294, 0.0],
                [1.431723651663651, 2.227366598372739, -0.0024999999999977263],
                [2.147585477622639, 3.3410498979547683, -0.004799999999988813],
                [2.8634473034544645, 4.454733196745478, -0.0072000000000116415],
                [3.57930912928629, 5.568416496327508, -0.009599999999977626],
                [4.295170955118115, 6.6820997951182175, 0.0],
                [5.011032780949941, 7.795783094700247, 0.0],
                [5.726894606908929, 8.909466393490955, 0.0],
                [6.4427564327407545, 10.023149693072984, 0.0],
            ]
        expected_pose3d = [
                None,
                [[0.25881904510252074, -0.9659258262890683, 0.0], [0.0, 0.0, -0.6087614290087207, 0.7933533402912352]],
                [[0.5726894606654606, 0.8909466396656236, 0.0], [0.0, 0.0, 0.47920966031320716, 0.8777004622663136]],
                [[1.2271917014259868, 1.9091713702033444, 0.0], [0.0, 0.0, 0.4792096602600197, 0.877700462295353]],
                [[1.9089648689526877, 2.96982213148018, 0.0], [0.0, 0.0, 0.47920966025647393, 0.8777004622972889]],
                [[2.6031339122197714, 4.049757451802612, 0.0], [0.0, 0.0, 0.47920966024494993, 0.8777004623035809]],
                [[2.6031339122197714, 4.049757451802612, 0.0], [0.0, 0.0, 0.47920966024494993, 0.8777004623035809]],
                [[4.155002206023627, 6.4640359325183345, 0.0], [0.0, 0.0, 0.47920966023853717, 0.8777004623070821]],
                [[4.930936352908216, 7.671175172984103, 0.0], [0.0, 0.0, 0.47920966024126976, 0.8777004623055902]],
                [[5.706870499839047, 8.878314413234058, 0.0], [0.0, 0.0, 0.4792096602331148, 0.8777004623100427]],
            ]
        for k in range(len(list_of_nmea)):
            localization.on_pose2d([1000*k, 0, 0])
            localization.on_orientation(quaternion.euler_to_quaternion(0, 0, 0))
            localization.on_nmea(list_of_nmea[k])
            # test GPS xyz
            gps_xyz = localization.gps_xyz
            self.assertIsNotNone(gps_xyz)
            self.assertIsInstance(gps_xyz, list)
            self.assertEqual(len(gps_xyz), 3)
            for i in range(3):
                self.assertAlmostEqual(gps_xyz[i], expected_gps_xyz[k][i])
            # test pose3d
            pose3d = localization.pose3d
            if expected_pose3d[k] is None:
                self.assertIsNone(pose3d)
            else:
                self.assertIsNotNone(pose3d)
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

if __name__ == '__main__':
    unittest.main()


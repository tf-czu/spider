# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for localization.py
"""

import unittest
import datetime
import random
import localization as lo

#import teachmedijkstra

class TestLocalization(unittest.TestCase):

    def test_const_velocity(self):
        # Simuluje rovnomerny primocary pohyb ve smeru danem vektorem rychlosti
        # `velocity`
        time = datetime.timedelta(seconds = 0)
        gps_err = [0.1, 0.1, 0.1]
        freq = 10 # Hz
        period = 1 / freq
        for i in range(10):
            # simuluj jizdu robota 10 ruznymi smery
            velocity = [random.randint(-10, 10) for j in range(3)]
            num_steps = 10
            loc = lo.Localization(remember_history = True)
            xyz = [0, 0, 0]
            for k in range(num_steps):
                loc.update_xyz_from_gps(time, xyz, gps_err = gps_err)
                pose3d = loc.get_pose3d()
                (filtered_x, filtered_y, __), __ = pose3d
                # otestuj, zda `filtered_x` a `filtered_y` jsou ve stejnem pomeru,
                # jako `velocity[0]` a `velocity[1]`
                self.assertAlmostEqual(velocity[1]*filtered_x, velocity[0]*filtered_y, places = 7)
                xyz = [xyz[i] + period*velocity[i] for i in range(len(xyz))]

if __name__ == '__main__':
    unittest.main()


# -*- coding: UTF-8 -*-

"""
    Implementation of unit tests for localization.py
"""

import unittest
import datetime
import random
import math
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
            gps_xyz = [0, 0, 0]
            for k in range(num_steps):
                loc.update_xyz_from_gps(time, gps_xyz, gps_err = gps_err)
                pose3d = loc.get_pose3d()
                (filtered_x, filtered_y, __), __ = pose3d
                # otestuj, zda `filtered_x` a `filtered_y` jsou ve stejnem pomeru,
                # jako `velocity[0]` a `velocity[1]`
                self.assertAlmostEqual(velocity[1]*filtered_x, velocity[0]*filtered_y, places = 7)
                gps_xyz = [gps_xyz[i] + period*velocity[i] for i in range(len(gps_xyz))]
        #loc.draw()

    def test_circular_trajectory(self):
        # Simuluje rovnomerny pohyb po kruznici
        time = datetime.timedelta(seconds = 0)
        gps_err = [0.1, 0.1, 0.1]
        freq = 10 # Hz
        period = 1 / freq
        angle = 0
        angular_velocity = 1 # rad / s
        num_steps = 30
        loc = lo.Localization(remember_history = True)
        for k in range(num_steps):
            gps_xyz = [math.cos(angle), math.sin(angle), 0]
            loc.update_xyz_from_gps(time, gps_xyz, gps_err = gps_err)
            pose3d = loc.get_pose3d()
            (filtered_x, filtered_y, __), __ = pose3d
            self.assertAlmostEqual(gps_xyz[0], filtered_x, places = 1)
            self.assertAlmostEqual(gps_xyz[1], filtered_y, places = 1)
            angle += angular_velocity * period
        #loc.draw()

    #def test_circular_trajectory_with_extrapolation(self):
    #    # Simuluje rovnomerny pohyb po kruznici
    #    time = datetime.timedelta(seconds = 0)
    #    gps_err = [0.1, 0.1, 0.1]
    #    freq = 10 # Hz
    #    period = 1 / freq
    #    period_timedelta = datetime.timedelta(seconds = period)
    #    angle = 0
    #    angular_velocity = 1 # rad / s
    #    num_steps = 50
    #    gps_step = 10 # GPS bude aktualizovana v kazdem 5. kroku
    #    loc = lo.Localization(remember_history = True)
    #    for k in range(num_steps):
    #        if k % gps_step == 0:
    #            angle = angular_velocity * time.total_seconds()
    #            print('angle:', angle)
    #            gps_xyz = [math.cos(angle), math.sin(angle), 0]
    #            loc.update_xyz_from_gps(time, gps_xyz, gps_err = gps_err)
    #        pose3d = loc.get_pose3d(time)
    #        (filtered_x, filtered_y, __), __ = pose3d
    #        time += period_timedelta
    #    loc.draw()

if __name__ == '__main__':
    unittest.main()


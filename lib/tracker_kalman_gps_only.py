# -*- coding: UTF-8 -*-

import math

import numpy as np

from lib.tracker import Tracker

from lib.kalman import KalmanFilterLocalization

def get_quaternion_from_difference(A, B):
    """
        Computes quaternion from the difference of the positions A an B.

        Only x and y coordinates are involved; z coordinate is ignored.

        The resulting quaternion is given by the angle between the x-axis
            and the difference vector describing the rotation by this angle
            along the z-axis.

        Args:
            A (list of float): list of two or three values representing a
                position in 2D or 3D space
            B (list of float): list of two or three values representing a
                position in 2D or 3D space

        Returns (list of float): list of four values representing a
            quaternion;
            According to the ROS standard
            (http://wiki.ros.org/tf2/Tutorials/Quaternions), a quaternion
            "a + bi + cj + dk" is returned as `[b, c, d, a]` with the real
            component as the last one.
    """
    d_x = B[0] - A[0]
    d_y = B[1] - A[1]
    d_norm = math.sqrt(d_x*d_x + d_y*d_y)
    assert d_norm > 0
    cos_angle = d_x / d_norm
    sin_angle = d_y / d_norm
    sin_angle_half = math.sqrt((1 - cos_angle) / 2)
    cos_angle_half = math.sqrt((1 + cos_angle) / 2)
    if sin_angle < 0:
        cos_angle_half *= -1
    return [0.0, 0.0, sin_angle_half, cos_angle_half]

class TrackerKalmanGPSOnly(Tracker):
    """
        Computes (smoothed) trajectory from GPS utilizing Kalman filter.

        Input: GPS only, Odometry+IMU ignored

        Output: pose3d
    """

    def __init__(self):
        # Kalman filter
        self.kf = KalmanFilterLocalization()
        # standard deviations of GPS signal
        self.gps_err = [0.0, 0.0, 0.0]
        # last position computed by Kalman filter
        self.xyz = None
        # last orientation computed from the positions computed by Kalman filter
        self.ori = None

    def set_gps_err(self, gps_err):
        """
            Sets the error of the GPS position.

            Args:
                gps_err (list of float): standard deviations (s_x, s_y, s_z) of
                    GPS coordinates (x, y, z); in meters
        """
        self.gps_err = gps_err

    def input_gps_xyz(self, time, xyz):
        self.kf.input(xyz_from_gps, time.total_seconds(), self.gps_err)
        __, curr_xyz = self.kf.get_last_xyz()
        last_xyz = self.xyz
        if curr_xyz is None or last_xyz is None:
            self.ori = None
        else:
            self.ori = get_quaternion_from_difference(last_xyz, curr_xyz)

    def input_distance_travelled(self, time, distance):
        pass

    def input_orientation(self, time, orientation):
        pass

    def get_pose3d(self):
        if self.xyz is None or self.ori is None:
            return None
        else:
            return [self.xyz, self.ori]

    def get_odo_xyz(self):
        return None


# -*- coding: UTF-8 -*-

import math

from lib.trackers import NMEATracker, OdometryTracker
from osgar.node import Node
from lib.localization_lqsr import LocalizationByLeastSquares

class LeastSquaresLocalization(Node):
    """
        Computes trajectory from asynchronously provided GPS, odometry, and IMU
            data.
    """

    def __init__(self, config, bus):
        super().__init__(config, bus)
        # register a stream to be published
        bus.register('pose3d')

        options = {
                "window": config.get('window', 5),
                "post window": config.get('post_window', None),
                "prune": config.get('prune', 1),
                "initial window": config.get('initial_window', 1.0),
                "initial scale": config.get('initial_scale', 1.0),
                "initial angle": config.get('initial_angle', -75),
            }
        self.loc = LocalizationByLeastSquares(options)

        self.encoders_scale = config.get('enc_scale', 0.00218)
        assert self.encoders_scale is not None

        # tracker to convert GPS data to cartesian coordinates
        self.nmea_tracker = NMEATracker()

        # choose the source of odometry
        self.odometry_from = "pose2d"
        #self.odometry_from = "encoders"
        # last x, y coordinates obrained from odometry, if odometry_from == "pose2d"
        self.last_xy = None

    def on_nmea(self, data):
        """
            Process next data obtained from GPS.

            Args:
                data (dict): GPS data according to NMEA format;
                    contains keys:
                        * `"identifier"`
                        * `"lon"`
                        * `"lon_dir"`
                        * `"lat"`
                        * `"lat_dir"`
                        * `"utc_time"`
                        * `"quality"`
                        * `"sats"`
                        * `"hdop"`
                        * `"alt"`
                        * `"a_units"`
                        * `"undulation"`
                        * `"u_units"`
                        * `"age"`
                        * `"stn_id"`
        """
        self.nmea_tracker.input_nmea(data)
        xyz = self.nmea_tracker.get_xyz()
        self.loc.input_gps_xyz(self.time, xyz)

    def on_pose2d(self, data):
        """
            Process next data obtained from odometry.

            Args:
                data (list of int): list of three values `[x, y, heading]`
                    where:
                    * `x` ... x-coordinate in [mm] originating from odometry
                    * `y` ... y-coordinate in [mm] originating from odometry
                    * `heading` ... angle (?)
        """
        if self.odometry_from == "pose2d":
            xy = [data[i] / 1000.0 for i in range(2)] # convert [mm] to [m]
            heading = math.radians(data[2] / 100.0)
            if self.last_xy is None:
                distance = 0.0
            else:
                difference = [xy[i] - self.last_xy[i] for i in range(len(xy))]
                distance = math.sqrt(sum(value**2 for value in difference))
                direction = (difference[0] * math.cos(heading) + difference[1] * math.sin(heading))
                if direction < 0:
                    distance = -distance
            self.last_xy = xy
            self.loc.input_distance_travelled(self.time, distance)

    def on_encoders(self, data):
        """
            Process next data obtained from odometry.

            Args:
                data (list of int): list of two values representing the
                    distance travelled by the left and right (or vice versa?)
                    wheel of the robot
        """
        if self.odometry_from == "encoders":
            distance = self.encoders_scale * ((data[0] + data[1]) / 2)
            self.loc.input_distance_travelled(self.time, distance)

    def on_orientation(self, data):
        """
            Process next data obtained from IMU.

            Args:
                data (list of float): list of four values representing a
                    quaternion that represents the orientation of the robot
        """
        self.loc.input_orientation(self.time, data)

    def get_pose3d(self):
        """
            Returns (list): list with two items
        """
        return self.loc.get_pose3d()

    def draw(self):
        self.loc.draw()


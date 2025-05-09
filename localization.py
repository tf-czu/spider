# -*- coding: UTF-8 -*-

import math

from osgar.node import Node

from osgar.lib.route import Convertor as GPSConvertor

from lib.tracker_lsqr import TrackerLeastSquares
from lib.tracker_kalman import TrackerKalman

class Localization(Node):
    """
        Computes trajectory from asynchronously provided GPS, odometry, and IMU
            data.
    """

    def __init__(self, config, bus):
        super().__init__(config, bus)
        # register a stream to be published
        bus.register('pose3d')

        self.algorithm = config.get('algorithm', None)
        assert self.algorithm is not None
        assert self.algorithm in ["lsqr", "kalman"]

        if self.algorithm == "lsqr":
            options = {
                    "window": config.get('window', 1),
                    "post window": config.get('post window', None),
                    "prune": config.get('prune', 1),
                    "initial window": config.get('initial window', None),
                    "initial scale": config.get('initial scale', None),
                    "initial angle": config.get('initial angle', None),
                }
            self.tracker = TrackerLeastSquares(options)
        elif self.algorithm == "kalman":
            options = {
                    # standard deviation [x, y, z] of position from GPS
                    # (the second argument is the default value)
                    "gps err": config.get('gps err', [2, 2, 6]),
                    # standard deviation [x, y, z] of position computed from odometry and IMU
                    # (the second argument is the default value)
                    "imu err": config.get('imu err', [4, 4, 100]),
                }
            self.tracker = TrackerKalman(options)

        # `on_the_way` indicates whether the robot has started moving
        #   ... this serves to filter out initial GPS values which tend to be messy
        #   ... it is set to True after the robot travels at least `initial_dumb_distance`
        self.initial_dumb_distance = config.get('initial dumb distance', None)
        if self.initial_dumb_distance is None:
            self.on_the_way = True
        else:
            self.on_the_way = False
        self.distance_travelled = 0.0

        # the source of odometry
        self.odometry_from = None
        # last x, y coordinates obrained from odometry, if odometry_from == "pose2d"
        self.last_xy = None
        # scale coefficient to convert odometry ticks to meters, if odometry_from == "encoders"
        # (should be 0.00218 for spider robot)
        self.encoders_scale = config.get('enc scale', None)

        # converting GPS to cartesian coordinates
        self.gps_converter = None
        self.gps_alt_0 = None

        # output (these values are mainly for unit-testing)
        self.pose3d = None
        self.gps_xyz = None

        # for debugging
        self.plot_gps = []
        self.plot_odo = []
        self.plot_pose3d = []
        self.counter_of_odometry_signal = 0

    def on_nmea(self, data):
        """
            Process next data obtained from GPS.

            Args:
                data (dict): GPS data according to NMEA format;
                    contains keys:
                        "identifier", "lon", "lon_dir", "lat", "lat_dir",
                        "utc_time", "quality", "sats", "hdop", "alt",
                        "a_units", "undulation", "u_units", "age", "stn_id"

                    * "quality" ... quality of the GPS signal
                        (https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html):

                        - 0 ... špatně
                        - 1 ... obyč gps
                        - 2 ... lepší gps
                        - 4 ... RTK gps - velmi přesné
                        - 5 ... float rtk, asi lepší než 1, ale potenciálně nevyzpytatelné
        """
        if self.on_the_way:
            if data["quality"] != 0:
                assert data["lon_dir"] == "E"
                assert data["lat_dir"] == "N"
                if self.gps_converter:
                    x, y = self.gps_converter.geo2planar((data["lon"], data["lat"]))
                    z = data["alt"] - self.gps_alt_0
                    self.gps_xyz = [x, y, z]
                else:
                    self.gps_converter = GPSConvertor((data["lon"], data["lat"]))
                    self.gps_alt_0 = data["alt"]
                    self.gps_xyz = [0.0, 0.0, 0.0]
                self.tracker.input_gps_xyz(self.time, self.gps_xyz)
                # for debugging
                if self.verbose:
                    self.plot_gps.append(self.gps_xyz)

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
        if self.odometry_from is None:
            self.odometry_from = "pose2d"
        assert self.odometry_from == "pose2d"
        # for debugging
        if self.verbose:
            self.counter_of_odometry_signal += 1
            if self.counter_of_odometry_signal % 1000 == 0:
                print(self.odometry_from, self.counter_of_odometry_signal)
        #
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
        self.tracker.input_distance_travelled(self.time, distance)
        # output
        self.pose3d = self.tracker.get_pose3d()
        if self.pose3d is not None:
            self.publish('pose3d', self.pose3d)
        # distance travelled
        self.distance_travelled += distance
        if not self.on_the_way and abs(self.distance_travelled) > self.initial_dumb_distance:
            self.on_the_way = True
        # for debugging
        if self.verbose:
            if self.pose3d is not None:
                self.plot_pose3d.append(self.pose3d[0])
            self.plot_odo.append(self.tracker.get_odo_xyz())

    def on_encoders(self, data):
        """
            Process next data obtained from odometry.

            Args:
                data (list of int): list of two values representing the
                    distance travelled by the left and right (or vice versa?)
                    wheel of the robot
        """
        if self.odometry_from is None:
            self.odometry_from = "encoders"
        assert self.odometry_from == "encoders"
        assert self.encoders_scale is not None
        # for debugging
        if self.verbose:
            self.counter_of_odometry_signal += 1
            if self.counter_of_odometry_signal % 1000 == 0:
                print(self.odometry_from, self.counter_of_odometry_signal)
        #
        distance = self.encoders_scale * ((data[0] + data[1]) / 2)
        self.tracker.input_distance_travelled(self.time, distance)
        # output
        self.pose3d = self.tracker.get_pose3d()
        if self.pose3d is not None:
            xyz, ori = self.pose3d
            if xyz is not None and ori is not None:
                self.publish('pose3d', self.pose3d)
        # distance travelled
        self.distance_travelled += distance
        if not self.on_the_way and abs(self.distance_travelled) > self.initial_dumb_distance:
            self.on_the_way = True
        #print(self.time, distance, self.distance_travelled, self.initial_dumb_distance, self.on_the_way)
        # for debugging
        if self.verbose:
            if self.pose3d is not None:
                xyz, ori = self.pose3d
                if xyz is not None:
                    self.plot_pose3d.append(xyz)
            self.plot_odo.append(self.tracker.get_odo_xyz())

    def on_orientation(self, data):
        """
            Process next data obtained from IMU.

            Args:
                data (list of float): list of four values representing a
                    quaternion that represents the orientation of the robot
        """
        self.tracker.input_orientation(self.time, data)

    def draw(self):
        import matplotlib.pyplot as plt
        trajectories = [
                {
                    "trajectory": self.plot_gps,
                    "options": "c.",
                    "label": "GPS",
                },
                {
                    "trajectory": self.plot_odo,
                    "options": "y.",
                    "label": "odometry+IMU",
                },
                {
                    "trajectory": self.plot_pose3d,
                    "options": "b.",
                    "label": "pose3d",
                },
            ]
        for trajectory in trajectories:
            list_of_x = []
            list_of_y = []
            for pos in trajectory["trajectory"]:
                list_of_x.append(pos[0])
                list_of_y.append(pos[1])
            plt.plot(list_of_x, list_of_y, trajectory["options"], label = trajectory["label"])
        plt.legend()
        plt.axis('equal')
        plt.show()


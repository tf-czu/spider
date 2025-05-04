# -*- coding: UTF-8 -*-

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
                "initial angle": config.get('initial_angle', -75)   
            }
        self.loc = LocalizationByLeastSquares(options)

        # data trackers
        self.nmea_tracker = NMEATracker()
        self.odo_tracker_pose2d = OdometryTracker()
        #self.odo_tracker_encoders = OdometryTracker()

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
        self.loc.time = self.time
        self.loc.input_gps_xyz(xyz)

    def on_pose2d(self, data):
        """
            Process next data obtained from odometry.

            Args:
                data (list of int): list of three values `[x, y, heading]`
                    where:
                    * `x` ... x-coordinate in [mm] originating from odometry
                    * `y` ... y-coordinate in [mm] originating from odometry
                    * `heading` ... ???
        """
        self.loc.time = self.time
        self.odo_tracker_pose2d.input_pose2d(data)
        distance = self.odo_tracker_pose2d.get_distance()
        if distance is not None:
            self.loc.input_distance_travelled(distance)
        #self.loc.on_pose2d(data)
        return
        distance_3d = self.odo_tracker_pose2d.input_pose2d(data)
        xyz = self.odo_tracker_pose2d.get_xyz()
        if xyz is not None:
            if self.last_sync_gps is not None:
                s = SyncGpsOdo(time = self.time,
                               gps = self.last_sync_gps,
                               odo = xyz,
                               ori = self.last_sync_ori,
                               tra = self.odo_tracker_pose2d.get_distance_travelled())
                self.sync_gps_odo.append(s)
                self.compute_trajectory()
        pose3d = self.get_pose3d()
        if pose3d is not None:
            self.publish('pose3d', pose3d)
            self.plot_pose3d.append(pose3d[0])

    def on_encoders(self, data):
        #self.loc.time = self.time
        #self.loc.on_encoders(data)
        return
        distance_3d = self.odo_tracker_encoders.input_encoders(data)
        xyz = self.odo_tracker_encoders.get_xyz()
        #print("xyz:", xyz)
        if xyz is not None:
            if self.last_sync_gps is not None:
                s = SyncGpsOdo(time = self.time,
                               gps = self.last_sync_gps,
                               odo = xyz,
                               ori = self.last_sync_ori,
                               tra = self.odo_tracker_encoders.get_distance_travelled())
                #self.sync_gps_odo.append(s)
                #self.compute_trajectory()
            scaled_xyz = [value / 400 for value in xyz]
            self.plot_xyz_by_encoders.append(scaled_xyz)
        #pose3d = self.get_pose3d()
        #if pose3d is not None:
        #    self.publish('pose3d', pose3d)
        #    self.plot_pose3d.append(pose3d[0])

    def on_orientation(self, data):
        self.loc.time = self.time
        self.loc.on_orientation(data)
        return
        """
            Process next data obtained from IMU.

            Args:
                data (list of float): list of four values representing a
                    quaternion that represents the orientation of the robot
        """
        self.odo_tracker_pose2d.input_orientation(data)
        self.odo_tracker_encoders.input_orientation(data)
        self.last_sync_ori = data

    def get_pose3d(self):
        """
            Returns (list): list with two items
        """
        return self.loc.get_pose3d()

    def draw(self):
        self.loc.draw()


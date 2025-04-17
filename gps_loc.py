"""
    TODO
"""

import math
import collections
import copy

from osgar.node import Node
from osgar.lib.route import Convertor
from lib.localization import Localization


Pose2d = collections.namedtuple("Pose2d", ("x", "y", "heading"))


def list2xy(data):
    x = [coord[0] for coord in data]
    y = [coord[1] for coord in data]
    return x, y

class LocalizationNode(Node):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register('pose3d')  # register a stream to be published

        # standard deviation [x, y, z] of position from GPS
        # (the second argument is the default value)
        self.gps_err = config.get('gps_err', [2, 2, 6])
        # standard deviation [x, y, z] of position computed from odometry and IMU
        # (the second argument is the default value)
        self.imu_err = config.get('imu_err', [4, 4, 100])

        self.localization = Localization(gps_err = self.gps_err, imu_err = self.imu_err)

        self.con = None # GPS convertor to planar coordinates
        self.alt_0 = None
        self.verbose = False
        self.debug_gps_orig_moving = []
        self.debug_gps_orig_waiting = []
        self.debug_pose3d_kf = []
        self.debug_pose3d_odo = []
        self.debug_xyz_odo_raw = []
        self.debug_starting_points = [[0.0,0.0]]

    def on_nmea_data(self, data):
        lon = data["lon"]
        lat = data["lat"]
        assert data["lon_dir"] == "E"
        assert data["lat_dir"] == "N"
        alt = data["alt"]
        if self.alt_0 is not None:
            z = alt - self.alt_0
        else:
            self.alt_0 = alt
            z = 0
        if self.con:
            x, y = self.con.geo2planar((lon, lat))
            self.localization.update_xyz_from_gps(self.time, [x, y, z])
            if self.verbose:
                if self.localization.status == "waiting":
                    self.debug_gps_orig_waiting.append([x, y])
                else:
                    self.debug_gps_orig_moving.append([x, y])
        else:
            self.con = Convertor((lon, lat))
        xyz, orientation = self.localization.get_pose3d_kf()
        if self.verbose and xyz is not None:
            self.debug_pose3d_kf.append([xyz[0], xyz[1]])
        #self.publish('pose3d', [xyz, orientation])

    def draw(self):
        # in verbose mode and with --draw parameter: draw a plot
        import matplotlib.pyplot as plt
        x, y = list2xy(self.debug_gps_orig_moving)
        plt.plot(x, y, "k+-", label="gps orig moving")
        x, y = list2xy(self.debug_gps_orig_waiting)
        plt.plot(x, y, "m+-", label="gps orig waiting")
        x, y = list2xy(self.debug_pose3d_kf)
        plt.plot(x, y, "bx-", label="pose3d from Kalman filter")
        x, y = list2xy(self.debug_pose3d_odo)
        plt.plot(x, y, "r.-", label="pose3d from odometry")
        x, y = list2xy(self.debug_xyz_odo_raw)
        plt.plot(x, y, "g.-", label="raw xy from odometry")
        x, y = list2xy(self.debug_starting_points)
        plt.plot(x, y, "mo", label="starting points of Kalman filter")
        
        plt.legend()
        plt.axis('equal')
        plt.show()

class GpsOdoLocalization(LocalizationNode):

    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.last_odom = None

    def on_odom(self, data): # pose2d format required
        x, y, heading = data
        odom = Pose2d(x / 1000.0, y / 1000.0, math.radians(heading / 100.0))
        if self.last_odom is not None:
            dist = math.hypot(odom.x - self.last_odom.x, odom.y - self.last_odom.y)
            direction = ((odom.x - self.last_odom.x) * math.cos(self.last_odom.heading) +
                         (odom.y - self.last_odom.y) * math.sin(self.last_odom.heading))
            if direction < 0:
                dist = -dist
        else:
            dist = 0.0
        self.last_odom = odom

        self.localization.update_distance(self.time, dist)

        xyz, orientation = self.localization.get_pose3d_odo()
        if self.verbose and xyz is not None:
            self.debug_pose3d_odo.append([xyz[0], xyz[1]])

        if self.verbose:
            xyz_odo_raw = self.localization.get_last_xyz_odo_raw()
            if xyz_odo_raw is not None:
                self.debug_xyz_odo_raw.append([xyz_odo_raw[0], xyz_odo_raw[1]])
            if self.localization.ase.origin != self.debug_starting_points[-1]:
                self.debug_starting_points.append(copy.deepcopy(self.localization.ase.origin))

    def on_orientation(self, data):
        """
            This is called when new orientation from IMU is obtained.

            Args:
                data (list of float): quaternion
        """
        self.localization.update_orientation(self.time, data)

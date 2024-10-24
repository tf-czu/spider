"""
    TODO
"""

import math
import collections

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
        self.localization = Localization()
        self.gps_sd = config.get('gps_sd', [2, 2, 6])  # standard deviation [x, y, z]
        self.con = None  # GPS convertor to planar coordinates
        self.alt_0 = None

        self.verbose = False
        self.debug_org_position = []
        self.debug_kalman_position = []
        self.debug_estimated_position = []

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
            self.localization.update_xyz_from_gps(self.time, [x, y, z], gps_err = self.gps_sd)
            if self.verbose:
                self.debug_org_position.append([x, y])

        else:
            self.con = Convertor((lon, lat))


    def draw(self):
        # in verbose mode and with --draw parameter: draw a plot
        import matplotlib.pyplot as plt
        x, y = list2xy(self.debug_org_position)
        plt.plot(x, y, "k+-", label="org_gps")
        x, y = list2xy(self.debug_kalman_position)
        plt.plot(x, y, "rx-", label="kalman")
        x, y = list2xy(self.debug_estimated_position)
        plt.plot(x, y, "b.", label="estimation")
        plt.legend()
        plt.show()


class GpsLocalization(LocalizationNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)

    def on_nmea_data(self, data):
        super().on_nmea_data(data)
        if self.verbose:
            kalman_pose3d = self.localization.get_pose3d()  # get last position from kalman
            if kalman_pose3d:
                xyz, __ = kalman_pose3d
                if xyz is not None:
                    self.debug_kalman_position.append([xyz[0], xyz[1]])

    def on_timer(self, data):
        estimated_pose3d = self.localization.get_pose3d(self.time)
        if estimated_pose3d:
            self.publish("pose3d", estimated_pose3d)
            if self.verbose:
                (x, y, __), __ = estimated_pose3d
                self.debug_estimated_position.append([x, y])


class GpsOdoLocalization(LocalizationNode):
    def __init__(self, config, bus):
        super().__init__(config, bus)

        self.last_odom = None

    def on_odom(self, data):  # pose2d format required
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

        # self.localization.update_dist(dist)
        pose3d = self.localization.get_pose3d()
        if pose3d:
            self.publish("pose3d", pose3d)
            if self.verbose:  # verbose/debug mode
                xyz, __ = pose3d
                if xyz is not None:
                    self.debug_kalman_position.append([xyz[0], xyz[1]])

    def on_orientation(self, data):
        # self.localization.update_orientation(data)
        pass

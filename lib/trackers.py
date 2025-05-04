import math
import copy

from osgar.lib.route import Convertor as GPSConvertor

import osgar.lib.quaternion as quaternion

class NMEATracker:
    """
        Converts GPS data to Cartesian coordinates.
    """
    def __init__(self):
        self.converter = None
        self.alt_0 = None
        self.xyz = None

    def input_nmea(self, data):
        """
            Process next GPS data.

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
        assert data["lon_dir"] == "E"
        assert data["lat_dir"] == "N"
        lon = data["lon"]
        lat = data["lat"]
        alt = data["alt"]
        if self.converter:
            x, y = self.converter.geo2planar((lon, lat))
            z = alt - self.alt_0
            self.xyz = [x, y, z]
        else:
            self.converter = GPSConvertor((lon, lat))
            self.alt_0 = alt
            self.xyz = [0.0, 0.0, 0.0]

    def get_xyz(self):
        """
            Returns (list of float): array of three values `[x, y, z]`
                representing the computed Cartesian coordinates;
                (the coordinates are actually planar since z is always `0.0`)
        """
        return self.xyz

class OdometryTracker:
    """
        Converts odometry and IMU data to Cartesian coordinates.
    """
    def __init__(self):
        self.raw_odo = None
        self.orientation = None
        self.xyz = [0.0, 0.0, 0.0]
        self.distance_travelled = 0.0
        self.odometry_from = None

    def input_pose2d(self, data): # pose2d format required
        """
            Process next odometry data.

            Args:
                data (list of int): list of three values `[x, y, heading]`
                    where:
                    * `x` ... x-coordinate in [mm] originating from odometry
                    * `y` ... y-coordinate in [mm] originating from odometry
                    * `heading` ... ???

            Returns (list of float): 3D vector representing the distance
                travelled relative from the last measurement
        """
        if self.odometry_from is None:
            self.odometry_from = "pose2d"
        assert self.odometry_from == "pose2d"
        #
        current_raw_odo = [data[0] / 1000.0, data[1] / 1000.0]
        heading = math.radians(data[2] / 100.0)
        if self.raw_odo is None:
            distance = 0.0
        else:
            difference = [current_raw_odo[i] - self.raw_odo[i] for i in range(len(current_raw_odo))]
            distance = math.sqrt(sum(value**2 for value in difference))
            direction = (difference[0] * math.cos(heading) + difference[1] * math.sin(heading))
            if direction < 0:
                distance = -distance
            self.distance_travelled += distance
        self.raw_odo = current_raw_odo
        # compute xyz from odometry and IMU
        if self.orientation != None:
            distance_3d = quaternion.rotate_vector([distance, 0.0, 0.0], self.orientation)
            for i in range(len(self.xyz)):
                self.xyz[i] += distance_3d[i]
            return distance_3d
        else:
            return None

    def input_encoders(self, data):
        """
            Process next odometry data.

            Args:
                data (list of int): list of two values representing the
                    distance travelled [no specified unit] on the left and right
                    wheel of the robot;
                    the distance is relative from the last measurement

            Returns (list of float): 3D vector representing the distance
                travelled relative from the last measurement
        """
        if self.odometry_from is None:
            self.odometry_from = "encoders"
        assert self.odometry_from == "encoders"
        #
        distance = (data[0] + data[1]) / 2
        self.distance_travelled += distance
        # compute xyz from odometry and IMU
        if self.orientation != None:
            distance_3d = quaternion.rotate_vector([distance, 0.0, 0.0], self.orientation)
            for i in range(len(self.xyz)):
                self.xyz[i] += distance_3d[i]
            return distance_3d
        else:
            return None

    def input_orientation(self, data):
        """
            Process next IMU data.

            Args:
                data (list of float): list of four values representing a quaternion
        """
        self.orientation = data

    def get_raw_odometry(self):
        """
            Returns (list of float): two coordinates `[x, y]` obtained directly from odometry
        """
        return copy.copy(self.raw_odo)

    def get_xyz(self):
        """
            Returns (list of float): three coordinates `[x, y, z]` computed from IMU and odometry
        """
        return copy.copy(self.xyz)

    def get_distance_travelled(self):
        """
            Returns (float): distance travelled
        """
        return self.distance_travelled


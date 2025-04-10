# -*- coding: UTF-8 -*-

"""
    TODO
"""

import math
import copy
from collections import namedtuple

from osgar.lib.route import Convertor as GPSConvertor
import osgar.lib.quaternion as quaternion
from osgar.node import Node


# SyncGpsOdo
# ... named tuple representing one item in the list of synchronized GPS and
# odometry positions
# Each tuple contains:
#   * time (datetime.timedelta): (absolute) time (time in seconds can be
#   obtained by calling `time.total_seconds()`)
#   * gps (numpy.array): array of 3 (or 2) values representing a position
#   * odo (numpy.array): array of 3 (or 2) values representing a position
#   * tra (float): distance travelled
SyncGpsOdo = namedtuple("SyncGpsOdo", "time gps odo ori tra")

def slerp(A, B, t):
    """
        Interpolates two quaternions utilizing the Spherical Linear
            intERPolation (SLERP).

        The input quaternions are given as list of four float values.
        The order of their coordinates is not important, hence, a quaternion
            "a + bi + cj + dk" can be passed as, e.g., `[a, b, c, d]`
            or `[b, c, d, a]`.
        The coordinates of the result will respect the same order.

        Args:
            A (list of float): quaternion as a list of four values
            B (list of float): quaternion as a list of four values
            t (float): a value between `0.0` and `1.0`

                * if `t == 0.0` then the function returns `A`
                * if `t == 1.0` then the function returns `B`
                * if `0.0 < t < 1.0` then the corresponding value on the
                    shortest path between `A` and `B` is returned

        Returns (list): interpolated quaternion
    """
    dotAB = sum(A[i]*B[i] for i in range(4)) # dot product
    # if the dot product is negative, negate one quaternion to get the shortest
    # path
    if dotAB < 0.0:
        for i in range(4):
            B[i] = -B[i]
        dotAB = -dotAB
    if dotAB > 0.9995:
        # linear interpolation (actually, a convex combination)
        # -----------------------------------------------------
        # (if the dot product is close to 1, the quaternions are very close)
        result = 4 * [0]
        # normalize the resulting quaternion
        sum_sqr = 0
        for i in range(4):
            result[i] = (1 - t)*A[i] + t*B[i] # convex combination
            sum_sqr += result[i]*result[i]
        norm_of_result = math.sqrt(sum_sqr)
        for i in range(4):
            result[i] /= norm_of_result 
        return result
    else:
        # Spherical Linear intERPolation (SLERP)
        # --------------------------------------
        alpha = math.acos(dotAB) # the angle between the quaternions
        beta = alpha * t         # the angle of the result
        coef_B = math.sin(beta) / math.sin(alpha)
        coef_A = math.cos(beta) - dotAB*coef_B
        return [coef_A*A[i] + coef_B*B[i] for i in range(4)]

def draw_trajectories(trajectories):
    """
        Draws a plot (see matplotlib) of several trajectories.

        Args:
            trajectories (list of dict): every item of the list is a `dict`
                with the keys:

                * `"trajectory"` ... list of tuples `(x, y)`
                * `"options"` ... plot options accepted by matplotlib
                * `"label"` ... description of the trajectory

        Example:

            draw_trajectories([{
                    "trajectory": [(0,1), (1,2), (2,4), (3,8), (4,16)],
                    "options": "b.",
                    "label": "Example trajectory",
                }])

    """
    import matplotlib.pyplot as plt
    for trajectory in trajectories:
        list_of_x = []
        list_of_y = []
        for coords in trajectory["trajectory"]:
            list_of_x.append(coords[0])
            list_of_y.append(coords[1])
        plt.plot(list_of_x, list_of_y, trajectory["options"], label = trajectory["label"])
    plt.legend()
    plt.axis('equal')
    plt.show()

def compute_rotation_and_scale(sync_gps_odo,
                               or_g = [0, 0, 0],
                               or_o = [0, 0, 0],
                               first_index = 0,
                               period = None,
                               prune = 1):
    """
        Computes rotation angle and scale coefficient between two series of
            positions (trajectories) measured by GPS and odometry.

        The algorithm utilizes the least squares criterion.

        The function returns two values:
            * rotation ... the 2x2 matrix of rotation
            * scale ... the scale coefficient

        To fit the odometry trajectory to the GPS trajectory, the odometry
            position needs to be multipled by `rotation` and by `scale`;
            see `fit_trajectory()`.

        Only a part of the trajectories can be involved.
        For such a purpose, utilize the parameters `first_index` and `period`.
            
        Args:
            sync_gps_odo (list): list of GPS and odometry positions synchronized
                in time;
                each item of the list is a 3-tuple containing:

                * timestamp (datetime.timedelta): (absolute) time
                    (time in seconds can be obtained by calling
                    `time.total_seconds()`)
                * gps (list): array of 3 (or 2) values representing a position
                * odo (list): array of 3 (or 2) values representing a position
            or_g (list): GPS origin; array of 3 (or 2) values representing a
                position
            or_o (list): odometry origin; array of 3 (or 2) values representing
                a position
            first_index (int): index in `sync_gps_odo` from which the scale and
                rotation is to be computed
            period (int): length of the interval in `sync_gps_odo` from which
                the scale and rotation is to be computed
            prune (int): pruning of the input data;
                If `prune == n` then only every n-th item of `sync_gps_odo` is
                involved.
                This parameter has been implemented to speed up the
                computation.
                The dafault value `1` means that *every* item of `sync_gps_odo`
                is involved.

        Returns (list of list of float, list of float, float):
            * 2x2 array representing the matrix of rotation
            * list of 4 values representing the quaternion
            * scale coefficient
    """
    n = len(sync_gps_odo)
    if period is None:
        period = n
    upper_border = first_index + period
    if upper_border > n:
        upper_border = n
    A = 0
    B = 0
    C = 0
    for k in range(first_index, upper_border):
        if (k - first_index) % prune == 0:
            s = sync_gps_odo[k]
            A += (s.odo[0] - or_o[0])**2 + (s.odo[1] - or_o[1])**2
            B += (s.gps[0] - or_g[0])*(s.odo[0] - or_o[0]) + (s.gps[1] - or_g[1])*(s.odo[1] - or_o[1])
            C += (s.gps[0] - or_g[0])*(s.odo[1] - or_o[1]) - (s.gps[1] - or_g[1])*(s.odo[0] - or_o[0])
    BB_CC = B*B + C*C
    if abs(A) < 1E-07 or abs(BB_CC) < 1E-07:
        return None, None, None
    sqrtBC = math.sqrt(BB_CC)
    scale = sqrtBC / A
    sin_angle = -C / sqrtBC
    cos_angle =  B / sqrtBC
    sin_angle_half = math.sqrt((1 - cos_angle) / 2)
    cos_angle_half = math.sqrt((1 + cos_angle) / 2)
    if sin_angle < 0:
        cos_angle_half *= -1
    rotation_matrix = [[cos_angle, -sin_angle], [sin_angle, cos_angle]]
    rotation_quaternion = [0.0, 0.0, sin_angle_half, cos_angle_half]
    return rotation_matrix, rotation_quaternion, scale

def rotate_and_scale(rotation, scale, vector, input_origin, output_origin):
    """
        Rotates and scales a vector (a position in 3D).

        The vector is first shifted to `input_origin`,
            then it is rotated and scaled by the given quaternion and scale coefficient,
            then it is shifted to `output_origin`.

        The rotatio is performed around the axis given by the vector (0, 0, 1);
            hence it is actually a 2D rotation in x and y.

        Args:
            rotation (list of float): a quaternion representing the rotation; a
                list of four values
            scale (float): scale coefficient; should be a positive number
            vector (list of float): input 3D vector
            input_origin (list of float): 3D vector representing the origin of
                the input vector
            output_origin (list of float): 3D vector representing the origin of
                the output vector
            
        Returns (list of float): transformed 3D vector
    """
    inp = [vector[i] - input_origin[i] for i in range(3)]
    rotated = quaternion.rotate_vector(inp, rotation)
    result = [scale * rotated[i] + output_origin[i] for i in range(3)]
    return result

class NMEAParser:
    """
        Converts GPS data to Cartesian coordinates.
    """
    def __init__(self):
        self.converter = None
        self.alt_0 = None
        self.xyz = None

    def parse(self, data):
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
        if self.converter:
            x, y = self.converter.geo2planar((lon, lat))
            self.xyz = [x, y, 0.0]
        else:
            self.converter = GPSConvertor((lon, lat))
            self.xyz = [0.0, 0.0, 0.0]

    def get_xyz(self):
        """
            Returns (list of float): array of three values `[x, y, z]`
                representing the computed Cartesian coordinates;
                (the coordinates are actually planar since z is always `0.0`)
        """
        return self.xyz

class OdometryParser:
    """
        Converts odometry and IMU data to Cartesian coordinates.
    """
    def __init__(self):
        self.raw_odo = None
        self.orientation = None
        self.xyz = [0.0, 0.0, 0.0]
        self.distance_travelled = 0.0

    def parse_odometry(self, data): # pose2d format required
        """
            Process next odometry data.

            Args:
                data (list of int): list of three values `[x, y, heading]`
                    where:
                    * `x` ... x-coordinate in [mm] originating from odometry
                    * `y` ... y-coordinate in [mm] originating from odometry
                    * `heading` ... ???
        """
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

    def parse_orientation(self, data):
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

class xxxLeastSquaresLocalization:
    """
        Computes trajectory from asynchronously provided GPS, odometry, and IMU
            data.

        IMPORTANT:
            It is expected, that the GPS, odometry, and IMU data are provided
                asynchronously, however, constantly.
            That is, no gaps!
            Actually, the synchronization is performed in a very naive way:
                with each new data from odometry, the last data from GPS and IMU
                are taken and this triplet is then stored to the list
                `sync_gps_odo` from which the resulting trajectory is being
                computed.
            Hence, the algorithm will probably behave inpredictibly if, e.g.,
                the GPS data is not provided for a longer period.
    """
    def __init__(self, window,
                 post_window = None,
                 prune = 1,
                 initial_angle = None,
                 initial_scale = 1.0,
                 initial_window = 1.0):
        """
            Args:
                window (float): window size; can be a number or a tuple;
                    * if it is a number (float or int) then it represents the
                        length of the moving window in meters
                    * if it is a tuple then the first value is expected to be a
                        number (float or int) representing the length of the moving
                        window and the second value is a unit:
                        - "m" ... meters
                        - "s" ... seconds
                        - "i" ... numer of list items, i.e., steps
                        (TODO: "s" and "i" not implemented, yet)
                post_window (float): can be a number, a tuple, or `None` (default);
                    The meaning is the same as the parameter `window`, however,
                    this represents the length of the window that is used in
                    the post-process computing of the trajectory.
                    If it is `None` then no post-processing is performed.
                prune (int): pruning of the input data;
                    If `prune == n` then, when computing the output trajectory,
                        only every n-th item of the input data is involved.
                    This parameter has been implemented to speed up the
                        computation.
                    The dafault value `1` means that *every* item of the input
                        data is involved.
                initial_angle (float): initial angle in degrees
                initial_scale (float):
                initial_window (float):
        """
        self.prune = prune
        # trajectory computed in real time, that is, a new position is computed
        # with every odometry tick
        if isinstance(window, tuple):
            self.window = window[0]
            self.window_unit = window[1]
        else:
            self.window = window
            self.window_unit = "m"
        self.trajectory = []
        self.window_first_index = None # position of the moving window
        # trajectory computed by post-processing, that is, when enough data is
        # gathered
        if post_window is None:
            self.post_processing = False
            self.post_window = None
            self.post_window_unit = None
            self.post_trajectory = None
        elif isinstance(window, tuple):
            self.post_processing = True
            self.post_window = post_window[0]
            self.post_window_unit = post_window[1]
            self.post_trajectory = []
        else:
            self.post_processing = True
            self.post_window = post_window
            self.post_window_unit = "m"
            self.post_trajectory = []
        # series of synchronized positions from GPS and odometry
        self.sync_gps_odo = []
        self.last_sync_gps = None
        self.last_sync_odo = None
        self.last_sync_ori = None
        self.nmea_parser = NMEAParser()
        self.odo_parser = OdometryParser()
        # initial values
        if initial_angle is None:
            self.init_qua = None
        else:
            # uhel pro urceni kvaternionu je polovicni, jinak bych to nasobil
            # 180 stupni a ne 90
            angle_half = math.pi * initial_angle / 360.0
            self.init_qua = [0.0, 0.0, math.sin(angle_half), math.cos(angle_half)]
        self.init_sca = initial_scale
        self.init_win = initial_window
        # output
        self.pose3d = None
        # for debugging
        self.plot_pose3d = []
        self.plot_init = []
        self.plot_est = []

    def add_gps(self, timestamp, gps):
        """
            Process next data obtained from GPS.

            Args:
                timestamp (datetime.timedelta): (absolute) time;
                    Note that the time in seconds can be obtained by calling
                    `timestamp.total_seconds()`.
                gps (dict): GPS data according to NMEA format;
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
        self.nmea_parser.parse(gps)
        xyz = self.nmea_parser.get_xyz()
        if xyz is not None:
            self.last_sync_gps = xyz

    def add_odo(self, timestamp, odo):
        """
            Process next data obtained from odometry.

            Args:
                timestamp (datetime.timedelta): (absolute) time;
                    Note that the time in seconds can be obtained by calling
                    `timestamp.total_seconds()`.
                odo (list of int): list of three values `[x, y, heading]`
                    where:
                    * `x` ... x-coordinate in [mm] originating from odometry
                    * `y` ... y-coordinate in [mm] originating from odometry
                    * `heading` ... ???
        """
        distance_3d = self.odo_parser.parse_odometry(odo)
        xyz = self.odo_parser.get_xyz()
        if xyz is not None:
            self.last_sync_odo = xyz
            if self.last_sync_gps is not None:
                s = SyncGpsOdo(time = timestamp,
                               gps = self.last_sync_gps,
                               odo = self.last_sync_odo,
                               ori = self.last_sync_ori,
                               tra = self.odo_parser.get_distance_travelled())
                self.sync_gps_odo.append(s)
                self.compute_trajectory()
        pose3d = self.get_pose3d()
        if pose3d is not None:
            self.plot_pose3d.append(pose3d[0])

    def add_ori(self, timestamp, ori):
        """
            Process next data obtained from IMU.

            Args:
                timestamp (datetime.timedelta): (absolute) time;
                    Note that the time in seconds can be obtained by calling
                    `timestamp.total_seconds()`.
                data (list of float): list of four values forming a quaternion
                    that represents the orientation of the robot
        """
        self.odo_parser.parse_orientation(ori)
        self.last_sync_ori = ori

    def get_pose3d(self):
        """
            Returns (list): list with two items
        """
        #if len(self.trajectory) == 0:
        #    return None
        #else:
        #    return self.trajectory[-1]
        return self.pose3d

    def compute_post_process_trajectory(self):
        """
            Computes the trajectory by post-processing, that is, when "enough"
                data is gathered.

            Every time the robot travels the distance given by the `window`
                attribute, a new chunk of trajectory is computed utilizing the
                least squares criterium.

            The result is then stored to the `post_trajectory` attribute.
        """
        len_post_trajectory = len(self.post_trajectory)
        if len_post_trajectory == 0:
            first_index = 0
        else:
            first_index = len_post_trajectory - 1
            assert first_index > 0
        last_index = len(self.sync_gps_odo) - 1
        # TODO Tady bude vyhledove rozliseni, v jakych jednotkach `post_window`
        # je a podle toho se pak spocita "vzdalenost" a porovna se s hodnotou
        # `post_window`.
        # Zatim pouze predpokladam, ze jednotkou jsou metry.
        assert self.post_window_unit == "m"
        first = self.sync_gps_odo[first_index]
        last = self.sync_gps_odo[last_index]
        if abs(last.tra - first.tra) > self.post_window:
            # new chunk of post-process trajectory is computed
            s = self.sync_gps_odo[first_index]
            or_o = s.odo
            if first_index == 0:
                #or_g = s.odo
                or_g = s.gps
            else:
                or_g, __ = self.post_trajectory[first_index]
            rot, qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                       or_g = or_g,
                                                       or_o = or_o,
                                                       first_index = first_index,
                                                       period = last_index - first_index + 1)
            for k in range(first_index, last_index + 1):
                s = self.sync_gps_odo[k]
                new_xyz = rotate_and_scale(qua, sca, s.odo, or_o, or_g)
                new_ori = quaternion.multiply(qua, s.ori)
                self.post_trajectory.append([new_xyz, new_ori])

    def compute_trajectory(self):
        """
            Computes in real-time the trajectory of the robot and saves its
                current position to `pose3d` attribute.

            The computation runs in two phases:

                1. before the robot travels the distance given by `window`:

                    - this is characterized by `len(self.trajectory) == 0`

                2. after the robot travels the distance given by `window`:
        """
        # Post-processed Trajectory
        if self.post_processing:
            self.compute_post_process_trajectory()
        # Real-time Trajectory
        len_trajectory = len(self.trajectory)
        if len_trajectory == 0:
            #first_index = 0
            len_sync_gps_odo = len(self.sync_gps_odo)
            last_index = len_sync_gps_odo - 1
            # TODO Tady bude vyhledove rozliseni, v jakych jednotkach `window`
            # je a podle toho se pak spocita "vzdalenost" a porovna se s hodnotou
            # `window`.
            # Zatim pouze predpokladam, ze jednotkou jsou metry.
            assert self.window_unit == "m"
            first = self.sync_gps_odo[0]
            last = self.sync_gps_odo[last_index]
            if abs(last.tra - first.tra) > self.window:
                #or_o = or_g = first.odo
                or_o = first.odo
                or_g = first.gps
                rot, qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                           or_g = or_g,
                                                           or_o = or_o,
                                                           first_index = 0,
                                                           period = len_sync_gps_odo)
                for k in range(len_sync_gps_odo):
                    s = self.sync_gps_odo[k]
                    new_xyz = rotate_and_scale(qua, sca, s.odo, or_o, or_g)
                    new_ori = quaternion.multiply(qua, s.ori)
                    self.trajectory.append([new_xyz, new_ori])
            else:
                # rotation and scale estimated by GPS when the full length
                # window has not been reached, yet
                rot_est, qua_est, sca_est = compute_rotation_and_scale(self.sync_gps_odo,
                                                                       or_g = first.gps,
                                                                       or_o = first.odo,
                                                                       first_index = 0,
                                                                       period = len_sync_gps_odo)
                if all(var is not None for var in [qua_est, sca_est, self.init_qua, self.init_sca]):
                    # Estimating pose3d at the start part of the trajectory
                    # -----------------------------------------------------
                    # self.init_qua, self.init_sca ... initial angle and scale
                    # qua_est, sca_est ... angle and scale computed from the
                    #       GPS data obtained so far; the distance travelled is
                    #       here shorter than the value of `window`
                    # weight ... number between 0.0 and 1.0 representing the
                    #       credibility of qua_est, sca_est
                    travelled = abs(self.odo_parser.get_distance_travelled())
                    if travelled > self.init_win:
                        weight = 1.0
                    else:
                        weight = travelled / self.init_win
                    qua = slerp(self.init_qua, qua_est, weight)
                    sca = (1 - weight)*self.init_sca + weight*sca_est
                    pose3d_xyz = rotate_and_scale(qua, sca, last.odo, first.odo, first.gps)
                    pose3d_ori = quaternion.multiply(qua, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                    # for debugging, to be removed
                    pose3d_xyz_init = rotate_and_scale(self.init_qua, self.init_sca, last.odo, first.odo, first.gps)
                    pose3d_ori_init = quaternion.multiply(self.init_qua, last.ori)
                    self.plot_init.append(pose3d_xyz_init)
                    pose3d_xyz_est = rotate_and_scale(qua_est, sca_est, last.odo, first.odo, first.gps)
                    pose3d_ori_est = quaternion.multiply(qua_est, last.ori)
                    self.plot_est.append(pose3d_xyz_est)
                elif all(var is not None for var in [self.init_qua, self.init_sca]):
                    pose3d_xyz = rotate_and_scale(self.init_qua, self.init_sca, last.odo, first.odo, first.gps)
                    pose3d_ori = quaternion.multiply(self.init_qua, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                elif all(var is not None for var in [qua_est, sca_est]):
                    pose3d_xyz = rotate_and_scale(qua_est, sca_est, last.odo, first.odo, first.gps)
                    pose3d_ori = quaternion.multiply(qua_est, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
            self.window_first_index = 0
        else:
            # TODO Tady bude vyhledove rozliseni, v jakych jednotkach `window`
            # je a podle toho se pak spocita "vzdalenost" a porovna se s hodnotou
            # `window`.
            # Zatim pouze predpokladam, ze jednotkou jsou metry.
            assert self.window_unit == "m"
            # moving the window
            len_sync_gps_odo = len(self.sync_gps_odo)
            last_index = len_sync_gps_odo - 1
            last = self.sync_gps_odo[last_index]
            while True:
                self.window_first_index += 1
                first = self.sync_gps_odo[self.window_first_index]
                if abs(last.tra - first.tra) < self.window:
                    self.window_first_index -= 1
                    first = self.sync_gps_odo[self.window_first_index]
                    break
            or_o = first.odo
            xyz, ori = self.trajectory[self.window_first_index]
            or_g = xyz
            rot, qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                       or_g = or_g,
                                                       or_o = or_o,
                                                       first_index = self.window_first_index,
                                                       period = last_index - self.window_first_index + 1,
                                                       prune = self.prune)
            new_xyz = rotate_and_scale(qua, sca, last.odo, or_o, or_g)
            new_ori = quaternion.multiply(qua, last.ori)
            self.pose3d = [new_xyz, new_ori]
            self.trajectory.append(self.pose3d)

    def draw(self):
        # plot GPS and odometry
        plot_gps = []
        plot_odo = []
        for s in self.sync_gps_odo:
            plot_gps.append(s.gps)
            plot_odo.append(s.odo)
        # list of drawn trajectories
        draw_list = []
        # plot post-processed trajectory
        if self.post_processing:
            plot_post_trajectory = []
            for xyz, ori in self.post_trajectory:
                plot_post_trajectory.append(xyz)
            draw_list.append({
                    "trajectory": plot_post_trajectory,
                    "options": "y.",
                    "label": "post-processing",
                })
        draw_list.extend([
                {
                    "trajectory": plot_gps,
                    "options": "c.",
                    "label": "GPS",
                },
                {
                    "trajectory": plot_odo,
                    "options": "g.",
                    "label": "Odometry & IMU",
                },
                {
                    "trajectory": self.plot_pose3d,
                    "options": "b.",
                    "label": "pose3d",
                },
                {
                    "trajectory": self.plot_init,
                    "options": "y.",
                    "label": "initial estimation by initial angle & scale",
                },
            ])
        draw_trajectories(draw_list)










class LeastSquaresLocalization(Node):
    def __init__(self, config, bus):
        print("PRDEL")
        super().__init__(config, bus)
        # register a stream to be published
        bus.register('pose3d')
        # moving window in meters
        window = config.get('window', 5)
        # window that is used when the trajectory is supposed to be computed by
        # post-processing, that is, when all the data are gathered;
        # if `None` then no post-processed trajectory is computed
        post_window = config.get('post_window', None)
        # pruning of the data;
        # only every n-th data sample is taken when performing the
        # least-squares method
        prune = config.get('prune', 1)   
        # initial angle (in radians) and scale;
        # this is utilized to estimate pose3d at the start part of the
        # trajectory when the distance travelled is still shorter than the
        initial_angle  = config.get('initial_angle', -75)   
        initial_scale  = config.get('initial_scale',  1.0)   
        initial_window = config.get('initial_window', 1.0)   
        #
        #self.localization = xxxLeastSquaresLocalization(
        #                            window,
        #                            post_window = post_window,
        #                            prune = prune,
        #                            initial_angle = initial_angle,
        #                            initial_scale = initial_scale,
        #                            initial_window = initial_window
        #                        )
        self.prune = prune
        # trajectory computed in real time, that is, a new position is computed
        # with every odometry tick
        if isinstance(window, tuple):
            self.window = window[0]
            self.window_unit = window[1]
        else:
            self.window = window
            self.window_unit = "m"
        self.trajectory = []
        self.window_first_index = None # position of the moving window
        # trajectory computed by post-processing, that is, when enough data is
        # gathered
        if post_window is None:
            self.post_processing = False
            self.post_window = None
            self.post_window_unit = None
            self.post_trajectory = None
        elif isinstance(window, tuple):
            self.post_processing = True
            self.post_window = post_window[0]
            self.post_window_unit = post_window[1]
            self.post_trajectory = []
        else:
            self.post_processing = True
            self.post_window = post_window
            self.post_window_unit = "m"
            self.post_trajectory = []
        # series of synchronized positions from GPS and odometry
        self.sync_gps_odo = []
        self.last_sync_gps = None
        self.last_sync_odo = None
        self.last_sync_ori = None
        self.nmea_parser = NMEAParser()
        self.odo_parser = OdometryParser()
        # initial values
        if initial_angle is None:
            self.init_qua = None
        else:
            # uhel pro urceni kvaternionu je polovicni, jinak bych to nasobil
            # 180 stupni a ne 90
            angle_half = math.pi * initial_angle / 360.0
            self.init_qua = [0.0, 0.0, math.sin(angle_half), math.cos(angle_half)]
        self.init_sca = initial_scale
        self.init_win = initial_window
        # output
        self.pose3d = None
        # for debugging
        self.plot_pose3d = []
        self.plot_init = []
        self.plot_est = []


    #def on_nmea_data(self, data):
    #    self.localization.add_gps(self.time, data)
#
#    def on_odom(self, data):
#        self.localization.add_odo(self.time, data)
#
#    def on_orientation(self, data):
#        self.localization.add_ori(self.time, data)
#
#    def draw(self):
#        self.localization.draw()
#
    def on_nmea_data(self, data):
        """
            Process next data obtained from GPS.

            Args:
                timestamp (datetime.timedelta): (absolute) time;
                    Note that the time in seconds can be obtained by calling
                    `timestamp.total_seconds()`.
                gps (dict): GPS data according to NMEA format;
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
        self.nmea_parser.parse(data)
        xyz = self.nmea_parser.get_xyz()
        if xyz is not None:
            self.last_sync_gps = xyz

    def on_odom(self, data):
        """
            Process next data obtained from odometry.

            Args:
                timestamp (datetime.timedelta): (absolute) time;
                    Note that the time in seconds can be obtained by calling
                    `timestamp.total_seconds()`.
                odo (list of int): list of three values `[x, y, heading]`
                    where:
                    * `x` ... x-coordinate in [mm] originating from odometry
                    * `y` ... y-coordinate in [mm] originating from odometry
                    * `heading` ... ???
        """
        distance_3d = self.odo_parser.parse_odometry(data)
        xyz = self.odo_parser.get_xyz()
        if xyz is not None:
            self.last_sync_odo = xyz
            if self.last_sync_gps is not None:
                s = SyncGpsOdo(time = self.time,
                               gps = self.last_sync_gps,
                               odo = self.last_sync_odo,
                               ori = self.last_sync_ori,
                               tra = self.odo_parser.get_distance_travelled())
                self.sync_gps_odo.append(s)
                self.compute_trajectory()
        pose3d = self.get_pose3d()
        if pose3d is not None:
            self.plot_pose3d.append(pose3d[0])

    def on_orientation(self, data):
        """
            Process next data obtained from IMU.

            Args:
                timestamp (datetime.timedelta): (absolute) time;
                    Note that the time in seconds can be obtained by calling
                    `timestamp.total_seconds()`.
                data (list of float): list of four values forming a quaternion
                    that represents the orientation of the robot
        """
        self.odo_parser.parse_orientation(data)
        self.last_sync_ori = data

    def get_pose3d(self):
        """
            Returns (list): list with two items
        """
        #if len(self.trajectory) == 0:
        #    return None
        #else:
        #    return self.trajectory[-1]
        return self.pose3d

    def compute_post_process_trajectory(self):
        """
            Computes the trajectory by post-processing, that is, when "enough"
                data is gathered.

            Every time the robot travels the distance given by the `window`
                attribute, a new chunk of trajectory is computed utilizing the
                least squares criterium.

            The result is then stored to the `post_trajectory` attribute.
        """
        len_post_trajectory = len(self.post_trajectory)
        if len_post_trajectory == 0:
            first_index = 0
        else:
            first_index = len_post_trajectory - 1
            assert first_index > 0
        last_index = len(self.sync_gps_odo) - 1
        # TODO Tady bude vyhledove rozliseni, v jakych jednotkach `post_window`
        # je a podle toho se pak spocita "vzdalenost" a porovna se s hodnotou
        # `post_window`.
        # Zatim pouze predpokladam, ze jednotkou jsou metry.
        assert self.post_window_unit == "m"
        first = self.sync_gps_odo[first_index]
        last = self.sync_gps_odo[last_index]
        if abs(last.tra - first.tra) > self.post_window:
            # new chunk of post-process trajectory is computed
            s = self.sync_gps_odo[first_index]
            or_o = s.odo
            if first_index == 0:
                #or_g = s.odo
                or_g = s.gps
            else:
                or_g, __ = self.post_trajectory[first_index]
            rot, qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                       or_g = or_g,
                                                       or_o = or_o,
                                                       first_index = first_index,
                                                       period = last_index - first_index + 1)
            for k in range(first_index, last_index + 1):
                s = self.sync_gps_odo[k]
                new_xyz = rotate_and_scale(qua, sca, s.odo, or_o, or_g)
                new_ori = quaternion.multiply(qua, s.ori)
                self.post_trajectory.append([new_xyz, new_ori])

    def compute_trajectory(self):
        """
            Computes in real-time the trajectory of the robot and saves its
                current position to `pose3d` attribute.

            The computation runs in two phases:

                1. before the robot travels the distance given by `window`:

                    - this is characterized by `len(self.trajectory) == 0`

                2. after the robot travels the distance given by `window`:
        """
        # Post-processed Trajectory
        if self.post_processing:
            self.compute_post_process_trajectory()
        # Real-time Trajectory
        len_trajectory = len(self.trajectory)
        if len_trajectory == 0:
            #first_index = 0
            len_sync_gps_odo = len(self.sync_gps_odo)
            last_index = len_sync_gps_odo - 1
            # TODO Tady bude vyhledove rozliseni, v jakych jednotkach `window`
            # je a podle toho se pak spocita "vzdalenost" a porovna se s hodnotou
            # `window`.
            # Zatim pouze predpokladam, ze jednotkou jsou metry.
            assert self.window_unit == "m"
            first = self.sync_gps_odo[0]
            last = self.sync_gps_odo[last_index]
            if abs(last.tra - first.tra) > self.window:
                #or_o = or_g = first.odo
                or_o = first.odo
                or_g = first.gps
                rot, qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                           or_g = or_g,
                                                           or_o = or_o,
                                                           first_index = 0,
                                                           period = len_sync_gps_odo)
                for k in range(len_sync_gps_odo):
                    s = self.sync_gps_odo[k]
                    new_xyz = rotate_and_scale(qua, sca, s.odo, or_o, or_g)
                    new_ori = quaternion.multiply(qua, s.ori)
                    self.trajectory.append([new_xyz, new_ori])
            else:
                # rotation and scale estimated by GPS when the full length
                # window has not been reached, yet
                rot_est, qua_est, sca_est = compute_rotation_and_scale(self.sync_gps_odo,
                                                                       or_g = first.gps,
                                                                       or_o = first.odo,
                                                                       first_index = 0,
                                                                       period = len_sync_gps_odo)
                if all(var is not None for var in [qua_est, sca_est, self.init_qua, self.init_sca]):
                    # Estimating pose3d at the start part of the trajectory
                    # -----------------------------------------------------
                    # self.init_qua, self.init_sca ... initial angle and scale
                    # qua_est, sca_est ... angle and scale computed from the
                    #       GPS data obtained so far; the distance travelled is
                    #       here shorter than the value of `window`
                    # weight ... number between 0.0 and 1.0 representing the
                    #       credibility of qua_est, sca_est
                    travelled = abs(self.odo_parser.get_distance_travelled())
                    if travelled > self.init_win:
                        weight = 1.0
                    else:
                        weight = travelled / self.init_win
                    qua = slerp(self.init_qua, qua_est, weight)
                    sca = (1 - weight)*self.init_sca + weight*sca_est
                    pose3d_xyz = rotate_and_scale(qua, sca, last.odo, first.odo, first.gps)
                    pose3d_ori = quaternion.multiply(qua, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                    # for debugging, to be removed
                    pose3d_xyz_init = rotate_and_scale(self.init_qua, self.init_sca, last.odo, first.odo, first.gps)
                    pose3d_ori_init = quaternion.multiply(self.init_qua, last.ori)
                    self.plot_init.append(pose3d_xyz_init)
                    pose3d_xyz_est = rotate_and_scale(qua_est, sca_est, last.odo, first.odo, first.gps)
                    pose3d_ori_est = quaternion.multiply(qua_est, last.ori)
                    self.plot_est.append(pose3d_xyz_est)
                elif all(var is not None for var in [self.init_qua, self.init_sca]):
                    pose3d_xyz = rotate_and_scale(self.init_qua, self.init_sca, last.odo, first.odo, first.gps)
                    pose3d_ori = quaternion.multiply(self.init_qua, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                elif all(var is not None for var in [qua_est, sca_est]):
                    pose3d_xyz = rotate_and_scale(qua_est, sca_est, last.odo, first.odo, first.gps)
                    pose3d_ori = quaternion.multiply(qua_est, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
            self.window_first_index = 0
        else:
            # TODO Tady bude vyhledove rozliseni, v jakych jednotkach `window`
            # je a podle toho se pak spocita "vzdalenost" a porovna se s hodnotou
            # `window`.
            # Zatim pouze predpokladam, ze jednotkou jsou metry.
            assert self.window_unit == "m"
            # moving the window
            len_sync_gps_odo = len(self.sync_gps_odo)
            last_index = len_sync_gps_odo - 1
            last = self.sync_gps_odo[last_index]
            while True:
                self.window_first_index += 1
                first = self.sync_gps_odo[self.window_first_index]
                if abs(last.tra - first.tra) < self.window:
                    self.window_first_index -= 1
                    first = self.sync_gps_odo[self.window_first_index]
                    break
            or_o = first.odo
            xyz, ori = self.trajectory[self.window_first_index]
            or_g = xyz
            rot, qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                       or_g = or_g,
                                                       or_o = or_o,
                                                       first_index = self.window_first_index,
                                                       period = last_index - self.window_first_index + 1,
                                                       prune = self.prune)
            new_xyz = rotate_and_scale(qua, sca, last.odo, or_o, or_g)
            new_ori = quaternion.multiply(qua, last.ori)
            self.pose3d = [new_xyz, new_ori]
            self.trajectory.append(self.pose3d)

    def draw(self):
        # plot GPS and odometry
        plot_gps = []
        plot_odo = []
        for s in self.sync_gps_odo:
            plot_gps.append(s.gps)
            plot_odo.append(s.odo)
        # list of drawn trajectories
        draw_list = []
        # plot post-processed trajectory
        if self.post_processing:
            plot_post_trajectory = []
            for xyz, ori in self.post_trajectory:
                plot_post_trajectory.append(xyz)
            draw_list.append({
                    "trajectory": plot_post_trajectory,
                    "options": "y.",
                    "label": "post-processing",
                })
        draw_list.extend([
                {
                    "trajectory": plot_gps,
                    "options": "c.",
                    "label": "GPS",
                },
                {
                    "trajectory": plot_odo,
                    "options": "g.",
                    "label": "Odometry & IMU",
                },
                {
                    "trajectory": self.plot_pose3d,
                    "options": "b.",
                    "label": "pose3d",
                },
                {
                    "trajectory": self.plot_init,
                    "options": "y.",
                    "label": "initial estimation by initial angle & scale",
                },
            ])
        draw_trajectories(draw_list)


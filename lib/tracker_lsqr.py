# -*- coding: UTF-8 -*-

import math, copy
from collections import namedtuple

import osgar.lib.quaternion as quaternion

from lib.tracker import Tracker

"""
    SyncGpsOdo ... named tuple representing one item in the list of synchronized
      positions obtained from GPS and odometry+IMU

      * time (datetime.timedelta): (absolute) time (note that the time in seconds
          can be obtained by calling `time.total_seconds()`)
      * gps (list of float): [x,y,z] position obtained from GPS [meters]
      * odo (list of float): [x,y,z] position obtained from odometry+IMU [meters]
      * ori (list of float): orientation obtained from IMU as a quaternion
          a+bi+cj+dk represented by the list [b,c,d,a]
      * tra (float): distance travelled [meters]
"""
SyncGpsOdo = namedtuple("SyncGpsOdo", "time gps odo ori tra")

def compute_rotation_and_scale(sync_gps_odo,
                               or_g = [0, 0, 0],
                               or_o = [0, 0, 0],
                               first_index = 0,
                               length = None,
                               prune = 1):
    """
        Computes rotation and scale between two series of positions
            (trajectories) measured by GPS and odometry+IMU.

        The algorithm utilizes the least squares criterion.

        Although 3D positions are required on the input, only x and y
            coordinates are involved in the algorithm; the z coordinate is
            ignored.

        Args:
            sync_gps_odo (list of SyncGpsOdo): list of positions from GPS and
                odometry+IMU synchronized in time;
            or_g (list of float): list of 3 (or 2) values representing the
                origin of the positions given by GPS
            or_o (list of float): list of 3 (or 2) values representing the
                origin of the positions given by odometry+IMU
            first_index (int): index in `sync_gps_odo`; see the `length`
                parameter
            length (int): length of the interval in `sync_gps_odo`;
                if both `first_index` and `length` are provided then the
                rotation and scale are computed only from the corresponding
                part of the list given by `sync_gps_odo`
            prune (int): pruning of the input data;
                If `prune == n` then only every n-th item of `sync_gps_odo` is
                involved when computing the rotation and scale.
                This parameter has been implemented to speed up the
                computation.
                The dafault value `1` means that *every* item of `sync_gps_odo`
                is involved.

        Returns (list of float, float):

            * quaternion as a list of 4 values representing the rotation
            * scale coefficient
    """
    n = len(sync_gps_odo)
    if length is None:
        length = n
    upper_border = first_index + length
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
        return None, None
    sqrtBC = math.sqrt(BB_CC)
    scale = sqrtBC / A
    sin_angle = -C / sqrtBC
    cos_angle =  B / sqrtBC
    sin_angle_half = math.sqrt((1 - cos_angle) / 2)
    cos_angle_half = math.sqrt((1 + cos_angle) / 2)
    if sin_angle < 0:
        cos_angle_half *= -1
    #rotation_matrix = [[cos_angle, -sin_angle], [sin_angle, cos_angle]]
    rotation_quaternion = [0.0, 0.0, sin_angle_half, cos_angle_half]
    return rotation_quaternion, scale

def rotate_and_scale(rotation, scale, vector, input_origin, output_origin):
    """
        Rotates and scales a vector (a position in 3D).

        The vector is first shifted to `input_origin`,
            then it is rotated and scaled by the given quaternion and scale coefficient,
            then it is shifted to `output_origin`.

        The rotation is performed around the axis given by the vector (0, 0, 1);
            hence it is actually a 2D rotation in x and y.

        Args:
            rotation (list of float): a quaternion representing the rotation;
                a quaternion a+bi+cj+dk is expected to be given by the list
                `[b,c,d,a]`.
            scale (float): scale coefficient; should be a positive number
            vector (list of float): input 3D vector; `[x,y,z]`
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

class TrackerLeastSquares(Tracker):
    """
        Computes trajectory from asynchronously provided GPS, odometry, and IMU
            data.

        Input and Output:

            See: Tracker

        Algorithm:

            The initial orientation of the trajectory given by odometry+IMU will
                generally differ from the orientation of the trajectory given by GPS.
            Also, the scale of these two trajectories will differ in general.

            The algorithm is based on the idea of computing the trajectory from
                odometry+IMU and then rotating and scaling it such that it fits best
                (by the least squares criterion) to the trajectory given by GPS.
            This approach would work perfectly if the scale error of the odometry
                and the angle error of the IMU would be perfectly constant ... they are
                not.
            The algorithm therefore does not compute the scale and the angle error
                from the whole input data series but uses a moving window which also
                allows a computataion of the trajectory in real time.

        Important:

            It is expected, that the GPS, odometry, and IMU data are provided
                asynchronously, however, constantly.
            That is, no gaps!
            Actually, the synchronization is performed in a very naive way:
                with each new data from odometry, the last data from GPS and IMU
                are taken and this triplet is then stored to the list
                `sync_gps_odo` from which the resulting trajectory is being
                computed.
            Hence, the algorithm will probably behave inpredictibly if, e.g.,
                the GPS data are not provided for a longer period.

        Author:

            * Milan Petrík (petrikm@tf.czu.cz)
    """

    def __init__(self,
                 window         = 1,
                 post_window    = None,
                 prune          = 1,
                 initial_window = None,
                 initial_scale  = None,
                 initial_angle  = None):
        # ----------------------------------------------------
        # input parameters
        # ----------------------------------------------------
        # length of the moving window in meters
        self.window = window
        # window that is used when the trajectory is supposed to be computed by
        # post-processing, that is, when all the data are gathered;
        # if `None` then no post-processed trajectory is computed
        self.post_window = post_window
        # pruning of the data;
        # only every n-th data sample is taken when performing the
        # least-squares method
        self.prune = prune
        # values for the initial computation of the trajectory when GPS and
        # odometry+IMU is still too unreliable
        self.initial_window = initial_window # in meters
        self.initial_scale = initial_scale
        self.initial_angle = initial_angle # in degrees
        if self.initial_angle is None:
            self.initial_rotation = None
        else:
            # half-angle in radians to determine the rotation quaternion
            angle_half = math.pi * self.initial_angle / 360.0
            self.initial_rotation = [0.0, 0.0, math.sin(angle_half), math.cos(angle_half)]
        # ----------------------------------------------------
        # other attributes
        # ----------------------------------------------------
        # total distance travelled by the robot from time=0
        self.distance_travelled = 0
        # computed trajectories
        self.trajectory = []
        self.post_trajectory = []
        # position of the moving window (index in `sync_gps_odo`)
        self.window_first_index = None
        # time series of synchronized positions from GPS and odometry
        self.sync_gps_odo = []
        self.last_sync_gps = None
        self.last_sync_ori = None
        self.last_sync_odo = [0.0, 0.0, 0.0]
        # output
        self.pose3d = None

    def input_gps_xyz(self, time, xyz):
        """
            Process next position obtained from GPS.

            Args:
                xyz (list of float): list of three values `[x, y, z]`
                    representing cartesian coordinates derived from GPS data
        """
        if xyz is not None:
            self.last_sync_gps = xyz

    def input_distance_travelled(self, time, distance):
        """
            Process next (relative) distance travelled obtained from odometry.

            Args:
                distance (float): distance travelled in meters; relative from
                    the last measurement
        """
        self.distance_travelled += distance
        if self.last_sync_ori is not None:
            distance_3d = quaternion.rotate_vector([distance, 0.0, 0.0], self.last_sync_ori)
            for i in range(len(self.last_sync_odo)):
                self.last_sync_odo[i] += distance_3d[i]
            if self.last_sync_gps is not None:
                s = SyncGpsOdo(time = time,
                               gps = copy.copy(self.last_sync_gps),
                               odo = copy.copy(self.last_sync_odo),
                               ori = copy.copy(self.last_sync_ori),
                               tra = self.distance_travelled)
                self.sync_gps_odo.append(s)
                self.compute_trajectory()

    def input_orientation(self, time, orientation):
        """
            Process next orientation quaternion obtained from IMU.

            Args:
                data (list of float): orientation of the robot as a quaternion
                    a+bi+cj+dk given by the list `[b,c,d,a]`
        """
        self.last_sync_ori = orientation

    def get_pose3d(self):
        """
            Returns (list): `[xyz, ori]` where:

                * xyz ... 3D position given by the list `[x,y,z]`
                * ori ... orientation as a quaternion a+bi+cj+dk given by the
                    list `[b,c,d,a]`
        """
        return copy.copy(self.pose3d)

    def get_odo_xyz(self):
        """
            Returns (list): `[x,y,z]` coordinates derived from odometry and IMU
        """
        return copy.copy(self.last_sync_odo)

    def compute_post_process_trajectory(self):
        """
            Computes the trajectory by post-processing, that is, when "enough"
                data is gathered.

            Every time the robot travels the distance given by the `window`
                attribute, a new chunk of trajectory is computed using the
                least squares criterion.

            The result is then stored to the `post_trajectory` attribute.
        """
        len_post_trajectory = len(self.post_trajectory)
        if len_post_trajectory == 0:
            first_index = 0
        else:
            first_index = len_post_trajectory - 1
            assert first_index > 0
        last_index = len(self.sync_gps_odo) - 1
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
            qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                       or_g = or_g,
                                                       or_o = or_o,
                                                       first_index = first_index,
                                                       length = last_index - first_index + 1)
            for k in range(first_index, last_index + 1):
                s = self.sync_gps_odo[k]
                new_xyz = rotate_and_scale(qua, sca, s.odo, or_o, or_g)
                new_ori = quaternion.multiply(qua, s.ori)
                self.post_trajectory.append([new_xyz, new_ori])

    def get_post_process_trajectory(self):
        """
            Returns (list): the trajectory computed by post-processing;
                or `None` of no such trajectory has been computed
        """
        if self.post_window is None:
            return None
        else:
            return self.post_trajectory

    def compute_trajectory(self):
        """
            Computes in real-time the trajectory of the robot and saves its
                current position to `pose3d` attribute.

            The computation runs in two phases:

                1. before the robot travels the distance given by `initial_window`:

                    - this is characterized by `len(self.trajectory) == 0`

                    - At the beginning, the results given by GPS and
                      Odometry+IMU may be too imprecise.
                      Therefore, during this stage, the resulting position and
                      orientation are given also by `initial_rotation` and
                      `initial_scale` (if provided) which are parameters given
                      by the config.
                      Parameter `weight` is being computed to continuously
                      interpolate between the position and orientation computed
                      by GPS+Odometry+IMU and by
                      initial_rotation+initial_scale.

                2. after the robot travels the distance given by `initial_window`:

                    - this is characterized by `len(self.trajectory) > 0`

                    - Once the length of the distance travelled reaches the
                      value given by `window`, the resulting trajectory of the
                      same length is computed and stored to `trajectory`.
                      From now on, a moving window is used.
                      Hence, with every new GPS+Odometry+IMU data obtained, the
                      new resulting position and orientation is computed from
                      the history of GPS+Odometry+IMU up to the distance
                      travelled given by `window`.
        """
        # Post-processed Trajectory
        if self.post_window is not None:
            self.compute_post_process_trajectory()
        # Real-time Trajectory
        len_trajectory = len(self.trajectory)
        if len_trajectory == 0:
            #first_index = 0
            len_sync_gps_odo = len(self.sync_gps_odo)
            last = self.sync_gps_odo[-1]
            if last.tra is not None and abs(last.tra) > self.window:
                or_o = [0.0, 0.0, 0.0]
                or_g = [0.0, 0.0, 0.0]
                qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                           or_o = or_o,
                                                           or_g = or_g,
                                                           first_index = 0,
                                                           length = len_sync_gps_odo)
                for k in range(len_sync_gps_odo):
                    s = self.sync_gps_odo[k]
                    new_xyz = rotate_and_scale(qua, sca, s.odo, or_o, or_g)
                    new_ori = quaternion.multiply(qua, s.ori)
                    self.trajectory.append([new_xyz, new_ori])
            else:
                # rotation and scale estimated by GPS+Odometry+IMU when the
                # full length window has not been reached, yet
                # ("goi" stands for GPS+Odometry+IMU)
                qua_goi, sca_goi = compute_rotation_and_scale(self.sync_gps_odo,
                                                                       or_o = [0.0, 0.0, 0.0],
                                                                       or_g = [0.0, 0.0, 0.0],
                                                                       first_index = 0,
                                                                       length = len_sync_gps_odo)
                input_origin  = [0.0, 0.0, 0.0]
                output_origin = [0.0, 0.0, 0.0]
                if all(var is not None for var in [qua_goi, sca_goi, self.initial_rotation, self.initial_scale]):
                    # Estimating pose3d at the start part of the trajectory
                    # -----------------------------------------------------
                    # self.initial_rotation, self.initial_scale ... initial angle and scale
                    # qua_goi, sca_goi ... angle and scale computed from the
                    #       GPS data obtained so far; the distance travelled is
                    #       here shorter than the value of `window`
                    # weight ... number between 0.0 and 1.0 representing the
                    #       credibility of qua_goi, sca_goi
                    weight = min(abs(self.distance_travelled) / self.initial_window, 1.0)
                    qua = quaternion.slerp(self.initial_rotation, qua_goi, weight)
                    sca = (1 - weight)*self.initial_scale + weight*sca_goi
                    pose3d_xyz = rotate_and_scale(qua, sca, last.odo, input_origin, output_origin)
                    pose3d_ori = quaternion.multiply(qua, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                elif all(var is not None for var in [self.initial_rotation, self.initial_scale]):
                    pose3d_xyz = rotate_and_scale(self.initial_rotation, self.initial_scale, last.odo, input_origin, output_origin)
                    pose3d_ori = quaternion.multiply(self.initial_rotation, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                elif all(var is not None for var in [qua_goi, sca_goi]):
                    pose3d_xyz = rotate_and_scale(qua_goi, sca_goi, last.odo, input_origin, output_origin)
                    pose3d_ori = quaternion.multiply(qua_goi, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
            self.window_first_index = 0
        else:
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
            qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                       or_g = or_g,
                                                       or_o = or_o,
                                                       first_index = self.window_first_index,
                                                       length = last_index - self.window_first_index + 1,
                                                       prune = self.prune)
            new_xyz = rotate_and_scale(qua, sca, last.odo, or_o, or_g)
            new_ori = quaternion.multiply(qua, last.ori)
            self.pose3d = [new_xyz, new_ori]
            self.trajectory.append(self.pose3d)


# -*- coding: UTF-8 -*-

"""
    Computes trajectory from asynchronously provided GPS, odometry, and IMU
        data.

    Author:
        Milan Petrík, petrikm@tf.czu.cz

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
            the GPS data are not provided for a longer period.

    Algorithm:

        There are the following observations:
            * GPS has a significant non-Gaussian error and the correct position
                can differ by several meters, however, GPS also provides data that
                are well reliable on long distances.
            * Odometry provides the distance travelled which differs from the
                real distance travelled by a scale factor.
            * IMU gives an orientation which is, however, relative. Hence its
                initial orientation may differ from the real initial orientation
                significantly.

        The algorithm is based on the idea of computing the trajectory from
            odometry and IMU and then rotating and scaling it such that it fits
            best (the least squares method) to the trajectory given by GPS.
        This approach would work perfectly if the scale error of the odometry
            and the angle error of the IMU would be constant ... they are not.
        The algorithm therefore does not compute the scale and the angle error
            from the whole input data series but uses a moving window.
        This also allows a computat,ion of the trajectory in real time.
                                   ,
    Contents:                      ,

        * `SyncGpsOdo`: named tuple representing one item in the list of
            synchronized GPS and odometry positions
        * `draw_trajectories()`: draws a plot of several trajectories using the
            matplotlib module
        * `compute_rotation_and_scale()`: utilizing the least squares method,
            computes rotation angle and scale coefficient between two series of
            positions (trajectories) measured by GPS and odometry
        * `rotate_and_scale()`: rotates and scales a vector utilizing the
            quaternion arithmetic
        * `NMEATracker`: converts GPS data to Cartesian coordinates
        * `OdometryTracker`: converts odometry and IMU data to Cartesian
            coordinates
        * `LeastSquaresLocalization`: the main class inherited from
            osgar.node.Node that computes the trajectory
"""

import math, copy
from collections import namedtuple

import osgar.lib.quaternion as quaternion

from lib.trackers import NMEATracker, OdometryTracker

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

class LocalizationByLeastSquares:
    """
        Computes trajectory from asynchronously provided GPS, odometry, and IMU
            data.

        Attributes:

            * `sync_gps_odo` (list of SyncGpsOdo): synchronized input data
                = positions from GPS and odometry+IMU
            * `last_sync_gps` (list of float): x, y, z representing the last
                position obtained from GPS (z is always 0.0)
            * `last_sync_ori` (list of float): quaternion representing the last
                orientation obtained from IMU
            * `pose3d` (list of list of float): output ... list with two items
                containing position and orientation

            * `trajectory` (list of pose3d): real-time computed trajectory
            * `window` (float): length of this window in meters
            * `window_first_index` (int): first position of `window` in
                `sync_gps_odo`

            * `post_trajectory` (list of pose3d): trajectory computed when all
                data are gathered
            * `post_window` (float): length of this window in meters 

            * `prune` (int): pruning of the input data; if prune == n then only
                every n-th item of `sync_gps_odo` is considered when computing
                rotation and scale

            * `initial_scale` (float): initial scale
            * `initial_rotation` (list of float): initial rotation as a quaternion
            * `initial_window` (float): initial length (in meters) of the
                trajectory on which the trajectory is estimated by `initial_scale`
                and `initial_rotation`

            * `nmea_tracker` (NMEATracker): tracks trajectory given by GPS
                input
            * `odo_tracker_pose2d` (OdometryTracker): tracks trajectory given
                by odometry and IMU input
    """

    def __init__(self, options):
        # ----------------------------------------------------
        # input parameters
        # ----------------------------------------------------
        # length of the moving window in meters
        # used when the trajectory is computed in real time
        self.window = options["window"]
        # window that is used when the trajectory is supposed to be computed by
        # post-processing, that is, when all the data are gathered;
        # if `None` then no post-processed trajectory is computed
        # trajectory computed by post-processing, that is, when enough data is
        # gathered
        self.post_window = options["post window"]
        # pruning of the data;
        # only every n-th data sample is taken when performing the
        # least-squares method
        self.prune = options["prune"]
        # TODO doc
        # initial window and values
        # for first 
        # angle (in ), scale and rotation quaternion
        # this is utilized to estimate pose3d at the start part of the
        # trajectory when the distance travelled is still shorter than the
        self.initial_window = options["initial window"] # in meters
        self.initial_scale = options["initial scale"]
        self.initial_angle = options["initial angle"] # in degrees
        if self.initial_angle is None:
            self.initial_rotation = None
        else:
            # half-angle in radians to determine the rotation quaternion
            angle_half = math.pi * self.initial_angle / 360.0
            self.initial_rotation = [0.0, 0.0, math.sin(angle_half), math.cos(angle_half)]
        # ----------------------------------------------------
        # other attributes
        # ----------------------------------------------------
        # computed trajectories
        self.trajectory = []
        self.post_trajectory = []
        # position of the moving window
        self.window_first_index = None
        # time series of synchronized positions from GPS and odometry
        self.sync_gps_odo = []
        self.last_sync_gps = None
        self.last_sync_ori = None
        self.last_sync_odo = [0.0, 0.0, 0.0]
        # output
        self.pose3d = None
        # for debugging
        self.plot_pose3d = []
        self.plot_init = []
        self.plot_est = []

        self.plot_xyz_by_encoders = []

        self.distance_travelled = 0

    def input_gps_xyz(self, time, xyz):
        """
            Process next data obtained from GPS.

            Args:
                xyz (list of float): list of three values `[x, y, z]`
                    representing cartesian coordinates which are expected to be
                    derived from GPS data
        """
        if xyz is not None:
            self.last_sync_gps = xyz

    def input_distance_travelled(self, time, distance):
        """
            Process next data obtained from odometry.

            Args:
                distance (float): distance travelled in meters; relative from
                    the last measurement;
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
                #if self.post_window is not None:
                #    self.compute_post_process_trajectory()


        pose3d = self.get_pose3d()
        if pose3d is not None:
            #self.publish('pose3d', pose3d)
            self.plot_pose3d.append(pose3d[0])

    def input_orientation(self, time, data):
        """
            Process next data obtained from IMU.

            Args:
                data (list of float): list of four values representing a
                    quaternion that represents the orientation of the robot
        """
        self.last_sync_ori = data

    def get_pose3d(self):
        """
            Returns (list): list with two items
        """
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
        # TODO complete documentation
        """
            Computes in real-time the trajectory of the robot and saves its
                current position to `pose3d` attribute.

            The computation runs in two phases:

                1. before the robot travels the distance given by `initial_window`:

                    - this is characterized by `len(self.trajectory) == 0`

                2. after the robot travels the distance given by `initial_window`:
        """

#def compute_rotation_and_scale(sync_gps_odo,
#                               or_g = [0, 0, 0],
#                               or_o = [0, 0, 0],
#                               first_index = 0,
#                               period = None,
#                               prune = 1):
#def rotate_and_scale(rotation, scale, vector, input_origin, output_origin):


        # Post-processed Trajectory
        #if self.post_window is not None:
        #    self.compute_post_process_trajectory()
        #return
        # Real-time Trajectory
        len_trajectory = len(self.trajectory)
        if len_trajectory == 0:
            #first_index = 0
            len_sync_gps_odo = len(self.sync_gps_odo)
            #last_index = len_sync_gps_odo - 1
            #first = self.sync_gps_odo[0]
            #last = self.sync_gps_odo[last_index]
            last = self.sync_gps_odo[-1]
            #if abs(last.tra - first.tra) > self.window:
            if last.tra is not None and abs(last.tra) > self.window:
                #or_o = or_g = first.odo
                #or_o = first.odo
                #or_g = first.gps
                # TODO zrusit rot
                or_o = [0.0, 0.0, 0.0]
                or_g = [0.0, 0.0, 0.0]
                rot, qua, sca = compute_rotation_and_scale(self.sync_gps_odo,
                                                           or_o = or_o,
                                                           or_g = or_g,
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
                                                                       or_o = [0.0, 0.0, 0.0],
                                                                       or_g = [0.0, 0.0, 0.0],
                                                                       first_index = 0,
                                                                       period = len_sync_gps_odo)
                input_origin  = [0.0, 0.0, 0.0]
                output_origin = [0.0, 0.0, 0.0]
                if all(var is not None for var in [qua_est, sca_est, self.initial_rotation, self.initial_scale]):
                    # Estimating pose3d at the start part of the trajectory
                    # -----------------------------------------------------
                    # self.initial_rotation, self.initial_scale ... initial angle and scale
                    # qua_est, sca_est ... angle and scale computed from the
                    #       GPS data obtained so far; the distance travelled is
                    #       here shorter than the value of `window`
                    # weight ... number between 0.0 and 1.0 representing the
                    #       credibility of qua_est, sca_est
                    if self.distance_travelled > self.initial_window:
                        weight = 1.0
                    else:
                        weight = self.distance_travelled / self.initial_window
                    qua = quaternion.slerp(self.initial_rotation, qua_est, weight)
                    sca = (1 - weight)*self.initial_scale + weight*sca_est
                    pose3d_xyz = rotate_and_scale(qua, sca, last.odo, input_origin, output_origin)
                    pose3d_ori = quaternion.multiply(qua, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                    # TODO for debugging, to be removed
                    pose3d_xyz_init = rotate_and_scale(self.initial_rotation, self.initial_scale, last.odo, input_origin, output_origin)
                    pose3d_ori_init = quaternion.multiply(self.initial_rotation, last.ori)
                    self.plot_init.append(pose3d_xyz_init)
                    pose3d_xyz_est = rotate_and_scale(qua_est, sca_est, last.odo, input_origin, output_origin)
                    pose3d_ori_est = quaternion.multiply(qua_est, last.ori)
                    self.plot_est.append(pose3d_xyz_est)
                elif all(var is not None for var in [self.initial_rotation, self.initial_scale]):
                    pose3d_xyz = rotate_and_scale(self.initial_rotation, self.initial_scale, last.odo, input_origin, output_origin)
                    pose3d_ori = quaternion.multiply(self.initial_rotation, last.ori)
                    self.pose3d = [pose3d_xyz, pose3d_ori]
                elif all(var is not None for var in [qua_est, sca_est]):
                    pose3d_xyz = rotate_and_scale(qua_est, sca_est, last.odo, input_origin, output_origin)
                    pose3d_ori = quaternion.multiply(qua_est, last.ori)
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
        import matplotlib.pyplot as plt

        #print("len:", len(self.plot_xyz_by_encoders))
        #for xyz in self.plot_xyz_by_encoders:
        #    print(xyz)

        # plot GPS and odometry
        plot_gps = []
        plot_odo = []
        for s in self.sync_gps_odo:
            plot_gps.append(s.gps)
            plot_odo.append(s.odo)
        # list of drawn trajectories
        trajectories = []
        # plot post-processed trajectory
        if self.post_window is not None:
            plot_post_trajectory = []
            for xyz, ori in self.post_trajectory:
                plot_post_trajectory.append(xyz)
            trajectories.append({
                    "trajectory": plot_post_trajectory,
                    "options": "y.",
                    "label": "post-processing",
                })
        trajectories.extend([
                {
                    "trajectory": plot_gps,
                    "options": "c+",
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
               #{
               #    "trajectory": self.plot_init,
               #    "options": "m.",
               #    "label": "estimation by initial angle & scale",
               #},
               #{
               #    "trajectory": self.plot_xyz_by_encoders,
               #    "options": "r.",
               #    "label": "xyz_by_encoders",
               #},
            ])
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



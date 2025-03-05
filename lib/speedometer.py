import numpy as np
import datetime
import osgar.lib.quaternion as quaternion

class Speedometer:
    """
        Tracks the current speed and distance travelled.

        Designed to be able to distinguish from the odometry measuring whether
            a robot is moving or not.

        Input:
            * `update()` ... adds incremetal distance travelled

        Output:
            * `get_distance_travelled()`
            * `get_speed()`
    """

    def __init__(self, period = 1, threshold = 0.03):
        """
            Args:
                period (float): the length of the time ineterval (in seconds)
                    over which the average velocity (in is_moving()) will be
                    computed
                threshold (float): the border value of the average velocity (is
                    meters per second) to distinguish whether the robot is moving
                    or not; see the method is_moving()
        """
        self.period = period
        self.threshold = threshold
        self.hard_reset()

    def update(self, time, distance):
        """
            Updates the total distance travelled by a new increase.

            Args:
                time (datetime.timedelta): time of the measurement;
                    (Note: time in seconds can be obtained by calling
                    `time.total_seconds()`)
                distance (float): distance travelled in meters
        """
        self.total_distance += distance
        # (atribut history mozna zrusit a jen si pamatovat dve tri posledni
        #   hodnoty)
        self.history.append((time, self.total_distance))

    def reset(self):
        """
            Resets the distance travelled.

            Remember the current distance travelled and subtract it in
                `get_distance_travelled()`.
        """
        self.distance_measured_from = self.total_distance

    def hard_reset(self):
        """
            Force initial settings.
        """
        self.total_distance = 0.0
        self.distance_measured_from = 0.0
        self.history = []

    def get_distance_travelled(self):
        """
            Returns (float): the distance travelled in meters (can be set to
                zero by calling `reset()`)
        """
        return self.total_distance - self.distance_measured_from

    def get_speed(self):
        """
            Returns (float): current speed in meters per second (can be
                negative if reversing);
                `None` if the current speed cannot be determined
        """
        if len(self.history) < 3:
            return None
        else:
            (t_a, s_a) = self.history[-2]
            (t_b, s_b) = self.history[-1]
            #print(t_a, s_a, t_b, s_b)
            return (s_b - s_a) / (t_b.total_seconds() - t_a.total_seconds())

    def compute_average_velocity_over_last_period(self):
        """
            Returns (float): the average velocity over the last time interval
                given by the value of self.period;
                if this cannot be computed (if there is not enough data in the
                time sequence) `None` is returned
        """
        if len(self.history) < 3:
            return None
        (t_start, s_start) = self.history[0]
        (t_end, s_end) = self.history[-1]
        # if the measured time sequence if shorter than self.period, return `None`
        if (t_end - t_start).total_seconds() < self.period:
            return None
        # compute the average velocity
        index = len(self.history) - 1
        while index >= 0:
            (t, s) = self.history[index]
            dt = (t_end - t).total_seconds()
            if dt >= self.period:
                ds = s_end - s
                return ds / dt
            index -= 1
        return None

    def is_moving(self):
        """
            Detects whether the robot is moving, or not.

            The method calculates the average velocity over the last time
                interval; the length of this interval is given by self.period.
            If this average velocity is greater or equal to the value of
                `self.threshold` then the robot is considered to be moving.

            Returns (bool): `True` if the robot is moving at the moment;
                `False` if it is not moving; `None` is the moving state cannot
                be determined
        """
        avg_vel = self.compute_average_velocity_over_last_period()
        if avg_vel is None:
            return None
        elif abs(avg_vel) >= self.threshold:
            return True
        else:
            return False

class Tracker:
    """
        Computes the (x,y,z) position according to the data obtained from
            odometry and IMU.

        Input:

            * `update_orientation()` ... processes new orientation of robot
            * `update_distance()` ... processes new distance travelled by robot

        Output:

            * `get_xyz()` ...  returns current (x,y,z) position of robot
    """

    def __init__(self):
        self.xyz = [0, 0, 0]
        self.orientation = None
        self.speedometer = Speedometer()

    def update_orientation(self, time, orientation):
        """
            Processes new orientation of the robot.

            Args:
                orientation (list of float): orientation of the robot as a
                    quaternion
                time (datetime.timedelta): (absolute) time
                    (Note: time in seconds can be obtained by calling
                    `time.total_seconds()`)
        """
        self.orientation = orientation

    def update_distance(self, time, distance):
        """
            Processes new distance travelled by the robot.

            Args:
                distance(float): distance travelled in meters;
                    may be negative if the robot is reversing
                time (datetime.timedelta): (absolute) time
                    (Note: time in seconds can be obtained by calling
                    `time.total_seconds()`)
        """
        self.speedometer.update(time, distance)
        # compute xyz from odometry and IMU
        if self.orientation != None:
            distance_3d = quaternion.rotate_vector([distance, 0.0, 0.0], self.orientation)
            for i in range(len(self.xyz)):
                self.xyz[i] += distance_3d[i]
            ## compute angle between xyz from imu and xyz from GPS
            ## and rotate xyz from imu
            #xyz_from_imu_rotated = self.kf.rotate_xyz_from_imu(xyz_from_imu, time.total_seconds())
            ## insert xyz from odometry and IMU into Kalman filter
            ## (which works primarily with xyz form GPS)
            #self.kf.input_imu(xyz_from_imu_rotated, time.total_seconds(), 10)
            #time_in_seconds, xyz = self.kf.get_last_xyz()
            #self.last_time = time
            #self.last_xyz = xyz

    def get_xyz(self):
        """
            Returns (list of float): the current (x,y,z) position computed
                according to the data obtained from odometry and IMU.
        """
        return self.xyz

    def is_moving(self):
        """
            Returns (bool): `True` if the robot is moving at the moment;
                `False` if it is not moving; `None` is the moving state cannot
                be determined
        """
        return self.speedometer.is_moving()



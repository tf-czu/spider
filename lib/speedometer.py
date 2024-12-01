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

    def __init__(self):
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

    def is_moving(self):
        """
            Returns (bool): `True` if the robot is moving at the moment;
                `False` if it is not moving; `None` is the moving state cannot
                be determined
        """
        if len(self.history) < 3:
            return None
        (t_start, s_start) = self.history[0]
        (t_end, s_end) = self.history[-1]
        # pokud neubehla alespon jedna vterina, vrat `None`
        if (t_end - t_start).total_seconds() < 1:
            return None
        # spocitej prumernou rychlost za posledni jednu vterinu
        index = len(self.history) - 1
        while index >= 0:
            (t, s) = self.history[index]
            dt = (t_end - t).total_seconds()
            if dt >= 1.0:
                ds = s_end - s
                avg_vel = ds / dt
                if avg_vel >= 0.01: # 1cm per second
                    return True
                else:
                    return False
            index -= 1

        return None

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



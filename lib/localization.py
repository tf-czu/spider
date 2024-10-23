import numpy as np
import osgar.lib.quaternion as quaternion
from lib.kalman import KalmanFilterLocalization

class Localization:
    def __init__(self, remember_history = False):
        """
            Args:
                remember_history (bool): for debugging purposes;
                    if `True`, input values will be stored to the `history`
                    attribute;
                    method `draw()` then can be called to draw a plot
        """
        self.kf = KalmanFilterLocalization()
        if remember_history:
            self.history = {
                    'xyz from gps': [], # kartezske souradnice z GPS
                    'xyz': [], # kartezske souradnice vypocitane Kalmanovym filtrem
                    'pose3d': [], # navratove hodnoty
                }
        else:
            self.history = None
        # posledni poloha spocitana Kalmanovym filtrem
        self.last_xyz = None
        # cas posledni polohy spocitane Kalmanovym filtrem
        self.last_time = None
        # posledni poloha ziskana z odometrie a IMU
        self.last_xyz_from_imu = [0.0, 0.0, 0.0]
        # posledni orientace ziskana z IMU jako kvaternion
        self.last_orientation = None

    def update_xyz_from_gps(self, time, xyz_from_gps, gps_err = None):
        """
            Input new position coordinates obtained from GPS.

            Args:
                time (datetime.timedelta): (absolute) time
                    (time in seconds can be obtained by calling
                    `time.total_seconds()`)
                xyz_from_gps (list of float): position `[x, y, z]` in meters
                    obtained from GPS as its projection into Cartesian
                    coordinate system with two axes tangential to Earth's
                    surface and one axis normal to Earth's surface
                gps_err (list of float): error `[s_x, s_y, s_z]` of
                    `xyz_from_gps` (as standard deviations, in meters)
        """
        self.kf.input(xyz_from_gps, time.total_seconds(), gps_err)
        time_in_seconds, xyz = self.kf.get_last_xyz()
        self.last_time = time
        self.last_xyz = xyz
        if self.history != None:
            self.history['xyz from gps'].append((time, xyz_from_gps))
            self.history['xyz'].append((time, xyz))

    def get_pose3d(self, time = None):
        """
            Returns the last value of `pose3d` or its extrapolation.

            Args:
                time (datetime.timedelta): (absolute) time;
                    if `time == None`:
                        then the last value of `pose3d` is returned,
                    else:
                        the extrapolation of `pose3d` is returned

            Returns (list): list with two items:
                1. list with three floats ... coordinates in meters
                    representing the position of the robot;
                2. list with four floats ... quaternion representing the
                    orientation of the robot;
                    if it cannot be computed, `None` is returned
        """
        # TODO kvaternion neni extrapolovany, ale vraci se posledni hodnota
        # ziskana z IMU
        if time == None:
            result = [self.last_xyz, self.last_orientation]
        else:
            result = [self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation]
        if self.history != None:
            self.history['pose3d'].append((time, result[0]))
        return result


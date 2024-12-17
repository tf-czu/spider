import numpy as np
import math
import osgar.lib.quaternion as quaternion
from lib.kalman import KalmanFilterLocalization
from lib.speedometer import Tracker
from lib.anglescaleestimator import AngleScaleEstimator

class Localization:
    #def __init__(self, gps_err = [2, 2, 6], imu_err = [4, 4, 100]):
    def __init__(self,  gps_err, imu_err):
        # odchylky se mohou v prubehu zpracovani menit metodami set_gps_err() a
        # set_imu_err(); jejich nastaveni je magie
        self.kf = KalmanFilterLocalization()
        # posledni poloha spocitana Kalmanovym filtrem
        self.last_xyz = None
        # posledni orientace ziskana z IMU jako kvaternion
        self.last_orientation = None
        # tracker pocita (x,y,z) polohu z udaju z odometrie a IMU
        self.tracker = Tracker()
        # vyjadruje aktualni stav robota, bud waiting nebo moving
        self.status = "waiting"
        # potreba pro prumerovani gps pozice behem stani
        self.number_waiting_gps_measurements = 0
        # prumerna pozice gps behem stani, na uplnem pocatku pohybu predpokladame, ze
        # je [0.0, 0.0, 0.0] 
        self.average_gps_xyz = [0.0, 0.0, 0.0] 
        #
        self.ase = AngleScaleEstimator(300)
        # DEBUG data
        self.debug_odo_xyz = [] # seznam dvojic (x, y)
        self.debug_odo_xyz_processed = [] # otocene a natahnute odo souradnice (x, y)
        self._gps_err = gps_err 
        self._imu_err = imu_err
        # zde bude ulozena puvodni hodnota imu_err pro ucely jejich uprav,
        # nebude se menit
        self.original_imu_err = imu_err

    def set_gps_err(self, gps_err):
        """
            Sets the error of the GPS position.

            Args:
                gps_err (list of float): standard deviations (s_x, s_y, s_z) of
                    GPS coordinates (x, y, z); in meters
        """
        all_pos = True
        for i in range(3):
            all_pos = all_pos and (gps_err > 0)
        if all_pos:
            self._gps_err = gps_err

    def set_imu_err(self, imu_err):
        """
            Sets the error of the position computed from odometry and IMU.

            Args:
                imu_err (list of float): standard deviations (s_x, s_y, s_z) of
                    coordinates (x, y, z) obtained from odometry and IMU; in
                    meters
        """
        all_pos = True
        for i in range(3):
            all_pos = all_pos and (imu_err > 0)
        if all_pos:
            self._imu_err = imu_err

    def update_xyz_from_gps(self, time, xyz_from_gps):
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

            Removed Args:
                gps_err (list of float): error `[s_x, s_y, s_z]` of
                    `xyz_from_gps` (as standard deviations, in meters)
        """
        if self.status == "waiting":
            # averaging gps measurements while not moving
            if self.number_waiting_gps_measurements == 0:
                self.average_gps_xyz = np.array(xyz_from_gps)
            else:
                self.average_gps_xyz = (self.average_gps_xyz*self.number_waiting_gps_measurements+np.array(xyz_from_gps))/(self.number_waiting_gps_measurements+1)
            self.number_waiting_gps_measurements += 1
            self.last_xyz = self.average_gps_xyz 
        elif self.status == "moving":
            # updating Kalman filter
            self.kf.input(xyz_from_gps, time.total_seconds(), self._gps_err)
            time_in_seconds, xyz = self.kf.get_last_xyz()
            self.last_xyz = xyz
            # updating AngleScaleEstimator using processed data by KF, not just measured by gps
            self.ase.update(self.tracker.get_xyz(), xyz_from_gps)

    def update_orientation(self, time, orientation):
        """
            Input new orientation of the robot.

            Args:
                time (datetime.timedelta): (absolute) time
                orientation (list of float): orientation of the robot
                    represented by a quaternion
        """
        self.last_orientation = orientation
        self.tracker.update_orientation(time, orientation)

    def update_distance(self, time, distance):
        """
            Input distance traveled by the robot.

            Args:
                time (datetime.timedelta): (absolute) time
                distance(float): distance in meters; 
                    can be negative if the robot is reversing
        """
        self.tracker.update_distance(time, distance)
        tracker_xyz = self.tracker.get_xyz()
        # if we cannot say whether the robot si moving or not, nothing happens
        if self.tracker.is_moving() is not None:
            # determining current status
            if self.status == "waiting" and not self.tracker.is_moving():
                status = "waiting"
            if self.status == "waiting" and self.tracker.is_moving():
                status = "settingOff"
            elif self.status == "moving" and self.tracker.is_moving():
                status = "moving"
            elif self.status == "moving" and not self.tracker.is_moving():
                status = "stopping"
            #
            if status == "moving":
                # zde dodelat otoceni a scale IMU pozice a nejak vlozit do
                # Kalmanova filtru
                if self.ase.get_angle() is not None and self.ase.get_scale() is not None:
                    # this works in 2D
                    # rotates by the angle clockwise
                    angle = self.ase.get_angle()
                    matrix_of_rotation = np.array([[math.cos(angle), math.sin(angle)],
                                                   [-math.sin(angle), math.cos(angle)]])
                    rotated_IMU_position = matrix_of_rotation @ tracker_xyz[:2]
                    # scaling
                    scale = self.ase.get_scale()
                    rotated_and_scaled_IMU_position = [0,0]
                    for i in range(2):
                        rotated_and_scaled_IMU_position[i] = rotated_IMU_position[i] / scale  
                    rotated_and_scaled_IMU_position_3D = list(rotated_and_scaled_IMU_position) + [0.0] # TODO tady nema byt `+ [0.0]` !!!
                    self.kf.input(rotated_and_scaled_IMU_position_3D, time.total_seconds(), self._imu_err)
                    self.debug_odo_xyz_processed.append((rotated_and_scaled_IMU_position[0], rotated_and_scaled_IMU_position[1]))
            elif status == "waiting":
                # robot se nehybe 
                pass
            elif status == "settingOff":
                self.status = "moving"
                self.kf = KalmanFilterLocalization()
                # pocet gps mereni behem stani snizuje chybu
                gps_err = [0,0,0]
                for i in range(3):
                    gps_err[i] = self._gps_err[i]/self.number_waiting_gps_measurements
                self.kf.input(self.average_gps_xyz, time.total_seconds(), gps_err)
            elif status == "stopping":
                self.status = "waiting"
                self.average_gps_xyz = self.last_xyz
                self.number_waiting_gps_measurements = 0
        # DEBUG
        self.debug_odo_xyz.append((tracker_xyz[0], tracker_xyz[1]))

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
        # ziskana z IMU
        if time == None:
            result = [self.last_xyz, self.last_orientation]
        else:
            result = [self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation]
        return result


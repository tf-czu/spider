import numpy as np
import math
import osgar.lib.quaternion as quaternion
from lib.kalman import KalmanFilterLocalization
from lib.speedometer import Tracker
from lib.anglescaleestimator import AngleScaleEstimator

class Localization:
    def __init__(self):
        self.kf = KalmanFilterLocalization()
        # posledni poloha spocitana Kalmanovym filtrem
        self.last_xyz = None
        # posledni orientace ziskana z IMU jako kvaternion
        self.last_orientation = None
        # tracker pocita (x,y,z) polohu z udaju z odometrie a IMU
        self.tracker = Tracker()
        # vyjadruje aktualni stav robota, bud waiting nebo moving
        self.status = "waiting"
        #self.status = "moving"
        # potreba pro prumerovani gps pozice behem stani
        self.number_waiting_gps_measurements = 0
        # prumerna pozice gps behem stani, na uplnem pocatku pohybu predpokladame, ze
        # je [0.0, 0.0, 0.0] 
        self.average_gps_xyz = [0.0, 0.0, 0.0] 
        #
        self.ase = AngleScaleEstimator()
        # DEBUG data
        self.debug_odo_xyz = [] # seznam dvojic (x, y)
        self.debug_odo_xyz_processed = [] # otocene a natahnute odo souradnice (x, y)

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
        # TODO NEW CODE DISABLED !!!
        self.kf.input(xyz_from_gps, time.total_seconds(), gps_err)
        time_in_seconds, xyz = self.kf.get_last_xyz()
        self.last_xyz = xyz
        return
        if self.status == "waiting":
            # averaging gps measurements while not moving
            if self.number_waiting_gps_measurements == 0:
                self.average_gps_xyz = np.array(xyz_from_gps)
            else:
                self.average_gps_xyz = (self.average_gps_xyz*self.number_waiting_gps_measurements+np.array(xyz_from_gps))/(self.number_waiting_gps_measurements+1)
            self.number_waiting_gps_measurements += 1
            self.last_xyz = self.average_gps_xyz 
        elif self.status == "moving":
            #print(time.total_seconds(), self.status)
            # updating Kalman filter
            self.kf.input(xyz_from_gps, time.total_seconds(), gps_err)
            time_in_seconds, xyz = self.kf.get_last_xyz()
            self.last_xyz = xyz
            # updating AngleScaleEstimator using processed data by KF, not just measured by gps
            self.ase.update(self.last_xyz, self.tracker.get_xyz())
            #print(self.ase.get_angle())

    def REMOVE__update_xyz_from_imu(self, time, xyz_from_imu, imu_err):
        """
            Input new position coordinates obtained from IMU.

            Args:
                time (datetime.timedelta): (absolute) time
                    (time in seconds can be obtained by calling
                    `time.total_seconds()`)
                xyz_from_imu (list of float): position `[x, y, z]` in meters
                imu_err (list of float): error `[s_x, s_y, s_z]` of
                    `xyz_from_imu and odometry` (as standard deviations, in meters)
        """
        travelled_distance = np.linalg.norm(xyz_from_imu - self.last_xyz_from_imu)
        self.tracker.update(time, travelled_distance)
        # if we cannot say whether the robot si moving or not, nothing happens
        if self.tracker.is_moving() == None:
            return
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
        #print(status)
        if status == "moving":
            # zde dodelat otoceni a scale IMU pozice a nejak vlozit do
            # Kalmanova filtru
            if self.ase.get_angle() != None and self.ase.get_scale() != None:
                # this works in 2D
                # rotates by the angle clockwise
                angle = self.ase.get_angle()
                matrix_of_rotation = np.array([[math.cos(angle), math.sin(angle)],
                                               [-math.sin(angle), math.cos(angle)]])
                rotated_IMU_position = matrix_of_rotation @ self.tracker.get_xyz()[:2]
                # scaling
                scale = self.ase.get_scale()
                rotated_and_scaled_IMU_position = scale * rotated_IMU_position
                rotated_and_scaled_IMU_position_3D = list(rotated_and_scaled_IMU_position) + [0.0] # TODO tady nema byt `+ [0.0]` !!!
                self.kf.input(rotated_and_scaled_IMU_position_3D, time.total_seconds(), imu_err)
        elif status == "waiting":
            # robot se nehybe 
            pass
        elif status == "settingOff":
            self.status = "moving"
            self.kf = KalmanFilterLocalization()
            kf.input(self.average_gps_xyz, time.total_seconds) # TODO otazka, jakou zde vlozit chybu gps
        elif status == "stopping":
            self.status = "waiting"

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
        if self.tracker.is_moving() != None:
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
                if self.ase.get_angle() != None and self.ase.get_scale() != None:
                    # this works in 2D
                    # rotates by the angle clockwise
                    angle = self.ase.get_angle()
                    matrix_of_rotation = np.array([[math.cos(angle), math.sin(angle)],
                                                   [-math.sin(angle), math.cos(angle)]])
                    rotated_IMU_position = matrix_of_rotation @ tracker_xyz[:2]
                    # scaling
                    scale = self.ase.get_scale()
                    rotated_and_scaled_IMU_position = scale * rotated_IMU_position
                    rotated_and_scaled_IMU_position_3D = list(rotated_and_scaled_IMU_position) + [0.0] # TODO tady nema byt `+ [0.0]` !!!
                    #print(' ... ', rotated_and_scaled_IMU_position_3D, time.total_seconds())
                    # TODO program se sem nikdy nedostane !!!
                    self.kf.input(rotated_and_scaled_IMU_position_3D, time.total_seconds(), [1, 1, 1])
            elif status == "waiting":
                # robot se nehybe 
                pass
            elif status == "settingOff":
                self.status = "moving"
                self.kf = KalmanFilterLocalization()
                self.kf.input(self.average_gps_xyz, time.total_seconds(), [1, 1, 1]) # TODO otazka, jakou zde vlozit chybu gps
            elif status == "stopping":
                self.status = "waiting"
                self.average_gps_xyz = [0, 0, 0]
                self.number_waiting_gps_measurements = 0
        # DEBUG
        self.debug_odo_xyz.append((tracker_xyz[0], tracker_xyz[1]))
        #self.debug_odo_xyz_processed.append

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


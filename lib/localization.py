import numpy as np
import math
import osgar.lib.quaternion as quaternion
from lib.kalman import KalmanFilterLocalization
from lib.speedometer import Tracker
from lib.anglescaleestimator import AngleScaleEstimator

class Localization:
    def __init__(self):
        self.kf = KalmanFilterLocalization()
        ## posledni poloha spocitana Kalmanovym filtrem
        self.last_xyz = None
        #self.extrapolated_xyz = None
        #self.extrapolation_start_direction = self.kf.velocity
        ## cas posledni polohy spocitane Kalmanovym filtrem
        #self.last_time = None
        ## posledni poloha ziskana z odometrie a IMU
        #self.last_xyz_from_imu = [0.0, 0.0, 0.0]
        ## posledni orientace ziskana z IMU jako kvaternion
        self.last_orientation = None
        # tracker pocita (x,y,z) polohu z udaju z odometrie a IMU
        self.tracker = Tracker()
        # DEBUG data
        self.debug_odo_xyz = [] # seznam dvojic (x, y)
        self.status = "waiting" # vyjadruje aktualni stav robota, bud waiting nebo moving
        self.number_waiting_gps_measurements = 0 #potreba pro prumerovani gps pozice behem stani
        self.average_gps_xyz = None # prumerna pozice gps behem stani
        self.ase = AngleScaleEstimator()

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
        if self.status == "waiting":
            # averaging gps measurements while not moving
            if self.number_waiting_gps_measurements == 0:
                self.average_gps_xyz = np.array(xyz_from_gps)
            else:
                self.average_gps_xyz = (self.average_gps_xyz*self.number_waiting_gps_measurements+np.array(xyz_from_gps))/(self.number_waiting_gps_measurements+1)
            self.number_waiting_gps_measurements += 1
            self.last_xyz = self.average_gps_xyz 
        elif self.status == "moving":
            #updating Kalman filter
            self.kf.input(xyz_from_gps, time.total_seconds(), gps_err)
            time_in_seconds, xyz = self.kf.get_last_xyz()
            self.last_xyz = xyz
            self.ase.update(self.last_xyz, self.tracker.get_xyz())  #updating AngleScaleEstimator using processed data by KF, not just measured by gps

        ## extrapolace: nastaveni pocatecni polohy
        #self.extrapolated_xyz = np.array(xyz)
        ## extrapolace: nastaveni pocatecniho smeru
        ## TODO nastavit pocatecni smer podle vektoru rychlosti z Kalmanova filtru
        #if self.last_xyz is None:
        #    self.extrapolation_direction = np.array([0, 0, 0])
        #else:
        #    direction_vector = np.array(xyz) - np.array(self.last_xyz)
        #    direction = direction_vector / np.linalg.norm(direction_vector)
        #    self.extrapolation_direction = direction
        #self.last_time = time
        #self.last_xyz = xyz

    def update_xyz_from_imu(self, time, xyz_from_imu, imu_err):
        """
            Input new position coordinates obtained from IMU.

            Args:
                time (datetime.timedelta): (absolute) time
                    (time in seconds can be obtained by calling
                    `time.total_seconds()`)
                xyz_from_gps (list of float): position `[x, y, z]` in meters
                    obtained from GPS as its projection into Cartesian
                    coordinate system with two axes tangential to Earth's
                    surface and one axis normal to Earth's surface
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

        
        if status == "moving":
            # zde dodelat otoceni a scale IMU pozice a nejak vlozit do Kalmanova
            # filtru
            if self.ase.get_angle() != None and self.ase.get_scale() != None:
                # this works in 2D
                angle = self.ase.get_angle()
                matrix_of_rotation = np.array([[math.cos(angle), math.sin(angle)],
                                               [-math.sin(angle), math.cos(angle)]])
                #rotates by the angle clockwise
                rotated_IMU_position = matrix_of_rotation @ self.tracker.get_xyz()[:2]
                scale = self.ase.get_scale()
                #scaling
                rotated_and_scaled_IMU_position = scale * rotated_IMU_position
                rotated_and_scaled_IMU_position_3D = list(rotated_and_scaled_IMU_position) + [0.0] # TODO tady nema byt `+ [0.0]` !!!
                self.kf.input(rotated_and_scaled_IMU_position_3D, time.total_seconds(), imu_err)



        elif status == "waiting":
            # robot se nehybe 
            pass
        elif status == "settingOff":
            self.status = "moving"
            self.kf = KalmanFilterLocalization()
            kf.input(self.average_gps_xyz, time.total_seconds) #otazka, jakou zde vlozit chybu gps
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
        # !!! TODO !!! `distance` je tu v absolutni hodnote !!!
        #self.extrapolated_xyz += abs(distance) * self.extrapolation_direction
        self.tracker.update_distance(time, distance)
        # DEBUG
        tracker_xyz = self.tracker.get_xyz()
        self.debug_odo_xyz.append((tracker_xyz[0], tracker_xyz[1]))




        ## compute xyz from odometry and IMU
        #if self.last_orientation == None:
        #    # with no direction there is no position
        #    pass
        #else:
        #    #print(velocity, len_velocity, direction)
        #    if not self.extrapolation_direction is None:
        #        for i in range(len(self.extrapolation_start_xyz)):
        #            continue
        #            #self.extrapolation_start_xyz[i] += velocity[i] * distance
        #            # !!! TODO !!! absolutni hodnota !!!
        #            #self.extrapolation_start_xyz[i] += velocity[i] * abs(distance) / len_velocity
        #            self.extrapolation_start_xyz[i] += self.extrapolation_direction[i] * abs(distance)
        #            #self.extrapolation_start_xyz[i] += self.extrapolation_direction[i] * (distance)




            #distance_3d = quaternion.rotate_vector([distance, 0.0, 0.0], self.last_orientation)
            #xyz_from_imu = [a + b for a, b in zip(self.last_xyz_from_imu, distance_3d)]
            #self.last_xyz_from_imu = xyz_from_imu
            ## compute angle between xyz from IMU and xyz from GPS
            ## and rotate xyz from IMU
            #xyz_from_imu_rotated = self.kf.rotate_xyz_from_imu(xyz_from_imu, time.total_seconds())
            ## insert xyz from odometry and IMU into Kalman filter
            ## (which works primarily with xyz form GPS)
            #self.kf.input_imu(xyz_from_imu_rotated, time.total_seconds(), 10)
            #time_in_seconds, xyz = self.kf.get_last_xyz()
            #self.last_time = time
            #self.last_xyz = xyz

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
        #if self.extrapolated_xyz is None:
        #    return None
        #else:
        #    return [self.extrapolated_xyz, self.last_orientation]
        # TODO kvaternion neni extrapolovany, ale vraci se posledni hodnota
        # ziskana z IMU
        if time == None:
            result = [self.last_xyz, self.last_orientation]
        else:
            result = [self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation]
        return result


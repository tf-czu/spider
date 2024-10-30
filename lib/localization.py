import numpy as np
import math
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
        self.extrapolated_xyz = None
        self.extrapolation_start_direction = self.kf.velocity
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

        # extrapolace: nastaveni pocatecni polohy
        self.extrapolated_xyz = np.array(xyz)
        # extrapolace: nastaveni pocatecniho smeru
        if self.last_xyz is None:
            self.extrapolation_direction = np.array([0, 0, 0])
        else:
            direction_vector = np.array(xyz) - np.array(self.last_xyz)
            direction = direction_vector / np.linalg.norm(direction_vector)
            self.extrapolation_direction = direction
        
        self.last_time = time
        self.last_xyz = xyz

        if self.history != None:
            self.history['xyz from gps'].append((time, xyz_from_gps))
            self.history['xyz'].append((time, xyz))

    def update_orientation(self, time, orientation):
        """
            Input new orientation of the robot.

            Args:
                time (datetime.timedelta): (absolute) time
                orientation (list of float): orientation of the robot
                    represented by a quaternion
        """
        self.last_orientation = orientation
        
    def update_distance(self, time, distance):
        """
            Input distance traveled by the robot.

            Args:
                time (datetime.timedelta): (absolute) time
                distance(float): distance in meters; 
                    can be negative if the robot is reversing
        """
        # !!! TODO !!! `distance` je tu v absolutni hodnote !!!
        self.extrapolated_xyz += abs(distance) * self.extrapolation_direction



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
        if self.extrapolated_xyz is None:
            return None
        else:
            return [self.extrapolated_xyz, self.last_orientation]
        # TODO kvaternion neni extrapolovany, ale vraci se posledni hodnota
        # ziskana z IMU
        if time == None:
            result = [self.last_xyz, self.last_orientation]
        else:
            result = [self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation]
        if self.history != None:
            self.history['pose3d'].append((time, result[0]))
        return result


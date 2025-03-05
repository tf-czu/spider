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
        self.last_xyz_kf = None
        # posledni poloha podle GPS
        self.last_xyz_gps = None
        # posledni poloha spocitana rotovanou a natahnutou odometrii
        self.last_xyz_odo = None
        # posledni poloha z IMU a odometrie (bez rotovani a natahnuti)
        self.last_xyz_odo_raw = None
        # rozdil mezi startovni odo a gps pro casti, kdy je prenastaveny pocatek
        # AngleScale estimatoru, aby obe hodnoty zacinaly zhruba v [0,0]
        self.diff_odo_gps = [0.0, 0.0, 0.0]
        # posledni orientace ziskana z IMU jako kvaternion
        self.last_orientation = None
        # tracker pocita (x,y,z) polohu z udaju z odometrie a IMU
        self.tracker = Tracker()
        # vyjadruje aktualni stav robota, bud waiting nebo moving
        self.status = "waiting"
        # potreba pro prumerovani gps pozice behem stani
        self.number_waiting_gps_measurements = 1
        # prumerna pozice gps behem stani, na uplnem pocatku pohybu predpokladame, ze
        # je [0.0, 0.0, 0.0] 
        self.average_gps_xyz = [0.0, 0.0, 0.0] 
        #
        self.ase = AngleScaleEstimator(5)
        #maximalni vzdalenost od pocatku AngleScaleEstimatoru, kdyz se zacne
        #zmensovat, prenastavi se pocatek
        self.max_dist_from_origin = 0
        # DEBUG data
        self.debug_odo_xyz = [] # seznam dvojic (x, y)
        self.debug_odo_xyz_processed = [] # otocene a natahnute odo souradnice (x, y)
        self.debug_starting_points = [] # souradnice ase.origin v prubehu jizdy (x, y)
        self._gps_err = gps_err 
        self._imu_err = imu_err
        # zde bude ulozena puvodni hodnota imu_err pro ucely jejich uprav,
        # nebude se menit
        self.original_imu_err = imu_err
        self.first_move = True

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
        self.last_xyz_gps = xyz_from_gps
        if self.status == "waiting":
            # averaging gps measurements while not moving
            if self.number_waiting_gps_measurements == 0:
                self.average_gps_xyz = np.array(xyz_from_gps)
            else:
                self.average_gps_xyz = (self.average_gps_xyz*self.number_waiting_gps_measurements+np.array(xyz_from_gps))/(self.number_waiting_gps_measurements+1)
            self.number_waiting_gps_measurements += 1
            self.last_xyz_kf = self.average_gps_xyz 
        elif self.status == "moving":
            #remembering max distance from origin
            p = self.ase.origin
            q = xyz_from_gps
            distance_from_last_origin = np.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2)
            self.max_dist_from_origin = max(self.max_dist_from_origin, distance_from_last_origin)

            #reseting origin in case of returning to much back
            if self.max_dist_from_origin > distance_from_last_origin + 2:
                self.max_dist_from_origin = 0
                self.kf = KalmanFilterLocalization()
                # pocet gps mereni behem stani snizuje chybu
                #self.kf.input(self.average_gps_xyz, time.total_seconds(), gps_err)
                #print("Resetting AngleScaleEstimator and origin to", xyz_from_gps[:2])
                self.ase.set_origin(self.last_xyz_gps[:2])
                # setting difference of new starting points to shift the IMU
                # info so that both gps and IMU "start at the same point"
                for i in range(3):
                    self.diff_odo_gps[i] = self.last_xyz_gps[i] - self.last_xyz_odo_raw[i]  
                self.debug_starting_points.append((self.ase.origin[0], self.ase.origin[1]))
                #print("Diff", self.diff_odo_gps)

            # updating Kalman filter
            self.kf.input(xyz_from_gps, time.total_seconds(), self._gps_err)
            time_in_seconds, xyz = self.kf.get_last_xyz()
            self.last_xyz_kf = xyz
            # updating AngleScaleEstimator using processed data by KF, not just measured by gps
            tracker_xyz = self.tracker.get_xyz()
            shifted_tracker_xyz = [0.0, 0.0, 0.0]
            for i in range(3):
                shifted_tracker_xyz[i] = tracker_xyz[i] + self.diff_odo_gps[i]
            self.ase.update(xyz_from_gps, shifted_tracker_xyz)
            p = xyz_from_gps
            q = shifted_tracker_xyz
            distance = np.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2)
            #if distance < 1:
            #    print("Ase updating:", xyz_from_gps, shifted_tracker_xyz)

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
        self.last_xyz_odo_raw = tracker_xyz 
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
                    # pokud je jiz prenastaveny pocatek AngleScaleEstimatoru,
                    # musi se souradnice z odometrie posunout tak, aby obe
                    # mereni (odo a gps) zacinaly ve stejnem miste
                    shifted_tracker_xyz = [0.0, 0.0, 0.0]
                    for i in range(3):
                        shifted_tracker_xyz[i] = tracker_xyz[i] + self.diff_odo_gps[i]

                    ## TODO pracuje se pouze s 2D odometrii a pak se k tomu
                    ## prida nulova z-ova souradnice;
                    ## mozna by se mohlo pracovat rovnou s 3D odometrii
                    rotated_and_scaled_IMU_position = self.ase.rotate_and_scale(shifted_tracker_xyz[:2])
                    rotated_and_scaled_IMU_position_3D = list(rotated_and_scaled_IMU_position) + [0.0] # TODO tady nema byt `+ [0.0]` !!!
                    self.kf.input(rotated_and_scaled_IMU_position_3D, time.total_seconds(), self._imu_err)
                    self.last_xyz_odo = rotated_and_scaled_IMU_position_3D
                    self.debug_odo_xyz_processed.append((rotated_and_scaled_IMU_position_3D[0], rotated_and_scaled_IMU_position_3D[1]))
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
                #print()
                #print("Setting off") 
                #print("odo raw:", self.last_xyz_odo_raw) 
                #print("odo:", self.last_xyz_odo) 
                #print("gps:", self.average_gps_xyz) 
                #print("Diff", self.diff_odo_gps)
                p = self.ase.origin
                q = self.average_gps_xyz
                distance_from_last_origin = np.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2)
                #print("distance", distance_from_last_origin)
                # origin is reset only if the new one is far from the old one
                if distance_from_last_origin > 1:
                    #print("Setting off and reseting origin to", self.average_gps_xyz[:2])
                    self.ase.set_origin(self.average_gps_xyz[:2])
                    # setting difference of new starting points to shift the IMU
                    # info so that both gps and IMU "start at the same point"
                    for i in range(3):
                        self.diff_odo_gps[i] = self.average_gps_xyz[i] - self.last_xyz_odo_raw[i]  
                    self.debug_starting_points.append((self.ase.origin[0], self.ase.origin[1]))
                    #print("Diff", self.diff_odo_gps)

        
            elif status == "stopping":
                self.status = "waiting"
                self.average_gps_xyz = self.last_xyz_kf
                self.number_waiting_gps_measurements = 1
                #print()
                #print("Stopping") 
                #print("odo raw:", self.last_xyz_odo_raw)
                #print("odo:", self.last_xyz_odo)
                #print("gps:", self.last_xyz_kf)
                #print("Diff", self.diff_odo_gps)
                

        # DEBUG
        self.debug_odo_xyz.append((tracker_xyz[0], tracker_xyz[1]))

    def get_last_xyz_odo_raw(self):
        return self.last_xyz_odo_raw

    def get_pose3d_kf(self, time = None):
        """
            Returns the position and the orientation of the robot as computed
                by the Kalman filter.

            According to the value of `time`, the returned value is either an
                output of the embedded Kalman filter or an extrapolation.
        
            Args:
                time (datetime.timedelta): (absolute) time;

                    * if time is None:
                        the current position is returned as the scaled and
                            rotated odometry position;
                    * if type(time) is datetime.timedelta):
                        the extrapolation of the position by the Kalman filter
                        is returned;

            Returns (list): list with two items:
                1. list with three floats ... coordinates in meters
                    representing the position of the robot;
                2. list with four floats ... quaternion representing the
                    orientation of the robot;
                    if it cannot be computed, `None` is returned;
                    this value is not extrapolated, the last computed value is
                    returned
        """
        if time is None:
            return [self.last_xyz_kf, self.last_orientation]
        else:
            return [self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation]

    def get_pose3d_odo(self):
        """
            Returns the position and the orientation of the robot as computed
                by IMU, the odometry, and the Kalman filter.

            No `time` parameter here compared to `get_pose3d_kf()`, hence no
                extrapolation.
        
            Returns (list): list with two items:
                1. list with three floats ... coordinates in meters
                    representing the position of the robot;
                2. list with four floats ... quaternion representing the
                    orientation of the robot;
                    if it cannot be computed, `None` is returned;
        """
        return [self.last_xyz_odo, self.last_orientation]

    def get_pose3d(self, time = None):
        # TODO this method is probably obsolete now
        """
            Returns the position and the orientation of the robot.

            According to the value of `time`, the returned value is either an
                output of the embedded Kalman filter or it is the scaled and
                rotated odometry position. 
        
            Args:
                time (datetime.timedelta): (absolute) time;

                    * if time is None:
                        the current position is returned as the scaled and
                            rotated odometry position;
                    * if type(time) is datetime.timedelta):
                        the extrapolation of the position by the Kalman filter
                        is returned;
                    * otherwise (e.g. if time == "now"):
                        the current position by the Kalman filter is returned;

            Returns (list): list with two items:
                1. list with three floats ... coordinates in meters
                    representing the position of the robot;
                2. list with four floats ... quaternion representing the
                    orientation of the robot;
                    if it cannot be computed, `None` is returned
        """
        # ziskana z IMU
        if time is None:
            #elif isinstance(time, datetime.timedelta):
            result = [self.last_xyz_kf, self.last_orientation]
        else:
            result = [self.kf.get_xyz_estimate(time.total_seconds()), self.last_orientation]
        return result


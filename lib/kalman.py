import math
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv

class KalmanFilterLocalization:
    """
        Implementation of Kalman filter with 3D position, velocity, and
            acceleration.

        Input: 3D position
        Inner states: 3D position, 3D velocity, 3D acceleration
        Output: filtered 3D position
    """
    def __init__(self, position = np.zeros(3), start_time = 0, P = np.identity(9)*2):
        """
            Args:
                position (numpy.array): the initial position;
                    numpy.array([0, 0, 0]) by default
                start_time (float): start time in seconds (can be in arbitrary
                    units, though); 0 by default
                P (numpy.array): matrix of variances and covariances;
                    by default: variances are 2 and covariances are 0
        """
        self.position = position # initial position
        self.velocity = np.array([0,0,0]) # initial velocity
        self.acceleration = np.array([0,0,0]) # initial acceleration
        self.number_of_IMU_measurements = 0
        self.time_of_last_update = start_time
        # Matrices used in Kalman filter
        self.P = P
        self.A = np.identity(9)
        self.AT = self.A.transpose()
        self.Q = np.array([[0.1,0,0,0.1,0,0,0.1,0,0],
                           [0,0.1,0,0,0.1,0,0,0.1,0],
                           [0,0,0.1,0,0,0.1,0,0,0.1],
                           [0.1,0,0,0.2,0,0,0.2,0,0],
                           [0,0.1,0,0,0.2,0,0,0.2,0],
                           [0,0,0.1,0,0,0.2,0,0,0.2],
                           [0.1,0,0,0.2,0,0,0.2,0,0],
                           [0,0.1,0,0,0.2,0,0,0.2,0],
                           [0,0,0.1,0,0,0.2,0,0,0.2]])
        self.H = np.array([[1,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,0,0],[0,0,1,0,0,0,0,0,0]])
        self.HT = self.H.transpose()
        # nasledujici filtry jsou pouzity, pokud je vstupem i poloha z IMU;
        # tyto filtry slouzi k odhadu ulhu mezi polohami z GPS a polohami z IMU
        #self.angle_filter = AngleLowPassFilter()
        #self.scale_filter = FloatLowPassFilter()

    def input(self, xyz, time, xyz_err):
        """
            Input of the Kalman filter.

            Args:

                xyz (list of float): position `[x, y, z]` in meters
                time (float): time in seconds (can be in arbitrary units,
                    though, see the constructor)
                xyz_err (list of float): error `[s_x, s_y, s_z]` of `xyz`
                    (as standard deviations, in meters)
        """
        dt = time-self.time_of_last_update
        da = dt*dt/2
        self.A = np.array([[1,0,0,dt,0,0,da,0,0],
                           [0,1,0,0,dt,0,0,da,0],
                           [0,0,1,0,0,dt,0,0,da],
                           [0,0,0,1,0,0,dt,0,0],
                           [0,0,0,0,1,0,0,dt,0], 
                           [0,0,0,0,0,1,0,0,dt], 
                           [0,0,0,0,0,0,1,0,0],
                           [0,0,0,0,0,0,0,1,0],
                           [0,0,0,0,0,0,0,0,1]])
        self.AT = self.A.transpose()
        variances = [e*e for e in xyz_err]
        self.R = np.array([[variances[0],0,0],
                           [0,variances[1],0],
                           [0,0,variances[2]]])
        # we work with nine-dimensional vector composed of three position
        # coordinates, three velocity coordinates and three acceleration
        # coordinates
        estPos = self.A @ np.concatenate((self.position, self.velocity, self.acceleration), axis = None)
        estP = self.A @ self.P @ self.AT + self.Q
        # Update step, if there are more inputs, just change this part according
        # to parametres of the other measurement
        K = estP @ self.HT @ inv(self.H @ estP @ self.HT + self.R)
        res = estPos + K@((xyz-self.H @ estPos).transpose())
        self.P = estP - K @ self.H @ estP
        # saving current position, velocity and acceleration
        self.position = res[:3]
        self.velocity = res[3:6]
        self.acceleration = res[6:]
        self.time_of_last_update = time

    def get_last_xyz(self):
        """
            Returns (float, [float, float, float]): time in seconds,
                coordinates of the last computed position
        """
        return (self.time_of_last_update, self.position)

    def get_xyz_estimate(self, time):
        """
            Extrapolates the position vector in the given (future) time.

            The extrapolation is computed from the last position, velocity, and
                acceleration vector.

            Args:
                time (float): time (in future) in seconds (can be in arbitrary
                    units, though, see the constructor)

            Returns (numpy.array): estimated (extrapolated) coordinates
                `[x, y, z]` of the position at the given time
        """
        dt = time - self.time_of_last_update
        # original version, did not work with acceleration
        #return self.position + (dt)*self.velocity
        return self.position + self.velocity*dt + self.acceleration*dt*dt/2

    def get_velocity_estimate(self, time):
        """
            Extrapolates the velocity vector in the given (future) time.

            The extrapolation is computed from the last velocity and
                acceleration vector.

            Args:
                time (float): time (in future) in seconds (can be in arbitrary
                    units, though, see the constructor)

            Returns (numpy.array): estimated (extrapolated) coordinates
                `[v_x, v_y, v_z]` of the velocity at the given time
        """
        dt = time - self.time_of_last_update
        return self.velocity + self.acceleration*dt

# 21.10.2024
# Zbytek kodu je zakomentovany.
# Byl pouzivany, dokud jsme jako vstup Kalmanova filtru pouzivali i polohu z IMU.
# Ted je kod zhednoduseny tak, ze se jako vstup uvazuje pouze poloha z GPS.
# Az k tomu zase pridame i polohu z IMU, tak tento kod nejspis znovu pouzijeme.

#    # TODO sloucit input_imu a rotate_xyz_from_imu
#    def input_imu(self, xyz, time, variance):
#        #print('input_IMU_measurement:', measurement, time, variance, self.angle_filter.has_converged())
#        if not (xyz is None) and self.angle_filter.has_converged():
#            # TODO kdyz sem dam maly nasobek variance, tak to vytvari uplne absurdni grafy
#            # TODO kdyz tam dam 100 tak to vypada dobre
#            self.input(xyz, time, variance * 2)
#            pass
#
#    def rotate_xyz_from_imu(self, xyz, time):
#        # TODO opravit tak, aby vracela tri souradnice !!!
#        """
#            Rotates the position computed from odometry and IMU such that its
#                angle corresponds with the angle of the extrapolated position
#                from GPS.
#
#            As a byproduct updates estimate of an angle between positions from
#                IMU and GPS. 
#
#            Args:
#                xyz (list of float): position as x, y, z coordinates
#                time (float): time of measurement
#
#            Returns (numpy.array): rotated position that was previously
#                computed from odometry and IMU;
#                if the rotation is not valid (if the rotation angle cannot be
#                computed or if it has not converged yet), `None` is returned
#        """
#        gps_estimate = self.get_xyz_estimate(time)[:2]
#        current_angle = calculate_angle(gps_estimate, xyz[:2])
#        if current_angle == None:
#            return None
#        else:
#            self.angle_filter.update(current_angle)
#            if self.angle_filter.has_converged():
#                matrix_of_rotation = np.array([[math.cos(self.angle_filter.get()), math.sin(self.angle_filter.get())],
#                                               [-math.sin(self.angle_filter.get()), math.cos(self.angle_filter.get())]])
#                #rotates by the angle clockwise
#                rotated_IMU_position = matrix_of_rotation @ xyz[:2]
#                current_scale = np.linalg.norm(gps_estimate) / np.linalg.norm(rotated_IMU_position)
#                self.scale_filter.update(current_scale)
#                if self.scale_filter.has_converged():
#                    scale = self.scale_filter.get()
#                else:
#                    scale = 1   # scale from scale_filter is not usable
#                rotated_and_scaled_IMU_position = scale * rotated_IMU_position
#                #return rotated_and_scaled_IMU_position
#                return list(rotated_and_scaled_IMU_position) + [0.0] # TODO tady nema byt `+ [0.0]` !!!
#            else:
#                return None
#
#    # TODO this method is probably obsolete and can be deleted
#    # this allows to run Kalman filter on given two sensors data
#    def OBSOLETE_run_on_data_from_two_inputs(self, primary_measurements, primary_times,
#            primary_variance, secondary_measurements, secondary_times, secondary_variance):
#        number_1 = primary_measurements.shape[1] #number of primary measurements
#        number_2 = secondary_measurements.shape[1] #number of secondary measurements
#        number = number_1 + number_2
#        res = np.zeros((3, number))
#        res[:,0] = primary_measurements[:,0] # we assume that the first primary measurement is at time 0
#        primary_index = 1
#        secondary_index = 0
#        #print(measurements)
#        self.position = primary_measurements[:, 0].transpose()
#        #print(pos.shape())
#        while primary_index < number_1 or secondary_index < number_2:
#            if primary_index < number_1 and primary_times[primary_index] <= secondary_times[secondary_index]:
#                self.input(primary_measurements[:, primary_index].transpose(), primary_times[primary_index], primary_variance)
#                index = primary_index + secondary_index
#                #print(index)
#                res[:,index] = self.position
#                primary_index += 1
#                #print(self.velocity)
#                #print(self.orientation())
#                #print(self.position_estimate(times[0,i])+1)
#            else:
#                self.input(secondary_measurements[:, secondary_index].transpose(), secondary_times[secondary_index], secondary_variance)
#                index = primary_index + secondary_index
#                #print(index)
#                res[:,index] = self.position
#                secondary_index += 1
#        #print(number_1, number_2)
#        #print(res.shape)
#        return res
#
#    # TODO this method is probably obsolete and can be deleted
#    def OBSOLETE_get_orientation(self):
#        if self.velocity.any() != 0:
#            return self.velocity / np.linalg.norm(self.velocity)
#        else:
#            return 0
#
#def calculate_angle(a, b):
#    """
#        Calculates the angle between two 2D vectors.
#
#        Warning by HH: this function is not continuous, in the case of
#            angles close to pi, it may not work properly
#
#        Args:
#            a (numpy.array): coordinates of a vector;
#                only the first two coordinates are involved
#            b (numpy.array): coordinates of a vector;
#                only the first two coordinates are involved
#
#        Returns (float): the angle between `a` and `b` in radians as a
#            value from the interval (-pi, pi]
#    """
#    a_norm = np.linalg.norm(a[:2])
#    b_norm = np.linalg.norm(b[:2])
#    if abs(a_norm) < 0.1 or abs(b_norm) < 0.1:
#        # refuse to calculate the angle if the vectors are too short
#        return None
#    else:
#        # calculate the angle utilizing the dot-product formula
#        ab = a[0]*b[0] + a[1]*b[1]
#        cos_value = ab / (a_norm * b_norm)
#        angle = np.arccos(cos_value)
#        # arccos gives angle between 0 and pi, but we need the angle with
#        # orientation, which is given by the determinant:
#        if a[0]*b[1] - a[1]*b[0] < 0 and angle < np.pi:
#            angle = -angle
#        return angle
#
#class FloatLowPassFilter:
#    """
#        A common low-pass filter of a series of floats used to filter scale
#            ratio between the positions given by the odometry and IMU and the
#            positions given by GPS.
#
#        Args:
#            history_max_len (int): from how many last values it will be
#                computed whether the value has stabilized (converged);
#                default 100
#            self.lower_convergence_limit (int): default 0.01
#            self.upper_convergence_limit (int): default 0.02
#    """
#    def __init__(self,
#                 history_max_len = 100,
#                 lower_convergence_limit = 0.01,
#                 upper_convergence_limit = 0.02):
#        self.value = None  
#        self.number_of_inputs = 0
#        # the following 5 variables serve to determine whether the value of the
#        # scale given by the low-pass filter has converged;
#        # see `update_scale()`, `has_converged()`
#        self.history = [] # FIFO of the last n computed scales
#        self.history_max_len = history_max_len
#        self.converged = False
#        self.lower_convergence_limit = lower_convergence_limit
#        self.upper_convergence_limit = upper_convergence_limit
#
#    def update(self, new_value):
#        """
#            Insert a new value to the low-pass filter series.
#
#            The value, resulting from the low-pass filter, is then obtained by
#                the method `get`.
#
#            Args:
#                new_value (float): the new value in the series
#        """
#        if self.value == None:
#            self.value = new_value
#        else:
#            weight_of_new_value = max(0.1, 1/(self.number_of_inputs + 1))
#            self.value = weight_of_new_value*new_value + (1 - weight_of_new_value)*self.value
#        self.number_of_inputs += 1
#        self.history.append(self.value)
#        if len(self.history) > self.history_max_len:
#            self.history.pop(0) # remove the oldest element
#
#    def get(self):
#        """
#            Returns (float): the filtered value
#        """
#        return self.value
#
#    def get_convergence_norm(self):
#        """
#            Computes the measure of stabilization of the resulting value.
#
#            Returns (float): The more the returned vaued is close to zero, the
#                more the resulting value is stabilized.
#                The returned norm is computed from the last `history_max_len`
#                values in the history.
#        """
#        if len(self.history) < self.history_max_len:
#            return np.inf
#        else:
#            last_value = self.history[-1]
#            weight = 0
#            for value in self.history:
#                weight += (value - last_value)**2
#            weight /= self.history_max_len
#            weight = math.sqrt(weight)
#            return weight
#
#    def has_converged(self):
#        """
#            Return (bool): `True` if the resulting value has "stablized enough"
#        """
#        norm = self.get_convergence_norm()
#        if  norm > self.upper_convergence_limit:
#            self.converged = False
#            return False
#        if norm < self.lower_convergence_limit:
#            self.converged = True
#            return True
#        if self.lower_convergence_limit <= norm <= self.upper_convergence_limit:
#            return self.converged
#
#class AngleLowPassFilter:
#    """
#        A low-pass filter of a series of angles in radians.
#
#        There is an issue as angle values are periodic (e.g. 400 degrees is the
#            same value as 40 degrees).
#        Therefore, in the code:
#            1.  the angles are first tanslated to complex numbers,
#            2.  then an average (a convex combination) is computed from the
#                complex numbers,
#            3.  finally, the resultin complex number is converted back to its
#                 angle.
#
#        Args:
#            history_max_len (int): from how many last values it will be
#                computed whether the value has stabilized (converged);
#                default 100
#            self.lower_convergence_limit (int): default 0.01
#            self.upper_convergence_limit (int): default 0.02
#    """
#    def __init__(self,
#                 history_max_len = 100,
#                 lower_convergence_limit = 0.01,
#                 upper_convergence_limit = 0.02):
#        self.angle = None  
#        self.number_of_inputs = 0
#        # the following 5 variables serve to determine whether the value of the
#        # angle given by the low-pass filter has converged;
#        # see `update_angle()`, `has_converged()`
#        self.history = [] # FIFO of the last n computed angles
#        self.history_max_len = history_max_len
#        self.converged = False
#        self.lower_convergence_limit = lower_convergence_limit
#        self.upper_convergence_limit = upper_convergence_limit
#
#    def update(self, new_angle):
#        """
#            Insert a new angle value to the low-pass filter series.
#
#            The angle, resulting from the low-pass filter, is then obtained by
#                the method `get`.
#
#            Args:
#                new_angle (float): the angle in radians
#        """
#        if self.angle == None:
#            self.angle = new_angle
#        else:
#            weight_of_new_angle = max(0.1, 1/(self.number_of_inputs+1))
#            self_point = np.array([math.cos(self.angle), math.sin(self.angle)])
#            new_angle_point = np.array([math.cos(new_angle), math.sin(new_angle)])
#            # new point is a weighted average of two points of absolute value 1,
#            # afterwards its angle is calculated
#            new_point = self_point*(1-weight_of_new_angle) + new_angle_point*weight_of_new_angle
#            point_as_complex_number = np.complex128(new_point[0]+new_point[1]*1j)
#            self.angle = np.angle(point_as_complex_number) 
#        self.number_of_inputs += 1
#        self.history.append(self.angle)
#        if len(self.history) > self.history_max_len:
#            self.history.pop(0) # remove the oldest element
#
#    def get(self):
#        """
#            Returns (float): the angle in radians as the current result of the
#                low-pass filter.
#        """
#        return self.angle
#
#    def get_convergence_norm(self):
#        """
#            Computes the measure of stabilization of the resulting angle.
#
#            Returns (float): The more the returned vaued is close to zero, the
#                more the resulting angle is stabilized.
#                The returned value is computed from the last `history_max_len`
#                values in the history.
#        """
#        if len(self.history) < self.history_max_len:
#            return np.inf
#        else:
#            last_value = self.history[-1]
#            weight = 0
#            for value in self.history:
#                weight += (value - last_value)**2
#            weight /= self.history_max_len
#            weight = math.sqrt(weight)
#            return weight
#
#    def has_converged(self):
#        """
#            Return (bool): `True` if the resulting angle has "stablized
#                enough", or if it is "unstable just a little".
#        """
#        norm = self.get_convergence_norm()
#        if  norm > self.upper_convergence_limit:
#            self.converged = False
#            return False
#        if norm < self.lower_convergence_limit:
#            self.converged = True
#            return True
#        if self.lower_convergence_limit <= norm <= self.upper_convergence_limit:
#            return self.converged

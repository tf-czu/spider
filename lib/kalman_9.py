import math
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from angle_low_pass_filter import AngleLowPassFilter
from float_low_pass_filter import FloatLowPassFilter

class Acc3DKalmanFilter:
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
                start_time (float): can be in arbitrary units, however, we do
                    it in seconds
                P (numpy.array): matrix of variances and covariances;
                    by default, variances are 2 and covariances are 0
        """
        # First three coordinates correspond to x,y,z positions,
        #   second three to x,y,z velocities
        self.position = position
        # !!! MP (24/09/25): Zmenil jsem `velocity` a `acceleration` z
        #       `[[0,0,0]]` na `[0,0,0]`.
        #   Pri vypisu hodnot `velocity` se totiz zpocatku stridave vyskytovaly
        #       hodnoty `[[0,0,0]]` a `[0,0,0]`, coz jednak vypadalo divne a
        #       jednak to delalo problemy v metode `get_position_estimate()`,
        #       kde se pokazde vracel jiny typ vysledku a take v metode
        #       `calculate_angle()`, kde se zase za behu menil typ vstupnich
        #       parametru.
        #   Zda se, ze to ted funguje spravne.
        #self.velocity = np.array([[0,0,0]]) # initial velocity
        #self.acceleration = np.array([[0,0,0]]) # initial acceleration
        self.velocity = np.array([0,0,0]) # initial velocity
        self.acceleration = np.array([0,0,0]) # initial acceleration
        self.number_of_IMU_measurements = 0
        self.time_of_last_update = start_time
        # Matrices used in Kalman filter
        self.P = P
        #self.v = 2 # variance of the primary sensor
        #self.secondary_v = 8 # variance of the secondary sensor
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
                           [0,0,0.1,0,0,0.2,0,0,0.2] ])
        self.H = np.array([[1,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,0,0],[0,0,1,0,0,0,0,0,0]])
        self.HT = self.H.transpose()
        self.angle_filter = AngleLowPassFilter()
        self.scale_filter = FloatLowPassFilter()

    def input_measurement(self, measurement, time, variance):
        """
            Input of the Kalman filter.

            Args:
                measurement (list of float): position as `[x, y, z]`
                    coordinates
                time (float): can be in arbitrary units, however, we do it in
                    seconds
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
        self.R = np.array([[variance*variance,0,0],
                           [0,variance*variance,0],
                           [0,0,variance*variance]])
        # we work with nine dimensional vector composed of three position
        # coordinates, three velocity coordinates and three acceleration
        # coordinates
        estPos = self.A @ np.concatenate((self.position, self.velocity, self.acceleration), axis = None)
        estP = self.A @ self.P @ self.AT + self.Q
        # Update step, if there are more inputs, just change this part according
        # to parametres of the other measurement
        K = estP @ self.HT @ inv(self.H @ estP @ self.HT + self.R)
        res = estPos + K@((measurement-self.H @ estPos).transpose())
        self.P = estP - K @ self.H @ estP
        # saving current position, velocity and acceleration
        self.position = res[:3]
        self.velocity = res[3:6]
        self.acceleration = res[6:]
        self.time_of_last_update = time

    def input_IMU_measurement(self, measurement, time, variance):
        #print('input_IMU_measurement:', measurement, time, variance, self.angle_filter.has_converged())
        if self.angle_filter.has_converged():
            # TODO kdyz sem dam maly nasobek variance, tak to vytvari uplne absurdni grafy
            # TODO kdyz tam dam 100 tak to vypada dobre
            #self.input_measurement(measurement, time, variance * 2)
            pass

    def get_position_estimate(self, time):
        """
            Extrapolates the position in the given (future) time.

            The extrapolation is given by the last position and the last
                velocity.
            Hence, acceleration is not involved.

            Args:
                time (float): time in future

            Returns (numpy.array): estimated (extrapolated) coordinates of the position at
                the given time
        """
        dt = time - self.time_of_last_update
        # original version, does not work with acceleration
        #return self.position + (dt)*self.velocity
        return self.position + self.velocity*dt + self.acceleration*dt*dt/2

    def rotate_IMU_position(self, xyz, time):
        """
            Rotates the position computed from odometry and IMU such that its
                angle corresponds with the angle of the extrapolated position
                from GPS.

            As a byproduct updates estimate of an angle between positions from
                IMU and GPS. 

            Args:
                xyz (list of float): position as x, y, z coordinates
                time (float): time of measurement

            Returns (numpy.array): rotated position that was previously
                computed from odometry and IMU;
                if the rotation is not valid (if the rotation angle cannot be
                computed or if it has not converged yet), `None` is returned
        """
        gps_estimate = self.get_position_estimate(time)[:2]
        current_angle = calculate_angle(gps_estimate, xyz[:2])
        if current_angle == None:
            return None
        else:
            self.angle_filter.update(current_angle)
            if self.angle_filter.has_converged():
                matrix_of_rotation = np.array([[math.cos(self.angle_filter.get()), math.sin(self.angle_filter.get())],
                                               [-math.sin(self.angle_filter.get()), math.cos(self.angle_filter.get())]])
                #rotates by the angle clockwise
                rotated_IMU_position = matrix_of_rotation @ xyz[:2]
                current_scale = np.linalg.norm(gps_estimate) / np.linalg.norm(rotated_IMU_position)
                self.scale_filter.update(current_scale)
                if self.scale_filter.has_converged():
                    scale = self.scale_filter.get()
                else:
                    scale = 1   # scale from scale_filter is not usable
                rotated_and_scaled_IMU_position = scale * rotated_IMU_position
                return rotated_and_scaled_IMU_position
            else:
                return None

    # TODO this method is probably obsolete and can be deleted
    # this allows to run Kalman filter on given two sensors data
    def run_on_data_from_two_inputs(self, primary_measurements, primary_times,
            primary_variance, secondary_measurements, secondary_times, secondary_variance):
        number_1 = primary_measurements.shape[1] #number of primary measurements
        number_2 = secondary_measurements.shape[1] #number of secondary measurements
        number = number_1 + number_2
        res = np.zeros((3, number))
        res[:,0] = primary_measurements[:,0] # we assume that the first primary measurement is at time 0
        primary_index = 1
        secondary_index = 0
        #print(measurements)
        self.position = primary_measurements[:, 0].transpose()
        #print(pos.shape())
        while primary_index < number_1 or secondary_index < number_2:
            if primary_index < number_1 and primary_times[primary_index] <= secondary_times[secondary_index]:
                self.input_measurement(primary_measurements[:, primary_index].transpose(), primary_times[primary_index], primary_variance)
                index = primary_index + secondary_index
                #print(index)
                res[:,index] = self.position
                primary_index += 1
                #print(self.velocity)
                #print(self.orientation())
                #print(self.position_estimate(times[0,i])+1)
            else:
                self.input_measurement(secondary_measurements[:, secondary_index].transpose(), secondary_times[secondary_index], secondary_variance)
                index = primary_index + secondary_index
                #print(index)
                res[:,index] = self.position
                secondary_index += 1
        #print(number_1, number_2)
        #print(res.shape)
        return res

    # TODO this method is probably obsolete and can be deleted
    def get_orientation(self):
        if self.velocity.any() != 0:
            return self.velocity / np.linalg.norm(self.velocity)
        else:
            return 0

def calculate_angle(a, b):
    """
        Calculates the angle between two 2D vectors.

        Warning by HH: this function is not continuous, in the case of
            angles close to pi, it may not work properly

        Args:
            a (numpy.array): coordinates of a vector;
                only the first two coordinates are involved
            b (numpy.array): coordinates of a vector;
                only the first two coordinates are involved

        Returns (float): the angle between `a` and `b` in radians as a
            value from the interval (-pi, pi]
    """
    a_norm = np.linalg.norm(a[:2])
    b_norm = np.linalg.norm(b[:2])
    if abs(a_norm) < 0.1 or abs(b_norm) < 0.1:
        # refuse to calculate the angle if the vectors are too short
        return None
    else:
        # calculate the angle utilizing the dot-product formula
        ab = a[0]*b[0] + a[1]*b[1]
        cos_value = ab / (a_norm * b_norm)
        angle = np.arccos(cos_value)
        # arccos gives angle between 0 and pi, but we need the angle with
        # orientation, which is given by the determinant:
        if a[0]*b[1] - a[1]*b[0] < 0 and angle < np.pi:
            angle = -angle
        return angle

    def OBSOLETE__compute_angle_and_scale_for_odo_to_gps(self):
        gps = self.history[ 'gps planar filtered' ]
        odo = self.history[ 'position from IMU filtered' ]
        # posunu indexy tak, aby platilo: odo[j-1]['time'] <= gps[i] <= odo[j]['time']
        # nebo: gps[i-1]['time'] <= odo[j] <= gps[i]['time']
        i = j = 0
        gps_time, gps_pos = gps[i]
        odo_time, odo_pos = odo[j]
        if gps_time < odo_time:
            while gps_time < odo_time:
                i += 1
                gps_time, gps_pos = gps[i]
        elif odo_time < gps_time:
            while odo_time < odo_time:
                j += 1
                odo_time, odo_pos = odo[j]
        else:
            i += 1
            j += 1
        # posouvam indexy dale, dokud se v gps a odo neobjevi nenulove hodnoty
        while True:
            gps_time, gps_pos = gps[i]
            odo_time, odo_pos = odo[j]
            if any(abs(crd) > 0.0001 for crd in gps_pos) and any(abs(crd) > 0.0001 for crd in odo_pos):
                break
            if gps_time < odo_time:
                i += 1
            elif odo_time < gps_time:
                j += 1
            else:
                i += 1
                j += 1
        # vypocet uhlu mezi gps a odo
        # chceme otocit a natahnout odo tak, aby vysledek byl co nejblize gps
        counter = 0
        angle_sin_sum = 0.0
        angle_cos_sum = 0.0
        scale_upper_sum = 0.0
        scale_lower_sum = 0.0
        len_gps = len(gps)
        len_odo = len(odo)
        while i < len_gps and j < len_odo:
            gps_time, gps_pos = gps[i]
            odo_time, odo_pos = odo[j]
            if gps_time < odo_time:
                odo_time_prev, odo_pos_prev = odo[j - 1]
                gps_pos_res = gps_pos
                odo_pos_res = [ interpolate(gps_time.total_seconds(), odo_time_prev.total_seconds(), odo_pos_prev[k], odo_time.total_seconds(), odo_pos[k]) for k in range(2) ]
                i += 1
            elif odo_time < gps_time:
                gps_time_prev, gps_pos_prev = gps[j - 1]
                gps_pos_res = [ interpolate(odo_time.total_seconds(), gps_time_prev.total_seconds(), gps_pos_prev[k], gps_time.total_seconds(), gps_pos[k]) for k in range(2) ]
                odo_pos_res = odo_pos
                j += 1
            else:
                gps_pos_res = gps_pos
                odo_pos_res = odo_pos
                i += 1
                j += 1
            # prevod gps a odo na komplexni cisla
            # zjisteni jejich uhlu a absolutnich hodnot
            gps_complex = complex( gps_pos_res[0], gps_pos_res[1] )
            odo_complex = complex( odo_pos_res[0], odo_pos_res[1] )
            gps_abs = np.absolute(gps_complex)
            odo_abs = np.absolute(odo_complex)
            gps_ang = np.angle( gps_complex )
            odo_ang = np.angle( odo_complex )

            #angle_sin_sum += np.sin( (odo_ang - gps_ang) / 2 )
            #angle_cos_sum += np.cos( (odo_ang - gps_ang) / 2 )
            angle_sin_sum += np.sin( (odo_ang - gps_ang) / 2 )
            angle_cos_sum += np.cos( (odo_ang - gps_ang) / 2 )

            scale_upper_sum += odo_abs * gps_abs
            scale_lower_sum += odo_abs * odo_abs

        scale = scale_upper_sum / scale_lower_sum

        cos_sqr_angle_half = angle_sin_sum**2 / (angle_sin_sum**2 + angle_cos_sum**2)
        #angle = 2 * np.arccos( np.sqrt( cos_sqr_angle_half ) )
        angle = 2 * np.arccos( -np.sqrt( cos_sqr_angle_half ) )

        return angle, scale

    def OBSOLETE__get_odo_adjusted_to_gps(self):
        angle, scale = self.compute_angle_and_scale_for_odo_to_gps()
        adjuster = scale * np.exp( 1j * angle )
        #print('angle:', angle, 'scale:', scale, 'adjuster:', adjuster)
        odo_adjusted = []
        for odo_time, odo_pos in self.history[ 'position from IMU filtered' ]:
            odo_pos_complex = odo_pos[0] + 1j * odo_pos[1]
            odo_pos_adjusted_complex = odo_pos_complex * adjuster
            #print(odo_pos, ' * ', adjuster, ' = ', odo_pos_adjusted_complex)
            odo_pos_adjusted = [ odo_pos_adjusted_complex.real, odo_pos_adjusted_complex.imag ]
            #print(odo_pos, '...', odo_pos_adjusted)
            odo_adjusted.append( (odo_time, odo_pos_adjusted) )
        return odo_adjusted

    def OBSOLETE__get_gps_adjusted_to_odo(self):
        angle, scale = self.compute_angle_and_scale_for_odo_to_gps()
        adjuster = scale * np.exp( 1j * angle )
        #print('angle:', angle, 'scale:', scale, 'adjuster:', adjuster)
        gps_adjusted = []
        for gps_time, gps_pos in self.history[ 'gps planar filtered' ]:
            gps_pos_complex = gps_pos[0] + 1j * gps_pos[1]
            gps_pos_adjusted_complex = gps_pos_complex * adjuster
            #print(gps_pos, ' * ', adjuster, ' = ', gps_pos_adjusted_complex)
            gps_pos_adjusted = [ gps_pos_adjusted_complex.real, gps_pos_adjusted_complex.imag ]
            #print(gps_pos, '...', gps_pos_adjusted)
            gps_adjusted.append( (gps_time, gps_pos_adjusted) )
        return gps_adjusted


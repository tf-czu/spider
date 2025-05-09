import math
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv

class KalmanFilterLocalization:
    """
        Implementation of Kalman filter with 3D position, velocity, and
            acceleration.

        Utilized by TrackerKalman in tracker_kalman.py.

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

    def input(self, xyz, seconds, xyz_err):
        """
            Input of the Kalman filter.

            Args:

                xyz (list of float): position `[x, y, z]` in meters
                seconds (float): time in seconds (can be in arbitrary units,
                    though, see the constructor)
                xyz_err (list of float): error `[s_x, s_y, s_z]` of `xyz`
                    (as standard deviations, in meters)
        """
        dt = seconds-self.time_of_last_update
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
        self.time_of_last_update = seconds

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


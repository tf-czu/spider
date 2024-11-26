import math
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv

class KalmanFilter1D:
    """
        Implementation of Kalman filter with 1D position, velocity, and
            acceleration.

        Input: 1D position
        Inner states: 1D position, 1D velocity, 1D acceleration
        Output: filtered 1D position
    """
    def __init__(self, position = np.zeros(1), start_time = 0, P = np.identity(3)*2):
        """
            Args:
                position (numpy.array): the initial position;
                    numpy.array([0]) by default
                start_time (float): start time in seconds (can be in arbitrary
                    units, though); 0 by default
                P (numpy.array): matrix of variances and covariances;
                    by default: variances are 2 and covariances are 0
        """
        self.position = position # initial position
        self.velocity = np.array([0]) # initial velocity
        self.acceleration = np.array([0]) # initial acceleration
        self.number_of_IMU_measurements = 0
        self.time_of_last_update = start_time
        # Matrices used in Kalman filter
        self.P = P
        self.A = np.identity(3)
        self.AT = self.A.transpose()
        self.Q = np.array([[0.1,0.1,0.1],
                           [0.1,0.2,0.2],
                           [0.1,0.2,0.2]])
        self.H = np.array([[1,0,0]])
        self.HT = self.H.transpose()

    def input(self, x, time, x_err):
        """
            Input of the Kalman filter.

            Args:

                x (float): position `x` in meters
                time (float): time in seconds (can be in arbitrary units,
                    though, see the constructor)
                x_err (float): error `s_x` of `x`
                    (as standard deviations, in meters)
        """
        dt = time-self.time_of_last_update
        da = dt*dt/2
        self.A = np.array([[1,dt,da],
                           [0,1,dt],
                           [0,0,1]])
        self.AT = self.A.transpose()
        variance = x_err**2
        self.R = np.array([[variance]])
        # we work with three-dimensional vector composed of one position
        # coordinate, one velocity coordinate and one acceleration
        # coordinate
        estPos = self.A @ np.array([self.position, self.velocity, self.acceleration])
        estP = self.A @ self.P @ self.AT + self.Q
        # Update step, if there are more inputs, just change this part according
        # to parametres of the other measurement
        M = inv(self.H @ estP @ self.HT + self.R)
        K = (estP @ self.HT) @ inv(self.H @ estP @ self.HT + self.R)
        res = estPos + K@((x-self.H @ estPos).transpose())
        self.P = estP - K @ self.H @ estP
        # saving current position, velocity and acceleration
        self.position = res[0]
        self.velocity = res[1]
        self.acceleration = res[2]
        self.time_of_last_update = time

    def get_last_x(self):
        """
            Returns (float, float): time in seconds,
                coordinate of the last computed position
        """
        return (self.time_of_last_update, self.position)

    def get_x_estimate(self, time):
        """
            Extrapolates the position in the given (future) time.

            The extrapolation is computed from the last position, velocity, and
                acceleration.

            Args:
                time (float): time (in future) in seconds (can be in arbitrary
                    units, though, see the constructor)

            Returns (float): estimated (extrapolated) coordinate
                `x` of the position at the given time
        """
        dt = time - self.time_of_last_update
        return self.position + self.velocity*dt + self.acceleration*dt*dt/2

    def get_velocity_estimate(self, time):
        """
            Extrapolates the velocity in the given (future) time.

            The extrapolation is computed from the last velocity and
                acceleration.

            Args:
                time (float): time (in future) in seconds (can be in arbitrary
                    units, though, see the constructor)

            Returns (float): estimated (extrapolated) coordinate
                `v_x` of the velocity at the given time
        """
        dt = time - self.time_of_last_update
        return self.velocity + self.acceleration*dt


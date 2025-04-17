import numpy as np
import datetime



class KalmanFilter:
    """
        Implementation of Kalman filter with 3D position, velocity, and
            acceleration.

        Input: 3D position
        Inner states: 3D position, 3D velocity, 3D acceleration
        Output: filtered 3D position
    """
    #def __init__(self, position = np.zeros(3), start_time = 0, P = np.identity(9)*2):
    def __init__(self):
        """
            Args:
                position (numpy.array): the initial position;
                    numpy.array([0, 0, 0]) by default
                start_time (float): start time in seconds (can be in arbitrary
                    units, though); 0 by default
                P (numpy.array): matrix of variances and covariances;
                    by default: variances are 2 and covariances are 0
        """
        P = np.identity(3)*2 # matrix of variances and covariances; variances are 2 and covariances are 0
        start_time = 0 # start time in seconds
        #self.position = position # initial position
        #self.velocity = np.array([0,0,0]) # initial velocity
        #self.acceleration = np.array([0,0,0]) # initial acceleration
        self.pva = np.array([0,0,0]) # position, velocity, acceleration
        #self.number_of_IMU_measurements = 0
        self.time_of_last_update = start_time
        # Matrices used in Kalman filter
        self.P = P
        self.A = np.identity(3)
        self.AT = self.A.transpose()
        self.Q = np.array([[0.1,0,0],
                           [0,0.2,0],
                           [0,0,0.2]])
        #self.H = np.array([[1,0,0,0,0,0,0,0,0],[0,1,0,0,0,0,0,0,0],[0,0,1,0,0,0,0,0,0]])
        self.H = np.array([[1,0,0]])
        self.HT = self.H.transpose()

    def input(self, new_pos, time, std_dev):
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
        #self.A = np.array([[1,0,0,dt,0,0,da,0,0],
        #                   [0,1,0,0,dt,0,0,da,0],
        #                   [0,0,1,0,0,dt,0,0,da],
        #                   [0,0,0,1,0,0,dt,0,0],
        #                   [0,0,0,0,1,0,0,dt,0], 
        #                   [0,0,0,0,0,1,0,0,dt], 
        #                   [0,0,0,0,0,0,1,0,0],
        #                   [0,0,0,0,0,0,0,1,0],
        #                   [0,0,0,0,0,0,0,0,1]])
        self.A = np.array([[1, dt, da],
                           [0,  1, dt],
                           [0,  0,  1]])
        self.AT = self.A.transpose()
        #variances = [e*e for e in xyz_err]
        #self.R = np.array([[variances[0],0,0],
        #                   [0,variances[1],0],
        #                   [0,0,variances[2]]])
        self.R = np.array([[ std_dev*std_dev ]])
        # we work with nine-dimensional vector composed of three position
        # coordinates, three velocity coordinates and three acceleration
        # coordinates
        #estPos = self.A @ np.concatenate((self.position, self.velocity, self.acceleration), axis = None)
        estPos = self.A @ self.pva
        print('+++++++++++++++++++++++++++++++++++++++')
        print('... estPos:', estPos)
        print('... self.A:', self.A)
        print('... self.pva:', self.pva)
        estP = self.A @ self.P @ self.AT + self.Q
        # Update step, if there are more inputs, just change this part according
        # to parametres of the other measurement
        new_pos = np.array([[ new_pos ]])
        K = estP @ self.HT @ np.linalg.inv(self.H @ estP @ self.HT + self.R)
        #res = estPos + K@((xyz - self.H @ estPos).transpose())
        print('.......................................')
        print('... K:', K)
        print('... new_pos:', new_pos)
        print('... self.H:', self.H)
        print('... estPos:', estPos)
        print('... self.H @ estPos:', self.H @ estPos)
        print('... (new_pos - self.H @ estPos).transpose():', (new_pos - self.H @ estPos).transpose())
        print('... K@((new_pos - self.H @ estPos).transpose()):', K@((new_pos - self.H @ estPos).transpose()))
        self.pva = estPos + K@((new_pos - self.H @ estPos).transpose())
        print('... self.pva:', self.pva)
        self.P = estP - K @ self.H @ estP
        # saving current position, velocity and acceleration
        #self.position = res[:3]
        #self.velocity = res[3:6]
        #self.acceleration = res[6:]
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



class Speedometer:
    """
        Tracks the current speed and distance travelled.

        Designed to be able to distinguish from the odometry measuring whether
            a robot is moving or not.

        Input:
            * `update()` ... adds incremetal distance travelled

        Output:
            * `get_distance_travelled()`
            * `get_speed()`
    """

    def __init__(self):
        self.hard_reset()

    def update(self, time, distance):
        """
            Updates the total distance travelled by a new increase.

            Args:
                time (datetime.timedelta): time of the measurement;
                    Note: time in seconds can be obtained by calling
                    `time.total_seconds()`
                distance (float): distance travelled in meters
        """
        self.total_distance += distance
        self.history.append((time, self.total_distance))

    def reset(self):
        """
            Resets the distance travelled.

            Remember the current distance travelled and subtract it in
                `get_travelled_distance()`.
        """
        pass

    def hard_reset(self):
        """
            Force initial settings.
        """
        self.total_distance = 0.0
        self.distance_measured_from = 0.0
        self.history = []

    def get_distance_travelled(self):
        """
            Returns (float): the distance travelled in meters (can be set to
                zero by calling `reset()`)
        """
        return self.total_distance - self.distance_measured_from

    def get_speed(self):
        """
            Returns (float): current speed in meters per second (can be
                negative if reversing)
        """
        (t_a, s_a) = self.history[-2]
        (t_b, s_b) = self.history[-1]
        print(t_a, s_a, t_b, s_b)
        return (s_b - s_a) / (t_b.total_seconds() - t_a.total_seconds())

def main():
    print('xxx')
    kf = KalmanFilter()
    kf.input(1, 0.1, 1)

    measured_t = []
    measured_x = []
    for i in range(10):
        t = i
        measured_t.append(t)
        measured_x.append(np.sin(t))
        kf.input(1, 0.1, 1)
        #kf.input(t, pos, 1)

    import matplotlib.pyplot as plt
    plt.plot(measured_t, measured_x, "gx", label="measured x")
    plt.legend()
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":
    main()

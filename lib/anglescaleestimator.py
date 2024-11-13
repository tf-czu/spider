import numpy as np
import datetime

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


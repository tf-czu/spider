import math
import copy
import bisect

def interpolate_xyz_in_time(time_1, xyz_1, time_2, xyz_2, time):
    """
        Computes interpolation of two time-stamped positions in (3D) space.

        Args:
            time_1 (datetime.timedelta): time of the first position
            xyz_1 (list of float): first position as a list of (three)
                coordinates
            time_2 (datetime.timedelta): time of the second position
            xyz_2 (list of float): second position as a list of (three)
                coordinates
            time (datetime.timedelta): time between `time_1` and `time_2` where
                the interpolation is to be computed

        Returns (list of float): interpolated position as a list of (three)
            coordinates
    """
    assert time_1 <= time <= time_2
    if time_1 == time == time_2:
        return copy.copy(xyz_1)
    t_1 = time_1.total_seconds()
    t_2 = time_2.total_seconds()
    t = time.total_seconds()
    alpha = (t - t_1) / (t_2 - t_1)
    return [(1 - alpha)*xyz_1[i] + alpha*xyz_2[i] for i in range(len(xyz_1))]

class RootMeanSquareDeviationCounter:
    """
        Coputes root mean square deviation from two asynchronously provided
            series of `[x,y]` or `[x,y,z]` positions.

        The two series are referred to as A and B;
            they are stored in the attributes `list_of_A` and `list_of_B`.

        Attributes:
    """

    def __init__(self):
        self.A_positions = []
        self.B_positions = []
        self.A_times = []
        self.B_times = []

    def add_A(self, time, xyz):
        """
            Args:
                time (datetime.timedelta): time
                xyz (list of float): position as a list of (three) coordinates
        """
        self.A_times.append(time)
        self.A_positions.append(xyz)

    def add_B(self, time, xyz):
        """
            Args:
                time (datetime.timedelta): time
                xyz (list of float): position as a list of (three) coordinates
        """
        self.B_times.append(time)
        self.B_positions.append(xyz)

    def get_interpolated_B_position(self, t):
        if t <= self.B_times[0]:
            return self.B_positions[0]
        if t >= self.B_times[-1]:
            return self.B_positions[-1]
        idx = bisect.bisect_right(self.B_times, t)
        t_1 = self.B_times[idx - 1].total_seconds()
        t_2 = self.B_times[idx].total_seconds()
        xyz_1 = self.B_positions[idx - 1]
        xyz_2 = self.B_positions[idx]
        alpha = (t.total_seconds() - t_1) / (t_2 - t_1)
        return [(1 - alpha)*xyz_1[i] + alpha*xyz_2[i] for i in range(3)]

    def compute_rmsd(self, dimension = 3):
        sum_sqr_dist = 0
        n = 0
        for k in range(len(self.A_times)):
            t = self.A_times[k]
            xyz_A = self.A_positions[k]
            xyz_B = self.get_interpolated_B_position(t)
            sqr_dist = sum([(xyz_A[i] - xyz_B[i])**2 for i in range(dimension)])
            sum_sqr_dist += sqr_dist
            n += 1
        return math.sqrt(sum_sqr_dist / n)


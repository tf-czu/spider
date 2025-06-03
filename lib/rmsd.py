import math
import copy
import bisect

class RootMeanSquareDeviationCounter:
    """
        Coputes root mean square deviation from two asynchronously provided
            series of `[x,y]` or `[x,y,z]` positions.

        The two series are referred to as A and B;
            they are stored in the attributes `list_of_A` and `list_of_B`.

        Attributes:
            A_positions (list of list of float): list of `[x, y, z]` positions
            B_positions (list of list of float): list of `[x, y, z]` positions
            A_times (list of datetime.timedelta): times of `A_positions`
            B_times (list of datetime.timedelta): times of `B_positions`
    """

    def __init__(self):
        self.A_positions = []
        self.B_positions = []
        self.A_times = []
        self.B_times = []

    def add_A(self, time, xyz):
        """
            Add a new (time, position) pair.

            Args:
                time (datetime.timedelta): time
                xyz (list of float): position as a list of (three) coordinates
        """
        self.A_times.append(time)
        self.A_positions.append(xyz)

    def add_B(self, time, xyz):
        """
            Add a new (time, position) pair.

            Args:
                time (datetime.timedelta): time
                xyz (list of float): position as a list of (three) coordinates
        """
        self.B_times.append(time)
        self.B_positions.append(xyz)

    def get_interpolated_B_position(self, t):
        """
            Computes an interpolated value in the B time-series.

            The B time-series is given by `B_positions` and `B_times`.

            Args:
                t (datetime.timedelta): where to compute the interpolated value

            Returns (list of float): interpolated position as an `[x, y, z]` list;
                if `t` is outside of the range of the values in `B_times` then
                `None` is returned
        """
        if t < self.B_times[0]:
            return None
        if t > self.B_times[-1]:
            return None
        idx = bisect.bisect_right(self.B_times, t)
        if self.B_times[idx - 1] == t:
            return self.B_positions[idx - 1]
        t_1 = self.B_times[idx - 1].total_seconds()
        t_2 = self.B_times[idx].total_seconds()
        xyz_1 = self.B_positions[idx - 1]
        xyz_2 = self.B_positions[idx]
        alpha = (t.total_seconds() - t_1) / (t_2 - t_1)
        return [(1 - alpha)*xyz_1[i] + alpha*xyz_2[i] for i in range(3)]

    def compute_rmsd(self, dimension = 3):
        """
            Coputes root mean square deviation from the distance between the
                positions accumulated in the A and B series.

            The A and B series  are stored in:
                `A_positions`, `B_positions`, `A_times`, and `B_times`.

            The values of the B series are approximated by the linear
                interpolation.

            Args:
                dimension (int): how many coordinates are to be involved in the
                    `[x, y, z]` positions

            Returns (float): root mean square deviation
        """
        sum_sqr_dist = 0
        n = 0
        for k in range(len(self.A_times)):
            t = self.A_times[k]
            xyz_A = self.A_positions[k]
            xyz_B = self.get_interpolated_B_position(t)
            if xyz_B:
                sqr_dist = sum([(xyz_A[i] - xyz_B[i])**2 for i in range(dimension)])
                sum_sqr_dist += sqr_dist
                n += 1
        if n > 0:
            return math.sqrt(sum_sqr_dist / n)
        else:
            return None


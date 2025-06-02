import copy

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

    def __init__(self):
        self.list_of_X = []
        self.list_of_Y = []
        self.synchronized = None

    def add_X(self, time, xyz):
        self.list_of_X.append((time, xyz))

    def add_Y(self, time, xyz):
        self.list_of_Y.append((time, xyz))

    def synchronize(self):
        i = j = 0
        len_list_of_X = len(self.list_of_X[i])
        len_list_of_Y = len(self.list_of_Y[j])
        while i < len_list_of_X and j < len_list_of_Y:
            X_time, X_xyz = self.list_of_X[i]
            Y_time, Y_xyz = self.list_of_Y[j]
            if X_time == Y_time:
                self.synchronized
                i += 1
                j += 1
            if X_time < Y_time:
                pass

    def get_rmsd(self):
        return "abc"


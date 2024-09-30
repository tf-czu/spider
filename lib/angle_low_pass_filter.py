import math
import numpy as np

class AngleLowPassFilter:
    """
        A low-pass filter of a series of angles in radians.

        There is an issue as angle values are periodic (e.g. 400 degrees is the
            same value as 40 degrees).
        Therefore, in the code:
            1.  the angles are first tanslated to complex numbers,
            2.  then an average (a convex combination) is computed from the
                complex numbers,
            3.  finally, the resultin complex number is converted back to its
                 angle.
    """
    def __init__(self):
        self.angle = None  
        self.number_of_inputs = 0
        # the following 5 variables serve to determine whether the value of the
        # angle given by the low-pass filter has converged;
        # see `update_angle()`, `has_converged()`
        self.history = [] # FIFO of the last n computed angles
        self.history_max_len = 100
        self.converged = False
        self.lower_convergence_limit = 0.01
        self.upper_convergence_limit = 0.02

    def update(self, new_angle):
        """
            Insert a new angle value to the low-pass filter series.

            The angle, resulting from the low-pass filter, is then obtained by
                the method `get`.

            Args:
                new_angle (float): the angle in radians
        """
        if self.angle == None:
            self.angle = new_angle
        else:
            weight_of_new_angle = max(0.1, 1/(self.number_of_inputs+1))
            self_point = np.array([math.cos(self.angle), math.sin(self.angle)])
            new_angle_point = np.array([math.cos(new_angle), math.sin(new_angle)])
            # new point is a weighted average of two points of absolute value 1,
            # afterwards its angle is calculated
            new_point = self_point*(1-weight_of_new_angle) + new_angle_point*weight_of_new_angle
            point_as_complex_number = np.complex128(new_point[0]+new_point[1]*1j)
            self.angle = np.angle(point_as_complex_number) 
        self.number_of_inputs += 1
        self.history.append(self.angle)
        if len(self.history) > self.history_max_len:
            self.history.pop(0) # remove the oldest element

    def get(self):
        """
            Returns (float): the angle in radians as the current result of the
                low-pass filter.
        """
        return self.angle

    def get_convergence_norm(self):
        """
            Computes the measure of stabilization of the resulting angle.

            Returns (float): The more the returned vaued is close to zero, the
                more the resulting angle is stabilized.
                The returned value is computed from the last `history_max_len`
                values in the history.
        """
        if len(self.history) < self.history_max_len:
            return np.inf
        else:
            last_value = self.history[-1]
            weight = 0
            for value in self.history:
                weight += (value - last_value)**2
            weight /= self.history_max_len
            weight = math.sqrt(weight)
            return weight

    def has_converged(self):
        """
            Return (bool): `True` if the resulting angle has "stablized
                enough", or if it is "unstable just a little".
        """
        norm = self.get_convergence_norm()
        if  norm > self.upper_convergence_limit:
            self.converged = False
            return False
        if norm < self.lower_convergence_limit:
            self.converged = True
            return True
        if self.lower_convergence_limit <= norm <= self.upper_convergence_limit:
            return self.converged

def main():
    """
        A unit test of the class AngleLowPassFilter.

        Performing this unit test, 1 radian is expected to converge to 6
            radians in 200 steps.
    """
    target_angle = 6 # radians
    af = AngleLowPassFilter()
    af.update_angle(1)
    for i in range(200):
        af.update_angle(target_angle)
    resulting_angle = af.get_angle()
    t_x = np.cos(target_angle)
    t_y = np.sin(target_angle)
    r_x = np.cos(resulting_angle)
    r_y = np.sin(resulting_angle)
    if abs(t_x - r_x) < 0.0001 and abs(t_y - r_y) < 0.0001:
        print('Unit test on AngleLowPassFilter has been SUCCESSFULL.')
    else:
        print('Unit test on AngleLowPassFilter has FAILED.')

if __name__ == "__main__":
    main()

import math
import numpy as np

class FloatLowPassFilter:
    """
        A common low-pass filter of a series of floats used to filter scale
            ratio between the positions given by the odometry and IMU and the
            positions given by GPS.
    """
    def __init__(self):
        self.value = None  
        self.number_of_inputs = 0
        # the following 5 variables serve to determine whether the value of the
        # scale given by the low-pass filter has converged;
        # see `update_scale()`, `has_converged()`
        self.history = [] # FIFO of the last n computed scales
        self.history_max_len = 100
        self.converged = False
        self.lower_convergence_limit = 0.01
        self.upper_convergence_limit = 0.02

    def update(self, new_value):
        """
            Insert a new value to the low-pass filter series.

            The value, resulting from the low-pass filter, is then obtained by
                the method `get`.

            Args:
                new_value (float): the new value in the series
        """
        if self.value == None:
            self.value = new_value
        else:
            weight_of_new_value = max(0.1, 1/(self.number_of_inputs + 1))
            self.value = weight_of_new_value*new_value + (1 - weight_of_new_value)*self.value
        self.number_of_inputs += 1
        self.history.append(self.value)
        if len(self.history) > self.history_max_len:
            self.history.pop(0) # remove the oldest element

    def get(self):
        """
            Returns (float): the filtered value
        """
        return self.value

    def get_convergence_norm(self):
        """
            Computes the measure of stabilization of the resulting value.

            Returns (float): The more the returned vaued is close to zero, the
                more the resulting value is stabilized.
                The returned norm is computed from the last `history_max_len`
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
            Return (bool): `True` if the resulting value has "stablized enough"
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
        A unit test of the class FloatLowPassFilter.

        Performing this unit test, -100 is expected to converge to 5 in 200
            steps.
    """
    target_value = 5
    lpf = FloatLowPassFilter()
    lpf.update(-100)
    for i in range(200):
        lpf.update(target_value)
        #print(lpf.get())
    resulting_value = lpf.get()
    if abs(resulting_value - target_value) < 0.0001:
        print('Unit test on FloatLowPassFilter has been SUCCESSFUL.')
    else:
        print('Unit test on FloatLowPassFilter has FAILED.')

if __name__ == "__main__":
    main()

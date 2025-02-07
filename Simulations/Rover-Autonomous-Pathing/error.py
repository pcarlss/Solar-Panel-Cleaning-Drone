import numpy as np
import random



class GaussianDerivativeError:
    def __init__(self, k, q, s0=0, minval=None, maxval=None, invert=False):
        """Generic Gaussian Derivative Error Model (error proportional to measurement rate of change in backwards difference)

        Args:
            k (_type_): Proportionality constant
            q (float): Discretization width and measurement separation.
            s0 (int, optional): Noise floor. Defaults to 0.
            minval (_type_, optional): Capped minimum value. Defaults to None.
            maxval (_type_, optional): Capped maximum value. Defaults to None.
            invert (bool, optional): Inverse-proportional error model. Defaults to False.
        """

        self.k = k
        self.s0 = s0
        self.q = q
        self.minval = minval
        self.maxval = maxval
        self.invert = invert
    
    def apply(self, true_value, prev_value=None, slope=None, discretize=True):
        if not slope:
            slope = (true_value - prev_value)/self.q


        if self.invert:
            sigma = np.maximum(np.abs(self.k/slope), self.s0)
        else:
            sigma = np.maximum(np.abs(slope*self.k), self.s0)


        # Compute the standard distribution of the set and the random measurement
        measurement = true_value + np.random.normal(0, sigma)

        # Apply the error to the measurement
        if self.minval and measurement < self.minval:
            return self.minval
        if self.maxval and measurement > self.maxval:
            return self.maxval
        if discretize:
            return np.round(measurement/self.q) * self.q
        return measurement
        



class GaussianProportionalError:
    def __init__(self, k, s0=0, q=0.01, minval=None, maxval=None, invert=False):
        """Generic Gaussian Proportional Error Model (error proportional to measurement)

        Args:
            k (_type_): Proportionality constant
            s0 (int, optional): Noise floor. Defaults to 0.
            q (float, optional): Discretization width. Defaults to 0.01.
            minval (_type_, optional): Capped minimum value. Defaults to None.
            maxval (_type_, optional): Capped maximum value. Defaults to None.
            invert (bool, optional): Inverse-proportional error model. Defaults to False.
        """
        self.k = k
        self.s0 = s0
        self.q = q
        self.minval = minval
        self.maxval = maxval
        self.invert = invert


    def apply(self, true_value, discretize=True):

        # Compute the standard distribution of the set and the random measurement
        if self.invert:
            sigma = np.maximum(np.abs(self.k/true_value), self.s0)
        else:
            sigma = np.maximum(np.abs(true_value*self.k), self.s0)

        measurement = true_value + np.random.normal(0, sigma)

        # Apply the error to the measurement
        if self.minval and measurement < self.minval:
            return self.minval
        if self.maxval and measurement > self.maxval:
            return self.maxval
        if discretize:
            return np.round(measurement/self.q) * self.q
        return measurement



    



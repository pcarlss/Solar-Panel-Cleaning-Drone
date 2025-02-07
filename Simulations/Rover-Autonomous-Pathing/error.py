import numpy as np
import random

class GaussianProportionalError:
    def __init__(self, k, s0=0, q=0.01, minval=None, maxval=None):
        """
        Standard gaussian proportional error
        """
        self.k = k
        self.s0 = s0
        self.q = q
        self.minval = minval
        self.maxval = maxval


    def apply(self, true_value, discretize=True):

        # Compute the standard distribution of the set and the random measurement
        sigma = np.max(np.abs(true_value*self.k), self.s0)
        measurement = true_value + np.random.normal(0, sigma)

        # Apply the error to the measurement
        if minval and measurement < minval:
            return minval
        if maxval and measurement > maxval:
            return maxval
        if discretize:
            return np.round(measurement/self.q) * self.q
        return measurement



    



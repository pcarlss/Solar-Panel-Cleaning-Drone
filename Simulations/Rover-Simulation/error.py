import numpy as np
import random

class Noise:
    def __init__(self):
        """Parent class with default methods for noise generation stuff
        """
        pass

    def apply(self, true_value):
        """Apply a noise value to a real measurement"""
        return true_value
    
    def error(self, true_value, *args, **kwargs):
        return self.apply(true_value, *args, **kwargs) - true_value

class GaussianNoise(Noise):
    """Gaussian noise model
    """
    def __init__(self, k, dt):
        """Standard gaussian noise model. IMU noise parameter k should be interpreted as (ug/sqrt(Hz))

        Args:
            k (float): IMU noise parameter
            dt (timestep): timestep
        """
        self.k = k
        self.dt = dt
        ug_to_ms2 = 9.81e-6
        self.sigma = k*ug_to_ms2 / (dt**2)
    
    def apply(self, true_value, discretize=False):
        return true_value + np.random.normal(0,self.sigma, size=true_value.shape)
    
class GaussMarkovBias(Noise):
    def __init__(self, tau=100.0, sigma=0.01, initial_bias=0.0):
        """
        :param tau: Correlation time (seconds)
        :param sigma: Diffusion strength (bias volatility)
        :param initial_bias: Initial bias value
        """
        self.tau = tau
        self.sigma = sigma
        self.bias = initial_bias

    def update(self, dt):
        """
        Update bias using Gauss-Markov dynamics
        :param dt: Time step (seconds)
        :return: Updated bias
        """
        # Decay factor (mean reversion)
        decay = 1.0 - dt / self.tau
        # Diffusion noise term
        noise = self.sigma * np.sqrt(dt) * np.random.randn()
        # Update bias
        self.bias = decay * self.bias + noise
        return self.bias
    
    def apply(self, true_value, dt):
        self.update(dt)
        return true_value + self.bias

class GaussianDerivativeError(Noise):
    def __init__(self, k, q, dt, s0=0, minval=None, maxval=None, invert=False):
        """Generic Gaussian Derivative Error Model (error proportional to measurement rate of change in backwards difference)

        Args:
            k (_type_): Proportionality constant
            q (float): Discretization width and measurement separation.
            dt (float): Timestep.
            s0 (int, optional): Noise floor. Defaults to 0.
            minval (_type_, optional): Capped minimum value. Defaults to None.
            maxval (_type_, optional): Capped maximum value. Defaults to None.
            invert (bool, optional): Inverse-proportional error model. Defaults to False.
        """

        self.k = k
        self.s0 = s0
        self.q = q
        self.dt = dt
        self.minval = minval
        self.maxval = maxval
        self.invert = invert
    
    def apply(self, true_value, prev_value=None, slope=None, discretize=True):
        if not slope:
            slope = (true_value - prev_value)/self.dt


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
        



class GaussianProportionalError(Noise):
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



    



import numpy as np
import random


#Standard Gaussian error model
class ErrorModel:
    def __init__(self, max, min, sigma):
        self.max = max
        self.min = min
        self.sigma = sigma
        
        pass
    
        
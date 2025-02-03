import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt

from panel import Area



class Rover:
    def __init__(self, start_x: float, start_y: float, start_theta: float, panel: Area):
        self.start_x = start_x
        self.start_y = start_y
        self.start_theta = start_theta
        self.panel = panel
        
    
    
    
    
        
        
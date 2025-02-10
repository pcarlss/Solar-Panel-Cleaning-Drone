import numpy as np
from common import *


class SolarPanelArea:
    def __init__(self, width: float, height: float, div: int):
        # basic dimensions and divisions
        self.width = width
        self.height = height
        self.div = div
        
        # length of x and y increments
        self.xstep = width/div
        self.ystep = height/div
        
        # x and y ranges as linspace
        x = np.linspace(0, width, div, endpoint=False)
        y = np.linspace(0, height, div, endpoint=False)

        # create position_array with all points x, y
        self.position_array = np.empty((div, div), dtype=object) 
        for i in range(div):
            for j in range(div):
                self.position_array[i][j] = Point(coord=(x[i], y[j]))
        pass 
    
    
    def locate(self, x, y):
        """Locate point associated with indicated coordinates. Rounds down by default
        
        Returns:
            Point: point nearest coordinates
        """
        if x <= 0 or x >= self.width:
            raise OutOfBoundsError(f"X-coordinate {x} out of bounds [0, {self.width}]")
        if y <= 0 or y >= self.height:
            raise OutOfBoundsError(f"y-coordinate {y} out of bounds [0, {self.height}]")
        
        return self.position_array[int(x//self.xstep)][int(y//self.ystep)]

    def locate_round(self, x, y):
        """Locate point associated with indicated coordinates. Rounds to actual nearest point

        Returns:
            Point: point nearest coordinates
        """
        if x <= 0 or x >= self.width:
            raise OutOfBoundsError(f"X-coordinate {x} out of bounds [0, {self.width}]")
        if y <= 0 or y >= self.height:
            raise OutOfBoundsError(f"y-coordinate {y} out of bounds [0, {self.height}]")
        
        return self.position_array[int(round(x/self.xstep))][int(round(y/self.ystep))]
    

    def get_random_starting_location(self,rover):
        pass

    def update_rover_on_panel(rover_actual_data):
        pass

    
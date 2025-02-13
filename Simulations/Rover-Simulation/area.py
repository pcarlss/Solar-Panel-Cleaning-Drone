import numpy as np
import matplotlib.pyplot as plt
from common import *

class SolarPanelArea:
    def __init__(self, width: float, height: float, div: int):
        # basic dimensions and divisions
        self.width = width
        self.height = height
        self.div = div
        
        # length of x and y increments
        self.xstep = width / div
        self.ystep = height / div
        
        # x and y ranges as linspace
        x = np.linspace(0, width, div, endpoint=False)
        y = np.linspace(0, height, div, endpoint=False)

        # create position_array with all points x, y
        self.position_array = np.empty((div, div), dtype=object) 
        for i in range(div):
            for j in range(div):
                self.position_array[i][j] = Point(coord=(x[i], y[j]))
        
        # Initialize visualization
        self.init_plot()

    def init_plot(self):
        """Initialize live visualization of the panel area and rover."""
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.ax.set_xticks(np.linspace(0, self.width, self.div))
        self.ax.set_yticks(np.linspace(0, self.height, self.div))
        self.ax.grid(True)

        # Scatter plot of grid points
        grid_x = [self.position_array[i][j].coord[0] for i in range(self.div) for j in range(self.div)]
        grid_y = [self.position_array[i][j].coord[1] for i in range(self.div) for j in range(self.div)]
        self.ax.scatter(grid_x, grid_y, color="gray", s=5)

        # Rover marker (start at origin)
        self.rover_marker, = self.ax.plot([], [], "ro", markersize=8)  # Red circle for rover

        plt.ion()  # Interactive mode on
        plt.show()

    def update_rover_on_panel(self, rover_actual_data):
        """Update the rover's position on the panel based on actual data."""
        x, y = rover_actual_data  # Assume it's a tuple (x, y)
        
        if x < 0 or x > self.width or y < 0 or y > self.height:
            raise OutOfBoundsError(f"Rover position ({x}, {y}) is out of bounds.")

        # Update rover marker position
        self.rover_marker.set_data([x], [y])  # Set the data for the marker (x, y)

        # Refresh the plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

import numpy as np
import random
from dataclasses import dataclass
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from components import DCMotorDiscrete, SimpleMotor
from panel import Area


class Rover:
    def __init__(self, start_x: float, start_y: float, start_theta: float, panel: Area):
        """
        Rover class should contain information relevant to the Assembly of various components (i.e., passing information from one to the other)
        For passing information between the panel and the rover, use something else.

        Args:
            start_x (float): Initial X position
            start_y (float): Initial Y position
            start_theta (float): Initial angle
            panel (Area): Initialize the panel on the rover (maybe bad idea)
        """
        self.start_x = start_x
        self.start_y = start_y
        self.start_theta = start_theta
        self.panel = panel
        
        # initialize left and right motors
        self.lmotor = SimpleMotor()
        self.rmotor = SimpleMotor()
        
        self.l_desired_speed = 0 #desired speed of left track, rad/s 
        self.r_desired_speed = 0 #desired speed of right track, rad/s 

        self.axle_length = 0.170 #170mm
        self.wheel_radius = 0.005 #5mm
        
    def set_trajectory(self, desired_speed, desired_turn_rate):
        """
        Sets the velocity of the left and right motors

        Args:
            desired_speed (_type_): _description_
            desired_turn_rate (_type_): _description_
        """
        # from the equations: 
        # Vavg = r (wl + wr) / 2
        # w = r (wr - wl) / 2
        self.l_desired_speed = (desired_speed - (self.axle_length * desired_turn_rate) / 2) / self.wheel_radius
        self.r_desired_speed = (desired_speed + (self.axle_length * desired_turn_rate) / 2) / self.wheel_radius
        
    
if __name__ == "__main__":
    # Working test code for scrolling visualization window -- not hooked up to any sim elements yet
    x = [1]
    y = [random.randint(1,10)]

    xbuf = 100

    # creating the first plot and frame
    fig, ax = plt.subplots()
    graph = ax.plot(x,y,color = 'g')[0]
    
    plt.ylim(0,10)
    # updates the data and graph
    def update(frame):
        global graph

        # updating the data
        x.append(x[-1] + 1)
        y.append(random.randint(1,10))

        # creating a new graph or updating the graph
        graph.set_xdata(x)
        graph.set_ydata(y)
        if len(x) >= xbuf:
            plt.xlim(x[-1*xbuf], x[-1])
        else: 
            plt.xlim(x[0], x[-1])
            
    
    
    anim = FuncAnimation(fig, update, frames = len(x), interval = 10, cache_frame_data=False)
    
    plt.show()
        
            

    
    
        
        
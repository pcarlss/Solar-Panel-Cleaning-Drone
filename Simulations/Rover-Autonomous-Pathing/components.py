import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm

class IMU:
    def __init__(self):
        self.prev_acceleration = [0, 0]  # [ax, ay] in mm/s²
        self.prev_velocity = 0          # Scalar velocity in mm/s
        self.prev_orientation = 0       # Angle in degrees (yaw)
        self.prev_position = [0, 0]     # [x, y] position in mm

    def getInfo(self):
        return 

class LimitSwitch:
   def detect_edge(self, solar_panel_area, rover_position):
        return 

class RotaryEncoder:
    def getTrackVelocity(self):
        return

class SimpleMotor():
    def __init__(self, Vmax=12, Vmin=-12, speedmax=10, speedmin=-10):
        """
        Super simplified motor model with speed directly proportional to voltage
        """
        self.Vmax = Vmax
        self.Vmin = Vmin
        self.speedmax = speedmax
        self.speedmin = speedmin
        
        self.slope = (self.speedmax-self.speedmin)/(self.Vmax-self.Vmin)
        self.b = self.speedmax - self.slope*self.Vmax
        self.speed = 0
        self.voltage = 0
        pass
    
    def update(self, voltage):
        """
        Update motor state for one time step using the given voltage
        Returns current angular velocity
        """
        self.voltage = max(min(voltage, self.Vmax), self.Vmin) 
        self.speed = self.slope*self.voltage - self.b
        return self.speed # Return angular velocity
    
    def get_speed(self):
        """Get the current angular velocity"""
        return self.speed


class DCMotorDiscrete:
    def __init__(self, dt=0.1, J=0.01, b=0.1, K=0.01, R=1.0, L=0.5, Vmax=12, Vmin=-12):
        """
        Discrete-time DC motor model with state-space representation
        """
        self.dt = dt  # Time step [s]
        self.J = J
        self.b = b
        self.K = K
        self.R = R
        self.L = L
        self.Vmax = Vmax
        self.Vmin = Vmin
        
        # State-space matrices (continuous time)
        A = np.array([
            [-R/L, -K/L],
            [K/J,  -b/J]
        ])
        B = np.array([[1/L], [0]])
        
        # Discretize using matrix exponential
        M = expm(np.block([[A, B], [np.zeros((1, 3))]]) * dt)
        self.Ad = M[:2, :2]
        self.Bd = M[:2, 2:3]
        
        # Initialize states [current, angular_velocity]
        self.x = np.array([[0], [0]])
        
        # Initialize voltage
        self.voltage = 0.0

    def update(self, voltage):
        """
        Update motor state for one time step using the given voltage
        Returns current angular velocity
        """
        self.voltage = max(min(voltage, self.Vmax), self.Vmin) 
        self.x = self.Ad @ self.x + self.Bd * self.voltage
        return self.x[1, 0]  # Return angular velocity
    
    def get_speed(self):
        """Get the current angular velocity"""
        return self.x[1, 0]

class TrackMotor:
    def power(self, pwm,direction):
       return
   
class CleaningMotor:
    def __init__(self):
        self.is_cleaning = False


    def power(self, is_on:bool):
       self.is_cleaning = is_on


    def is_cleaning(self):
        return self.is_cleaning
    
class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.1):
        """
        Generic PID controller with settable Kp, Kd, Ki, and dt
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.reset()
        
    def reset(self):
        """
        Reset errors completely
        """
        self.integral = 0
        self.previous_error = 0
        self.error = 0
        
    def calculate(self, setpoint, measurement):
        """
        Calculate next gain output using setpoint (desired) and measurement (data). 
        """
        self.error = setpoint - measurement
        
        # Proportional term
        P = self.Kp * self.error
        
        # Integral term
        self.integral += self.error * self.dt
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (self.error - self.previous_error) / self.dt
        self.previous_error = self.error
        
        return P + I + D
 
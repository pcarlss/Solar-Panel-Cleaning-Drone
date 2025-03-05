import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm
from area import SolarPanelArea
from error import GaussianNoise
from common import PositionalInformation

class IMU:
    def __init__(self, time_step, k_linear, k_angular, position_offset=np.zeros((3,1)), angular_offset=np.eye(3,3), linear_cross_axis=np.eye(3,3), angular_cross_axis = np.eye(3,3)):
        """IMU function.

        Args:
            position_offset (np.array, optional): Positional offset vector from actual turn center. Defaults to np.zeros((1,3)).
            angular_offset (np.array, optional): Angular offset alignment matrix, define as 3-axis rotation matrix. Defaults to np.eye(3).
            cross_axis (_type_, optional): . Defaults to np.eye(3).
        """
        # self.prev_acceleration = [0, 0]  # [ax, ay] in mm/s²
        # self.prev_velocity = 0          # Scalar velocity in mm/s
        # self.prev_orientation = 0       # Angle in degrees (yaw)
        # self.prev_position = [0, 0]     # [x, y] position in mm
        self.position_offset = position_offset
        self.angular_offset = angular_offset
        self.inv_angular_offset = np.linalg.inv(angular_offset)
        self.angular_cross_axis = angular_cross_axis
        self.inv_angular_cross_axis = np.linalg.inv(angular_cross_axis)
        self.linear_cross_axis = linear_cross_axis
        self.inv_linear_cross_axis = np.linalg.inv(linear_cross_axis)
        self.time_step = time_step
        ug_to_ms2 = 9.81e-6
        dps_to_radps = np.pi/180

        self.linear_error_model = GaussianNoise(k_linear*ug_to_ms2, time_step)
        self.angular_error_model = GaussianNoise(k_angular*dps_to_radps, time_step)

        
    def get_imu_data(self, positional_information: PositionalInformation, body_axis=True):

        #obtain ground-truth values
        ax_body = positional_information.linear_accel # linear component
        ay_body = positional_information.linear_velocity * positional_information.turn_rate # centripetal component
        az_body = 0

        # column-vector representation
        a_body = np.array([[ax_body,ay_body,az_body]]).T
        w_body = np.array([[0,0,positional_information.turn_rate]]).T
        alpha_body = np.array([[0,0,positional_information.turn_accel]]).T

        # apply axis change
        a_imu = a_body + np.cross(alpha_body,self.position_offset, axis=0) + np.cross(w_body,np.cross(w_body,self.position_offset, axis=0), axis=0)
        a_imu = self.inv_angular_offset@a_body
        w_imu = self.inv_angular_offset@w_body

        # Apply error functions
        a_imu_err = self.linear_error_model.apply(self.linear_cross_axis@a_imu)
        w_imu_err = self.angular_error_model.apply(self.angular_cross_axis@w_imu)


        if not body_axis:
            return a_imu_err, w_imu_err
        

        #revert to body frame
        a_body_err = self.angular_offset@a_imu_err - np.cross(alpha_body,self.position_offset, axis=0) - np.cross(w_body,np.cross(w_body,self.position_offset, axis=0), axis=0)
        w_body_err = self.angular_offset@w_imu_err

        return a_body_err, w_body_err

class LimitSwitch:
   def __init__(self, solar_panel_area: SolarPanelArea, relative_pos):
       self.solar_panel_area = solar_panel_area
       self.relative_pos = relative_pos

   def is_pressed(self, rover_position, rover_azimuth):
        #compute switch location
        #check solarpanel area if switch detects edge
        x, y = self.get_position(rover_position, rover_azimuth)
        ret_val = True
        if x > self.solar_panel_area.width or x < 0:
            ret_val = False
            
        if y > self.solar_panel_area.height or y < 0:
            ret_val = False
        
        
        return ret_val
   
   def get_position(self, rover_position, rover_azimuth):
        x0, y0 = rover_position
        dx_rel, dy_rel = self.relative_pos
        
        dx = dx_rel*np.cos(rover_azimuth) - dy_rel*np.sin(rover_azimuth)
        dy = dx_rel*np.sin(rover_azimuth) + dy_rel*np.cos(rover_azimuth)

        return np.array([x0+dx, y0+dy])

class RotaryEncoder:
    def __init__(self, resolution=20, time_step=0.01, zero_time=0.5, reverse_timeout_window=5):
        self.min_angle = 2*np.pi/resolution
        self.zero_time = zero_time
        self.time_step = time_step   
        self.discrete_pos = 0
        self.position = (np.random.rand()-0.5)*2*np.pi #random position between -pi and pi
        self.velocity = 0
        self.time_between = 0
        self.prev_step = None
        self.reverse_timeout_window = reverse_timeout_window
        self.since_last_change = 0
        pass
    

    def get_track_velocity(self, track_vel):
        self.position += track_vel * self.time_step
        self.time_between += self.time_step
        step = (self.position - self.discrete_pos) // self.min_angle
        
        # Handling for immediate reversal of position (since we start from 0, this will cause speed to spike to negative limit)
        # Instead, ignore the first reading since this one may be erroneous. Underestimates velocity if the time step is too low/velocity too high.
        if self.prev_step == None:
            self.prev_step = step
            self.discrete_pos = self.discrete_pos + step*self.min_angle # update previous known position
            
        
        # Zero-crossing, this can reliably be ignored with higher frequency readings as it will give an abberant velocity indication.
        # Position must still be tracked and updated to new position, as well as step direction
        elif (self.prev_step != 0) and self.prev_step == -1*step:
            self.discrete_pos = self.discrete_pos + step*self.min_angle # update previous known position
            self.velocity = 0
            self.time_between = 0
            self.prev_step = step   
            self.since_last_change = 0

        # If it has stepped one complete step in a direction (+ or -): 
        elif step:
            self.discrete_pos = self.discrete_pos + step*self.min_angle # update previous known position
            self.velocity = step*self.min_angle/self.time_between
            self.time_between = 0
            self.prev_step = step
            self.since_last_change = 0
        
        # Zeroes out the rotary encoder velocity
        elif self.time_between > self.zero_time:
            self.velocity = 0
            self.time_between = self.zero_time
            
        self.since_last_change += 1
                    
        # self.prev_step = step # Try moving this to only when it steps?

        return self.velocity, self.discrete_pos
    



class TrackMotor:
    def power(self, pwm,direction):
       return
    
    def get_real_speed(self):
        return
    def get_real_acceleration(self):
        return
    
    def get_position(self,positional_information):
        return
   
class CleaningMotor:
    def __init__(self):
        self.is_cleaning = False


    def power(self, is_on:bool):
       self.is_cleaning = is_on


    def is_cleaning(self):
        return self.is_cleaning



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

    def __repr__(self):
        return f"Motor: <dt={self.dt}, J={self.J}, b={self.b}, K={self.K}, R={self.R}, L={self.L}, Vmax={self.Vmax, self.Vmin}>"

    def update(self, voltage):
        """
        Update motor state for one time step using the given voltage
        Returns current angular velocity
        """
        self.voltage = max(min(voltage, self.Vmax), self.Vmin) 
        self.x = self.Ad @ self.x + self.Bd * self.voltage
        return self.x[1, 0]  # Return angular velocity

    def get_angular_acceleration(self):
        """
        Calculate instantaneous angular acceleration using:
        alpha = (K*i - b*w)/J
        Where:
        - i = current (from state vector)
        - w = angular velocity (from state vector)
        """
        current = self.x[0, 0]      # First state element: current [A]
        angular_velocity = self.x[1, 0]  # Second state element: angular velocity [rad/s]
        
        return (self.K * current - self.b * angular_velocity) / self.J
    
    def get_speed(self):
        """Get the current angular velocity"""
        return self.x[1, 0]

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
        
    def calculate(self, setpoint, measurement, angular=False):
        """
        Calculate next gain output using setpoint (desired) and measurement (data). 
        """
        self.error = setpoint - measurement
        if angular:
            self.error = ((setpoint - measurement + np.pi) % (2*np.pi)) - np.pi
        
        # Proportional term
        P = self.Kp * self.error
        
        # Integral term
        self.integral += self.error * self.dt
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (self.error - self.previous_error) / self.dt
        self.previous_error = self.error
        
        return P + I + D
 
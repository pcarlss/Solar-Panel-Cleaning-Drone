import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import expm


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

class PIDController:
    def __init__(self, Kp=1.0, Ki=0.0, Kd=0.0, dt=0.1):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.reset()
        
    def reset(self):
        self.integral = 0
        self.previous_error = 0
        
    def calculate(self, setpoint, measurement):
        error = setpoint - measurement
        
        # Proportional term
        P = self.Kp * error
        
        # Integral term
        self.integral += error * self.dt
        I = self.Ki * self.integral
        
        # Derivative term
        D = self.Kd * (error - self.previous_error) / self.dt
        self.previous_error = error
        
        return P + I + D


# Simulation example
if __name__ == "__main__":
    dt = 0.01  # Time step [s]
    sim_time = 5.0  # Total simulation time [s]
    steps = int(sim_time/dt)
    
    motor = DCMotorDiscrete(K = 1.14, dt=dt)
    pid = PIDController(Kp=3, Ki=10, Kd=0.05, dt=dt)
    
    # motor = SimpleMotor()
    # pid = PIDController(Kp=0, Ki=5, Kd=0, dt=dt)

    time = np.zeros(steps)
    speed = np.zeros(steps)
    target = np.zeros(steps)
    voltage_input = np.zeros(steps)
    
    # Simulation loop
    for i in range(steps):
        # Update target speed at specific times
        if i < 100:
            new_target = 0
        elif i < 300:
            new_target = 2.5
        else:
            new_target = 5
        
        # Get current speed
        current_speed = motor.get_speed()
        
        # Calculate control voltage using PID controller
        control_voltage = pid.calculate(new_target, current_speed)
        
        # Update motor state with the control voltage
        motor.update(control_voltage)
        
        # Record values
        time[i] = i * dt
        speed[i] = current_speed
        target[i] = new_target
        voltage_input[i] = control_voltage
    
    # Plot results
    plt.figure(figsize=(10, 6))
    
    plt.plot(time, target, 'r--', label='Target Speed')
    plt.plot(time, speed, 'b-', label='Actual Speed')
    # plt.plot(time, voltage_input, 'g--', label='Control Voltage')
    
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Velocity [rad/s] / Voltage [V]')
    plt.title('Motor Speed Tracking with PID Control')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
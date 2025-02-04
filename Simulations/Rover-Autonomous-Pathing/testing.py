import random
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from components import DCMotorDiscrete, SimpleMotor, PIDController
from area import SolarPanelArea
from rover import Rover



def visualization_test():
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

def motor_test():
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

def panel_test():
    a = SolarPanelArea(1, 1, 10)
    print(a.locate_round(0.74, 0.76)) 

if __name__ == '__main__':
    # Write which test you want to run here

    #motor_test()
    #visualization_test()
    #panel_test()



    pass
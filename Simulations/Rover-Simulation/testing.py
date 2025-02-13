import random
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from components import DCMotorDiscrete, SimpleMotor, PIDController
from area import SolarPanelArea
from rover import Rover

from error import GaussianProportionalError, GaussianDerivativeError, GaussianNoise, GaussMarkovBias


def imu_test():
    dt=0.1

    accel_func_list = [
        lambda x: 0,
        lambda x: -1,
        lambda x: 0,
        lambda x: 1,
        lambda x: 0, 
        lambda x: 1,
        lambda x: 0,
        lambda x: -1,
        lambda x: 0
    ]
    condition_list = lambda x: [
        x >= 0,
        x >= 10,
        x >= 11,
        x >= 30,
        x >= 31,
        x >= 50,        
        x >= 51,
        x >= 70,
        x >= 71,

    ]
    xs = np.arange(0,200,dt)
    piecewise_accels = np.piecewise(xs, condlist=condition_list(xs), funclist=accel_func_list)
    piecewise_velocities = np.cumsum(piecewise_accels*dt)
    piecewise_positions = np.cumsum(piecewise_velocities*dt)

    # error_model = GaussianProportionalError(k=0.5,s0=0.001,dt=dt)
    error_model = GaussianNoise(20, 0.1)

    measured_accels = error_model.apply(piecewise_accels)
    measured_velocities = np.cumsum(measured_accels*dt)
    measured_positions = np.cumsum(measured_velocities*dt)

    fig, axs = plt.subplots(2,3)

    axs[0,0].plot(xs, measured_accels, 'r-')
    axs[0,0].plot(xs, piecewise_accels, 'k--')
    axs[0,0].set_title("Acceleration")

    axs[0,1].plot(xs, measured_velocities, 'r-')
    axs[0,1].plot(xs, piecewise_velocities, 'k--')
    axs[0,1].set_title("Velocity")

    axs[0,2].plot(xs, measured_positions, 'r-')
    axs[0,2].plot(xs, piecewise_positions, 'k--')
    axs[0,2].set_title("Position")

    axs[1,0].plot(xs, piecewise_accels-measured_accels)
    axs[1,0].set_title("Acceleration Error")

    axs[1,1].plot(xs, piecewise_velocities-measured_velocities)
    axs[1,1].set_title("Velocity Error")

    axs[1,2].plot(xs, piecewise_positions-measured_positions)
    axs[1,2].set_title("Position Error")



    plt.show()


    pass



def gauss_error_test():
    x = np.linspace(0, 10, 100)
    real_y = np.sin(x)

    errormodel = GaussianProportionalError(0.1, 0.005, q=0.1, minval=-1, maxval=1)
    measure_y = np.array(list(map(lambda x: errormodel.apply(x, discretize=True), real_y,)))

    plt.plot(x, real_y, 'k--')
    plt.step(x, measure_y, 'r-')
    plt.xlim(np.min(x), np.max(x))
    plt.ylim(np.min(np.append(real_y, measure_y)),np.max(np.append(real_y, measure_y)))
    plt.show()
    pass

def derivative_error_test():
    q = 0.01
    x = np.arange(-2, 2, q)
    real_y = np.sin(x**2)
    real_y_prev = np.insert(real_y, 0, values=0)

    errormodel = GaussianDerivativeError(0.05, s0=0.005, q=q)
    measure_y = np.array(list(map(lambda x, x_prev: errormodel.apply(x, x_prev, discretize=True), real_y, real_y_prev)))

    plt.step(x, measure_y, 'r-')
    plt.plot(x, real_y, 'k--')

    plt.xlim(np.min(x), np.max(x))
    plt.ylim(np.min(np.append(real_y, measure_y)),np.max(np.append(real_y, measure_y)))
    plt.show()
    pass

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
            new_target = 6.8
        else:
            new_target = -6.8
        
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

def rover_movement_test():
    panel = SolarPanelArea(10,10,1000)
    plot_xs = [0]
    plot_ys = [0]

    rover = Rover(panel, 0.01)
    tracking_point = (1,1)
    print(rover.track_motor_r)
    rover.set_trajectory(0.05,1)
    print(f"Desired speeds: {rover.l_desired_speed, rover.r_desired_speed}")

    for t in range(50000):
        rover.update_motors()
        rover.update_position()
        # print(f"Motor State: {rover.update_motors()}")
        # print(f"Motor: {rover.update_position()}")

        plot_xs.append(rover.positional_information.position[0])
        plot_ys.append(rover.positional_information.position[1])

    plt.plot(plot_xs, plot_ys)
    plt.show()

        



if __name__ == '__main__':
    # Write which test you want to run here

    # motor_test()
    # visualization_test()
    # panel_test()

    # derivative_error_test()
    # imu_test()

    rover_movement_test()


    pass
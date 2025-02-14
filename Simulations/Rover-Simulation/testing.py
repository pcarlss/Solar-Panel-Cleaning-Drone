import random
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
from components import DCMotorDiscrete, SimpleMotor, PIDController, RotaryEncoder
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
        x >= 15,
        x >= 30,
        x >= 35,
        x >= 50,        
        x >= 55,
        x >= 70,
        x >= 75,

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
    
    fig.set_size_inches(9, 9)
    
    plt.setp(axs, xlim=(0,200))
                

    m_a, = axs[0,0].plot(xs, measured_accels, 'r-')
    p_a, = axs[0,0].plot(xs, piecewise_accels, 'k--')
    axs[0,0].set_title("Acceleration")

    m_v, = axs[0,1].plot(xs, measured_velocities, 'r-')
    p_v, = axs[0,1].plot(xs, piecewise_velocities, 'k--')
    axs[0,1].set_title("Velocity")

    m_p, = axs[0,2].plot(xs, measured_positions, 'r-')
    p_p, = axs[0,2].plot(xs, piecewise_positions, 'k--')
    axs[0,2].set_title("Position")

    a_e, = axs[1,0].plot(xs, piecewise_accels-measured_accels)
    axs[1,0].set_title("Acceleration Error")

    v_e, = axs[1,1].plot(xs, piecewise_velocities-measured_velocities)
    axs[1,1].set_title("Velocity Error")

    p_e, = axs[1,2].plot(xs, piecewise_positions-measured_positions)
    axs[1,2].set_title("Position Error")
    
    # def animate(i):
    #     x = xs[:i]
    #     m_a.set_data(x, measured_accels[:i])
    #     p_a.set_data(x, piecewise_accels[:i])
    #     m_v.set_data(x, measured_velocities[:i])
    #     p_v.set_data(x, piecewise_velocities[:i])
    #     m_p.set_data(x, measured_positions[:i])
    #     p_p.set_data(x, piecewise_positions[:i])
    #     a_e.set_data(x, (piecewise_accels-measured_accels)[:i])
    #     v_e.set_data(x, (piecewise_velocities-measured_velocities)[:i])
    #     p_e.set_data(x, (piecewise_positions-measured_positions)[:i])
    #     return m_a, p_a, m_v, p_v, m_p, p_p, a_e, v_e, p_e
    
    # ani = FuncAnimation(
    #     fig,
    #     animate,
    #     interval=1,
    #     blit=False,
    #     frames=range(len(xs)),
    #     repeat_delay=100
    # )
    
    # writer = animation.PillowWriter(fps=15,
    #                                 bitrate=1800)
    # ani.save('imu.gif', writer=writer)


    plt.tight_layout()
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
        voltage_input[i] = max(min(control_voltage,12),-12)
    
    # Plot results
    plt.figure(figsize=(9, 9))
    
    plt.plot(time, target, 'r--', label='Target Speed')
    plt.plot(time, speed, 'b-', label='Actual Speed')
    plt.plot(time, voltage_input, 'g--', label='Control Voltage')
    plt.xlim(0,5)
    plt.ylim(-12.5,12.5)
    
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
    plot_xs = []
    plot_ys = []
    or_xs, or_ys = [], []
    time_step = 0.01
    time_stop = 5
    time_arange = np.arange(0, time_stop, time_step)

    rover = Rover(panel, time_step)
    rover.set_trajectory(0.05, 1)

    print(f"Desired speeds: {rover.l_desired_speed, rover.r_desired_speed}")

    for t in range(100):
        rover.update_sensors()
        
    for t in time_arange:
        rover.update_sensors()
        rover.update_motors(use_sensors=True)
        rover.update_position()
        # print(f"Motor State: {rover.update_motors()}")
        # print(f"Motor: {rover.update_position()}")

        plot_xs.append(rover.positional_information.position[0])
        plot_ys.append(rover.positional_information.position[1])
        or_xs.append(rover.positional_information.orientation[0])
        or_ys.append(rover.positional_information.orientation[1])
        
        
    # scale = 0.25
    # plt.xlim(-0.05,0.25)
    # plt.ylim(0,0.25)
    
    fig, ax = plt.subplots(1,1)
    path, = ax.plot(plot_xs, plot_ys, 'k--', label='Rover path')
    r_point, = ax.plot(0,0,"ro", label='Rover CG')
    
    arr_scale = 0.0075
    or_arrow = ax.arrow(0,0,0,0 , head_width=0.0025, head_length=0.001, color='r', ec=None, label='Rover orientation')
    ax.set_aspect('equal')
    ax.set_title("Rover circular path demonstration")
    ax.legend(loc='center', ncol=1, fancybox=True, shadow=True)

    def animate(i):
        x, y = plot_xs[:i], plot_ys[:i]
        or_x, or_y = or_xs[i], or_ys[i]
        path.set_data(x, y)
        r_point.set_data([x[-1]],[y[-1]])
        or_arrow.set_data(x=x[-1],y=y[-1], dx=arr_scale*or_x, dy=arr_scale*or_y)
        return path, r_point
    
    ani = FuncAnimation(
        fig,
        animate,
        interval=10,
        blit=False,
        frames=range(1,len(plot_xs)),
        repeat_delay=100
    )
    
    writer = animation.PillowWriter(fps=60,
                                    bitrate=1800)
    ani.save('movement.gif', writer=writer)
        
    plt.show()

def rotary_encoder_test():
    time_step = 0.01
    time_stop = 5
    time_arr = np.arange(0,time_stop,time_step)

    encoder = RotaryEncoder(time_step=time_step, zero_time=1)
    encoder_v = []
    encoder_vavg = []
    encoder_p = []
    real_velocity = 1*signal.square(2 * np.pi * time_arr * 1/4)
    
    averaging_period = 3
    
    for v in real_velocity:
        enc_v, enc_p = encoder.get_track_velocity(v)
        encoder_v.append(enc_v)
        encoder_p.append(enc_p)
        if len(encoder_vavg) < averaging_period:
            encoder_vavg.append(enc_v)
        else:
            encoder_vavg.append(np.average(encoder_v[-1*averaging_period:-1]))

    
    plt.plot(time_arr, real_velocity, 'k--')
    plt.plot(time_arr, encoder_v)
    plt.plot(time_arr, encoder_vavg, 'r')
    
    plt.show()

def rover_sensor_movement_test():
    panel = SolarPanelArea(10,10,1000)
    
    time_step = 0.01
    time_stop = 10
    time_arange = np.arange(0, time_stop, time_step)

    rover = Rover(panel, time_step)
    rover.set_trajectory(0.05, 0.7)

    l_speed_actual = []
    l_speed_enc = []
    r_speed_actual = []
    r_speed_enc = []
    
    l_desired = np.full_like(time_arange, rover.l_desired_speed)
    r_desired = np.full_like(time_arange, rover.r_desired_speed)
    
    x_position = []
    y_position = []
    
    # This initial buffer is NECESSARY to prevent immediate-reversal issues. Also better represents real world situation
    for t in range(100):
        rover.update_sensors()
    
    for t in time_arange:
        rover.update_sensors()

        rover.update_motors(use_sensors=True)
        rover.update_position()
        
        x_position.append(rover.positional_information.position[0])
        y_position.append(rover.positional_information.position[1])
        
        l_speed_actual.append(rover.positional_information.l_speed)
        r_speed_actual.append(rover.positional_information.r_speed)

        l_speed_enc.append(rover.estimated_pos.l_speed)
        r_speed_enc.append(rover.estimated_pos.r_speed)
        
    

    
    fig, (l_plot, r_plot) = plt.subplots(2,1)  
    plt.setp((l_plot, r_plot), xlim=(0,time_stop), xlabel='time [s]', ylabel='Angular velocity [rad/s]')
    
    fig.set_size_inches(9,9)
    l_plot.set_title("Left track velocity")
    r_plot.set_title("Right track velocity")
    
    l_plot.plot(time_arange, l_desired, 'r--', label='Desired Velocity')
    r_plot.plot(time_arange, r_desired, 'r--', label='Desired Velocity')
        
    l_plot.plot(time_arange, l_speed_enc, 'b', lw=0.5, label='Measured Velocity')
    r_plot.plot(time_arange, r_speed_enc, 'b', lw=0.5, label='Measured Velocity')
    l_plot.plot(time_arange, l_speed_actual, 'k', label='Actual Velocity')
    r_plot.plot(time_arange, r_speed_actual, 'k', label='Actual Velocity')
    
    l_plot.legend()
    r_plot.legend()
    

    plt.tight_layout()


    print(np.shape(l_speed_enc))
    # scale = 0.25
    # plt.xlim(-1*scale,scale)
    # plt.ylim(-1*scale,scale)
    
    plt.show()


if __name__ == '__main__':
    # Write which test you want to run here

    # motor_test()
    # visualization_test()
    # panel_test()

    # derivative_error_test()
    imu_test()

    # rover_movement_test()
    # rotary_encoder_test()
    rover_sensor_movement_test()

    pass
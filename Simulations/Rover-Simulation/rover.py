from components import IMU, LimitSwitch, RotaryEncoder, TrackMotor, CleaningMotor, SimpleMotor, DCMotorDiscrete, PIDController
from common import DecisionStates, InnerLoopStates, OuterLoopStates, RadioMessage, SearchForCornerStates, PositionalInformation, MotorStates, LOCKOUT_COUNTDOWN
from dataclasses import dataclass
from typing import List, Literal
from scipy.ndimage import rotate
import time
import numpy as np


class Rover:
    def __init__(self, solar_panel_area, time_step, position = np.array([0,0]), orientation = np.array([1,0])):
        """Rover Class, contains all internal logic for the rover. Should be made to be portable to Arduino relatively easily

        Args:
            solar_panel_area (SolarPanelArea): panel area
            time_step (float): time step in seconds
            position (1D array): [x,y] initial position of the rover
            orientation (1D array): [x,y] unit vector orientation of the rover 
        """

        # X,Y,Theta and their derivatives
        self.positional_information = PositionalInformation(
            position=position, 
            orientation=orientation)
        
        # Estimated values for the above
        self.estimated_pos = PositionalInformation(
            position=np.array([0,0]), 
            orientation=np.array([1,0]))

        self.imu_info = [np.array([[0,0,0]]).T, np.array([[0,0,0]]).T]
        # Physical constants
        self.axle_length = 0.170 #170mm
        self.wheel_radius = 0.025 #25mm
        self.top_speed = 0.04 # 4 cm/s 
        self.top_turn_rate = 1 # rad/s

        # Simulation constants
        self.time_step = time_step

        # Sensor definitions
        self.imu = IMU(time_step=self.time_step, k_linear=400, k_angular=0.005)
        self.rotary_encoder_l = RotaryEncoder(resolution=20, time_step = self.time_step, zero_time=0.5)
        self.rotary_encoder_r = RotaryEncoder(resolution=20, time_step = self.time_step, zero_time=0.5)
        
        # Limit switches identified by last three chars: l/r for left/right, f/b for front/back, i/o for inboard/outboard
        self.limit_switch_lfo = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,0.105]))
        self.limit_switch_lfi = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,0.1]))
        self.limit_switch_rfo = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,-0.105]))
        self.limit_switch_rfi = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,-0.1]))
        self.limit_switch_lbo = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,0.105]))
        self.limit_switch_lbi = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,0.1]))
        self.limit_switch_rbo = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,-0.105]))
        self.limit_switch_rbi = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,-0.1]))
        
        self.track_motor_l = DCMotorDiscrete(K=1.14, J=4.785E-4, L=10e-3, R=30,  dt=time_step)
        self.track_motor_r = DCMotorDiscrete(K=1.14, J=4.785E-4, L=10e-3, R=30, dt=time_step)
        self.cleaning_motors = CleaningMotor()

        self.track_pid_l = PIDController(Kp=0.05, Ki=0.5, Kd=0.05, Kff=3, dt=time_step, clamp_value=1)
        self.track_pid_r = PIDController(Kp=0.05, Ki=0.5, Kd=0.05, Kff=3, dt=time_step, clamp_value=1)
    
        self.positional_pid = PIDController(Kp=0.25, Ki=0.05, Kd=0.0, dt=time_step)
        self.orientation_pid = PIDController(Kp=0.8, Ki=0, Kd=0, dt=time_step)
        self.turn_compensation_factor = 1.08 # determined through trial and error 

        # Decision States
        self.decision_state = DecisionStates.IDLE
        self.search_for_corner_state = SearchForCornerStates.MOVEBACKWARDSUNTILEDGE
        self.outer_loop_states = OuterLoopStates.FOLLOWEDGE
        self.inner_loop_states = InnerLoopStates.FOLLOWPATH
        self.radio_message = RadioMessage.NOMESSAGE

        self.motor_state = MotorStates.MOTOR_EN
        # Desired setpoints for PIDs
        self.l_desired_speed = 0
        self.r_desired_speed = 0   
        
        self.positional_information.azimuth = self.get_azimuth()
        
        self.desired_azimuth = None # Used to track to correct azimuthal angle in a rotation operation
        self.desired_distance = None # Used to go the correct distance forwards in a straight-line operation

        self.lockout_countdown = 0
        
        # Panel Information
        self.corner_locations = [] # X,Y location of all corners
        self.panel_height = 0 # Panel height measured
        self.panel_width = 0 # Panel width measured
        self.panel_height_nodes = 0 # Number of panel height nodes
        self.panel_width_nodes = 0 # Number of panel width nodes
        self.cleaning_axis = "height" # Which direction (height/width) the rover is currently cleaning
    
    def simulate(self):
        self.update_sensors()
        self.update_motors()        
        self.update_position()
        self.make_decision()
    
    def set_desired_azimuth(self, desired_azimuth):
        """Tell the robot to move to a certain orientation"""
        self.desired_azimuth = desired_azimuth
    
    def update_turning_trajectory(self, use_sensors=True):
        if use_sensors:
            ref = self.estimated_pos
        else:
            ref = self.positional_information
        
        desired_speed = 0
        desired_turn_rate = self.orientation_pid.calculate(self.desired_azimuth, ref.azimuth, angular=True)
        
        # If the desired turn rate is low (low error), the left and right speeds are zero (ZUPT condition), and we are using sensors (real data tracking will never ZUPT)
        # if abs(desired_turn_rate) < 0.05 and ref.l_speed == 0 and ref.r_speed == 0 and use_sensors:
        #     ref.azimuth = self.desired_azimuth
        #     desired_speed, desired_turn_rate = (0,0)
            
            
        self.set_trajectory(desired_speed=desired_speed, desired_turn_rate=desired_turn_rate)
        
        return desired_speed, desired_turn_rate
    
    def update_positional_trajectory(self, use_sensors=True,direction=1):

        if use_sensors:
            ref = self.estimated_pos
        else:
            ref = self.positional_information

        self.desired_distance = self.desired_distance - ref.linear_velocity * self.time_step
        desired_speed = self.positional_pid.calculate(0, -1*self.desired_distance)

        if direction*self.desired_distance < 0:
            self.set_trajectory(0,0)
            return

        desired_turn_rate = self.orientation_pid.calculate(self.desired_azimuth, ref.azimuth, angular=True)
        self.set_trajectory(desired_speed, desired_turn_rate)
        return desired_speed, desired_turn_rate
        
    def set_desired_distance(self, desired_distance, use_sensors=True):
        if use_sensors: 
            ref = self.estimated_pos
        else:
            ref = self.positional_information
        self.desired_distance = desired_distance
        self.desired_azimuth = ref.azimuth
        pass
        
    def get_limit_switch_readout(self, display=False, type: Literal["state", "pos"] = "state"):
        """Get full limit switch readout in array form.
        
        Ordering is, in order of priority: Upper, Left, Outer (i.e., LFO, LFI, RFO, ...) indexed 0-7

        Args:
            display (bool, optional): Whether to print out the whole return dataset. Defaults to False.
            type (Literal["state", "pos"], optional): Return state (True/False) or position (np.array([x,y])) for each switch. Defaults to "state".

        Returns:
            list: list of return values
        """
        # Keep default ordering, copied from Rover initialization
        if type=='pos':
            lfo_pos = self.limit_switch_lfo.get_position(self.positional_information.position, self.positional_information.azimuth)
            lfi_pos = self.limit_switch_lfi.get_position(self.positional_information.position, self.positional_information.azimuth)
            rfo_pos = self.limit_switch_rfo.get_position(self.positional_information.position, self.positional_information.azimuth)
            rfi_pos = self.limit_switch_rfi.get_position(self.positional_information.position, self.positional_information.azimuth)
            lbo_pos = self.limit_switch_lbo.get_position(self.positional_information.position, self.positional_information.azimuth)
            lbi_pos = self.limit_switch_lbi.get_position(self.positional_information.position, self.positional_information.azimuth)
            rbo_pos = self.limit_switch_rbo.get_position(self.positional_information.position, self.positional_information.azimuth)
            rbi_pos = self.limit_switch_rbi.get_position(self.positional_information.position, self.positional_information.azimuth)
            
            ret_list = [
                lfo_pos,
                lfi_pos,
                rfo_pos,
                rfi_pos,
                lbo_pos,
                lbi_pos,
                rbo_pos,
                rbi_pos
            ]
            
            if display:
                print("lfo_pos: ", lfo_pos)
                print("lfi_pos: ", lfi_pos)
                print("rfo_pos: ", rfo_pos)
                print("rfi_pos: ", rfi_pos)
                print("lbo_pos: ", lbo_pos)
                print("lbi_pos: ", lbi_pos)
                print("rbo_pos: ", rbo_pos)
                print("rbi_pos: ", rbi_pos)
            return ret_list
                
        if type=='state':
            lfo_state = self.limit_switch_lfo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            lfi_state = self.limit_switch_lfi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            rfo_state = self.limit_switch_rfo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            rfi_state = self.limit_switch_rfi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            lbo_state = self.limit_switch_lbo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            lbi_state = self.limit_switch_lbi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            rbo_state = self.limit_switch_rbo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            rbi_state = self.limit_switch_rbi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
            
            ret_list = [
                lfo_state,
                lfi_state,
                rfo_state,
                rfi_state,
                lbo_state,
                lbi_state,
                rbo_state,
                rbi_state
            ]
            
            if display:
                print("lfo_state: ", lfo_state)
                print("lfi_state: ", lfi_state)
                print("rfo_state: ", rfo_state)
                print("rfi_state: ", rfi_state)
                print("lbo_state: ", lbo_state)
                print("lbi_state: ", lbi_state)
                print("rbo_state: ", rbo_state)
                print("rbi_state: ", rbi_state)
            return ret_list            
       
    def get_azimuth(self):
        """Get rover azimuth angle from its orientation vector

        Returns:
            float: angle (rad)
        """
        return (np.atan2(self.positional_information.orientation[1], self.positional_information.orientation[0])) % (2*np.pi)
    
    def compute_trajectory(self, use_sensors=True):
        """Compute trajectory (linear velocity, turn rate), either using sensor data or real data

        Args:
            use_sensors (bool, optional): _description_. Defaults to True.

        Returns:
            _type_: _description_
        """
        if use_sensors:
            l_speed = self.estimated_pos.l_speed
            r_speed = self.estimated_pos.r_speed
        else:
            l_speed = self.positional_information.l_speed
            r_speed = self.positional_information.r_speed
        linear_velocity = (l_speed + r_speed) * self.wheel_radius / 2
        turn_rate = (r_speed - l_speed) * self.wheel_radius / self.axle_length
        return linear_velocity, turn_rate
        
    def update_position(self):
        """Update the rover's actual oposition. 
        Update is based off of REAL values for left and right track velocity, as obtained from motor

        Returns:
            PositionalInformation: position object
        """
        # Get track speeds and accelerations (angular, rad)
        l_speed = self.track_motor_l.get_speed()
        r_speed = self.track_motor_r.get_speed()
        self.positional_information.l_speed = l_speed
        self.positional_information.r_speed = r_speed
        l_accel = self.track_motor_l.get_angular_acceleration()
        r_accel = self.track_motor_r.get_angular_acceleration()

        # Compute the center of mass velocities and turn rates and accelerations
        linear_velocity, turn_rate = self.compute_trajectory(use_sensors=False)
        
        linear_accel = (l_accel + r_accel) * self.wheel_radius / 2
        turn_accel = (l_accel - r_accel) * self.wheel_radius / 2
        
        self.positional_information.linear_accel = linear_accel
        self.positional_information.linear_velocity = linear_velocity
        self.positional_information.turn_rate = turn_rate
        self.positional_information.turn_accel = turn_accel  
        
        # Compute full travel
        self.positional_information.position, self.positional_information.orientation, self.positional_information.azimuth = self.compute_travel(use_sensors=False)
        
        return self.positional_information

    def compute_travel(self, use_sensors=True):
        """For this to be accurate, you must have updated:
                1. turn rate
                2. linear velocity
                3. r_speed
                4. l_speed

        Args:
            use_sensors (bool, optional): _description_. Defaults to True.

        Returns:
            _type_: _description_
        """
        if use_sensors:
            ref = self.estimated_pos
        else:
            ref = self.positional_information
        
        # Compute the distance amount
        d_theta = ref.turn_rate * self.time_step # rotation amount, radians
        d_s = ref.linear_velocity * self.time_step # distance amount, m
        
        # Rotate d_theta degrees
        azimuth = (ref.azimuth + d_theta) % (2*np.pi)

        x, y = ref.orientation
        new_x = x*np.cos(d_theta) - y*np.sin(d_theta)
        new_y = x*np.sin(d_theta) + y*np.cos(d_theta)
        orientation = np.array([new_x, new_y])

        # If the track speeds are equal, just move forwards
        if ref.r_speed == ref.l_speed:
            # Prevents a divide by zero error
            turn_radius = 0
            d_theta = 0
            position = ref.position + ref.orientation * d_s

        # If the track speeds are not equal, move along the perimiter of a circle
        else:
            # Find the turn radius
            turn_radius = self.axle_length/2 * (ref.r_speed + ref.l_speed) / (ref.r_speed - ref.l_speed)

            # displacement vector 
            displacement_vector = np.array([x*np.cos(d_theta/2) - y*np.sin(d_theta/2), x*np.sin(d_theta/2) + y*np.cos(d_theta/2)])  * 2*turn_radius*np.sin(d_theta/2)
            position = ref.position + displacement_vector
                        
        return position, orientation, azimuth

    def set_trajectory(self, desired_speed, desired_turn_rate):
        """
        Sets the desired velocity of the left and right motors to match the set trajectory

        Args:
            desired_speed (float): desired forward velocity in m/s (limit to ~0.1)
            desired_turn_rate (float): desired turn rate in rad/s
        """
        # from the equations: 
        # Vavg = r (wl + wr) / 2
        # w = r (wr - wl) / 
        self.l_desired_speed = (2*desired_speed - (self.axle_length * desired_turn_rate)) / self.wheel_radius
        self.r_desired_speed = (2*desired_speed + (self.axle_length * desired_turn_rate)) / self.wheel_radius
        return self.l_desired_speed, self.r_desired_speed    

    def update_motors(self, use_sensors=True):
        """Update motors, following PID logic from the setpoints calculated from set_trajectory()

        Args:
            use_sensors (bool, optional): Whether to use sensor information (estimated position) as positional reference for the PID measurement calcuation. Defaults to True.
                \\ Turn this FALSE to validate rover components separate of measurement error

        Returns:
            tuple: left and right control voltages (not including cap due to motor voltage limits)
        """
        if use_sensors:
            pos_ref = self.estimated_pos
        else:
            pos_ref = self.positional_information
        # For now, using absolute information on speed
        if self.motor_state == MotorStates.MOTOR_DIS:
            self.track_motor_l.update(0)
            self.track_motor_r.update(0)
            return 0, 0
        control_voltage_l = self.track_pid_l.calculate(self.l_desired_speed, pos_ref.l_speed)
        control_voltage_r = self.track_pid_r.calculate(self.r_desired_speed, pos_ref.r_speed)
        self.track_motor_l.update(control_voltage_l)
        self.track_motor_r.update(control_voltage_r)
        return control_voltage_l, control_voltage_r

    def update_sensors(self):
        """Updates sensor readings AND generates new self.estimated_pos readings
        """
        # NOTE: encoder velocity derivation step is included in the RotaryEncoder method, instead of the Rover. This should be moved but it works for now
        l_enc_speed = self.rotary_encoder_l.get_track_velocity(self.positional_information.l_speed)[0]
        r_enc_speed = self.rotary_encoder_r.get_track_velocity(self.positional_information.r_speed)[0]
        
        self.estimated_pos.l_speed = l_enc_speed
        self.estimated_pos.r_speed = r_enc_speed
        self.estimated_pos.linear_velocity, self.estimated_pos.turn_rate = self.compute_trajectory()
        

        # IMU estimates
        self.imu_info = self.imu.get_imu_data(self.positional_information, body_axis=True)
        # self.estimated_pos.linear_accel = self.imu_info[0][0][0]
        self.estimated_pos.turn_rate = self.imu_info[1][2][0]


        if l_enc_speed == 0 and r_enc_speed == 0 and self.estimated_pos.linear_accel <= 0.25:
            self.estimated_pos.linear_accel = 0
            self.estimated_pos.linear_velocity = 0


        else:
            self.estimated_pos.linear_velocity = self.estimated_pos.linear_velocity + self.estimated_pos.linear_accel*self.time_step
            self.estimated_pos.position = self.estimated_pos.position + self.estimated_pos.orientation * self.estimated_pos.linear_velocity * self.time_step
        _, self.estimated_pos.orientation, self.estimated_pos.azimuth = self.compute_travel()
        
        pass
    
    def get_actual_data(self):
        #update position,velocity,acceleration,
        #get rover center point
        return self.positional_information

    def get_sensor_data(self):
        """
        NOTE: CHANGE THE LOGIC HERE. Many of the sensors are time-dependent (i.e., making a reading of the position assumes a time-step has happened and will affect derivatives and integrals of values)
        Changed this to a get method for estimated position based on currently updated sensor positions.
        CONCRETE ACTION
        Fetch sensor readings for decision-making."""
        # imu_data = self.imu.get_imu_data()
        # encoder_l_pos = self.rotary_encoder_l.get_track_velocity()
        # encoder_r_pos = self.rotary_encoder_r.get_track_velocity()
        # limit_switches = [switch.is_pressed() for switch in self.limit_switch_array]
        # return imu_data, encoder_l_pos, encoder_r_pos, limit_switches
        return self.estimated_pos

    def set_radio_message(self, radio_message):
        """CONCRETE ACTION"""
        """Set the current radio message."""
        self.radio_message = radio_message

    def search_for_corner(self):
        """DECISION TREE LEVEL (2)"""
        match self.search_for_corner_state:
            case SearchForCornerStates.MOVEBACKWARDSUNTILEDGE:
                self.move_backward_until_edge()
            case SearchForCornerStates.ALIGNWITHEDGE:
                self.align_with_edge()
            case SearchForCornerStates.TURNRIGHT:
                self.turn_right(90)
            case SearchForCornerStates.ADJUSTBACKANDFORTH:
                self.adjust_back_and_forth()
            case SearchForCornerStates.MOVEBACKWARDSUNTILCORNER:
                self.move_backward_until_corner()
            case SearchForCornerStates.DONE:
                self.decision_state = DecisionStates.BEGINCLEANING
 
        #self.decision_state = DecisionStates.BEGINCLEANING -> move this inside move backwards until corner

    def initialize_cleaning(self):
        """DECISION TREE LEVEL (2)"""
        # self.imu.reset_position()
        self.cleaning_motors.power(True)
        self.decision_state = DecisionStates.CLEANOUTERLOOP

    def clean_outer_loop(self):
        """DECISION TREE LEVEL (2)"""
        match self.outer_loop_states:
            case OuterLoopStates.FOLLOWEDGE:
                self.follow_edge()
            case OuterLoopStates.TURNLEFT:
                self.turn_left(90)
            case OuterLoopStates.REVERSE:
                self.outer_loop_reverse()
            case OuterLoopStates.DONE:
                self.decision_state = DecisionStates.CLEANINNERLOOPS
    
    def outer_loop_reverse(self):
        if self.desired_distance == None:
            self.set_desired_distance(-1*self.axle_length*0.75)
            self.set_desired_azimuth(self.estimated_pos.azimuth)
        self.update_positional_trajectory(direction=-1)
        if self.desired_distance >= 0:
            self.desired_distance = None
            self.desired_azimuth = None
            self.outer_loop_states = OuterLoopStates.TURNLEFT

    def clean_inner_loops(self):
        """DECISION TREE LEVEL (2)"""
        match self.inner_loop_states:
            case InnerLoopStates.FOLLOWPATH:
                self.follow_inner_path()
            case InnerLoopStates.TURNLEFT:
                self.turn_left(90)
            case InnerLoopStates.DONE:
                self.decision_state = DecisionStates.DONE

    def when_done(self):
        """DECISION TREE LEVEL (2)"""
        self.set_radio_message(RadioMessage.CLEANINGDONETAKEMEAWAY)
        self.stop_motors()
        self.decision_state = DecisionStates.DONE

    def make_decision(self):
        """DECISION TREE HEAD (1)"""
        if self.radio_message == RadioMessage.NOMESSAGE:
            return
        if self.motor_state == MotorStates.MOTOR_DIS:
            self.lockout_countdown = self.lockout_countdown - 1
            if self.lockout_countdown <= 0:
                self.motor_state = MotorStates.MOTOR_EN
            return        
        elif self.radio_message == RadioMessage.STARTCLEANINGOK and self.decision_state == DecisionStates.IDLE:
            self.decision_state = DecisionStates.SEARCHFORCORNER

        # check state tied to countdown
        # stopmotor()
        #early return
        #if opposing limit switch pairs are on, stop

        match self.decision_state:
            case DecisionStates.SEARCHFORCORNER:
                self.search_for_corner()
            case DecisionStates.BEGINCLEANING:
                self.initialize_cleaning()
            case DecisionStates.CLEANOUTERLOOP:
                self.clean_outer_loop()
            case DecisionStates.CLEANINNERLOOPS:
                self.clean_inner_loops()
            case DecisionStates.DONE:
                self.when_done()

    def move_backward_until_edge(self):
        """CONCRETE ACTION"""
        """Move backward with gradually increasing speed until any back limit switch is triggered."""
        
        max_speed = 0.75 * self.top_speed  # 75% of max speed

        # UNUSED -- Handled by PID functions, don't try to apply external acceleration-like inputs to it
        # Gradually ramp up speed
        # if self.l_desired_speed > -max_speed:
        #     self.l_desired_speed -= 0.01  # Gradual increase in reverse
        # if self.r_desired_speed > -max_speed:
        #     self.r_desired_speed -= 0.01  

        # Update trajectory to move backward
        self.set_trajectory(desired_speed=-1*max_speed, desired_turn_rate=0)

        # Check back limit switches
        back_switches = [
            self.limit_switch_lbo.is_pressed(self.positional_information.position, self.positional_information.azimuth),
            self.limit_switch_lbi.is_pressed(self.positional_information.position, self.positional_information.azimuth),
            self.limit_switch_rbo.is_pressed(self.positional_information.position, self.positional_information.azimuth),
            self.limit_switch_rbi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        ]

        # Stop immediately if any back limit switch is off the panel
        if not all(back_switches):
            self.stop_motors()
            self.search_for_corner_state = SearchForCornerStates.ALIGNWITHEDGE


    def align_with_edge(self):
        """CONCRETE ACTION"""
        """Align with the panel edge using back limit switches and small adjustments."""

        # Read back limit switch states
        left_back_off = not self.limit_switch_lbo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        right_back_off = not self.limit_switch_rbo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        # If both back limit switches are off, we are aligned
        if left_back_off and right_back_off:
            self.search_for_corner_state = SearchForCornerStates.TURNRIGHT
            self.stop_motors()            
            return
        
        # If only the left back switch is off, rotate counterclockwise (CCW)
        if left_back_off:
            self.set_trajectory(desired_speed=0.01, desired_turn_rate=-0.1)  # Small forward + left turn

        # If only the right back switch is off, rotate clockwise (CW)
        elif right_back_off:
            self.set_trajectory(desired_speed=0.01, desired_turn_rate=0.1)  # Small forward + right turn
        
        else:
            self.set_trajectory(desired_speed=-0.01,desired_turn_rate=0)


    def turn_right(self, deg=90):
        """CONCRETE ACTION"""
        """Rotate the rover clockwise (right) by a given degree amount."""
        # Convert degrees to radians
        if self.desired_azimuth == None:
            target_azimuth = (self.estimated_pos.azimuth - np.radians(deg)) % (2 * np.pi)
            self.set_desired_azimuth(target_azimuth)

        # Update the turning trajectory
        self.update_turning_trajectory(use_sensors=True)

        # Stop when the turn is completed
        if abs(self.estimated_pos.azimuth - self.desired_azimuth) < 0.05:  # Small error threshold
            self.desired_azimuth = None
            self.stop_motors()
            self.search_for_corner_state = SearchForCornerStates.ADJUSTBACKANDFORTH

    def turn_left(self,deg):
        """CONCRETE ACTION"""
        """Rotate the rover clockwise (right) by a given degree amount."""
        
        # Convert degrees to radians
        if self.desired_azimuth == None:
            target_azimuth = (self.estimated_pos.azimuth + np.radians(deg)) % (2 * np.pi)
            self.set_desired_azimuth(target_azimuth)

        # Update the turning trajectory
        self.update_turning_trajectory(use_sensors=True)

        # Stop when the turn is completed
        if abs(self.estimated_pos.azimuth - self.desired_azimuth) < 0.01:  # Small error threshold
            self.desired_azimuth = None
            self.stop_motors()
            if self.outer_loop_states == OuterLoopStates.TURNLEFT:
                # This is the last turn, after the reversal
                if len(self.corner_locations) == 5:
                    self.outer_loop_states = OuterLoopStates.DONE
                    return
                self.outer_loop_states = OuterLoopStates.FOLLOWEDGE
            elif self.inner_loop_states == InnerLoopStates.TURNLEFT:
                # Decrease the number of nodes in the current axis by 1, and flip the cleaning axis
                if self.cleaning_axis == "height":
                    self.panel_height_nodes -=1
                    self.cleaning_axis = "width"
                else:
                    self.panel_width_nodes -=1
                    self.cleaning_axis = "height"                
                self.inner_loop_states = InnerLoopStates.FOLLOWPATH


    def adjust_back_and_forth(self):
        """CONCRETE ACTION"""
        """Make small adjustments until back limit switches detect the edge correctly."""

        # Read limit switch states
        rfo = self.limit_switch_rfo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        rfi = self.limit_switch_rfi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        rbo = self.limit_switch_rbo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        rbi = self.limit_switch_rbi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        
        if not (rbo or rbi):
            # if both back switches are off
            if not (rfo or rfi):
                # all switches are off, forward and CCW
                self.set_trajectory(0.02,0.1)
            else: 
                # One or both front switches are on, forward and CW
                self.set_trajectory(0.02,-0.1)
                
        elif (rbo and rbi):
            # if both back switches are on, go back and CCW no matter what
            self.set_trajectory(-0.02,0.1)
        
        else:
            # if back switch one is off and one is on:
            if (rfo and rfi):
                # if both front switches are on, reverse straight back
                self.set_trajectory(-0.02,0)
            if not (rfo or rfi):
                # if neither front switches are on, reverse straight back
                self.set_trajectory(-0.02,0)
            else:        
                self.stop_motors()
                self.search_for_corner_state = SearchForCornerStates.MOVEBACKWARDSUNTILCORNER


    def move_backward_until_corner(self):
        """CONCRETE ACTION"""
        """Move backwards until corner is detected by the limit switches."""

        self.set_trajectory(desired_speed=-0.05, desired_turn_rate=0)

        # Check for corner condition: both left and right back outer switches pressed
        if not self.limit_switch_lbo.is_pressed(self.positional_information.position, self.positional_information.azimuth) and \
        not self.limit_switch_rbo.is_pressed(self.positional_information.position, self.positional_information.azimuth):
            self.stop_motors()
            self.corner_locations.append(self.estimated_pos.position)
            self.search_for_corner_state = SearchForCornerStates.DONE
            
    def map_inner_path(self):
        # Maps the inner path using collected data
        
        self.panel_height = np.average([np.linalg.norm(self.corner_locations[0] - self.corner_locations[1]), np.linalg.norm(self.corner_locations[2] - self.corner_locations[3])])
        self.panel_width = np.average([np.linalg.norm(self.corner_locations[1] - self.corner_locations[2]), np.linalg.norm(self.corner_locations[3] - self.corner_locations[0])])
        self.panel_height_nodes = int(self.panel_height//(self.axle_length*0.75)) - 1 # -2 but round up, so turns out to -1
        self.panel_width_nodes = int(self.panel_width//(self.axle_length*0.75)) - 2 # -3 but round up, so turns out to -2
        
    def follow_edge(self):
        """CONCRETE ACTION"""
        """Follow the edge of the solar panel while maintaining alignment."""

        speed = 0.03  # Move forward slowly

        # Read limit switch states
        rfo = self.limit_switch_rfo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        rfi = self.limit_switch_rfi.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        lfo = self.limit_switch_lfo.is_pressed(self.positional_information.position, self.positional_information.azimuth)
        lfi = self.limit_switch_lfi.is_pressed(self.positional_information.position, self.positional_information.azimuth)

        # If the left front switches went off, we reached a corner
        if not (lfo and lfi):
            self.stop_motors()
            # If this is the last corner
            self.corner_locations.append(self.estimated_pos.position)

            if len(self.corner_locations) == 5:
                # Compute the number of inner nodes
                self.map_inner_path()
                self.outer_loop_states = OuterLoopStates.REVERSE # Need to reverse slightly
            else:
                self.outer_loop_states = OuterLoopStates.TURNLEFT
            return
        
        # Adjust turn rate based on limit switch readings
        if not rfi:
            # inner limit switch went off, adjust slight CCW
            self.set_trajectory(speed/2,0.1)
        elif rfo:
            # outer limit switch went on, adjust slight CW
            self.set_trajectory(speed/2,-0.1)
        else:
            self.set_trajectory(speed, 0)
        
        
    def follow_inner_path(self):
        """CONCRETE ACTION"""
        """Follow the inner cleaning path using IMU for alignment."""
        # Set the desired distance to travel based on the height and width of the panel
        
        if not all(self.get_limit_switch_readout()[:3]):
            self.stop_motors()
            self.radio_message = RadioMessage.ERROR
        
        if self.desired_distance == None:
            if self.cleaning_axis == "height":
                distance = self.panel_height_nodes*self.axle_length*0.75
            elif self.cleaning_axis == "width":
                distance = self.panel_width_nodes*self.axle_length*0.75
            if distance == 0:
                self.inner_loop_states = InnerLoopStates.DONE
                self.stop_motors()
                return
            self.set_desired_distance(distance)
            self.set_desired_azimuth(self.estimated_pos.azimuth)
        self.update_positional_trajectory()
        
        if self.desired_distance <=0:
            self.desired_distance = None
            self.desired_azimuth = None
            self.stop_motors()
            self.inner_loop_states = InnerLoopStates.TURNLEFT

    def stop_motors(self): 
        """CONCRETE ACTION"""
        self.track_motor_l.update(0)
        self.track_motor_r.update(0)
        self.track_pid_l.reset()
        self.track_pid_r.reset()
        self.motor_state = MotorStates.MOTOR_DIS
        self.lockout_countdown = LOCKOUT_COUNTDOWN

    
 
            



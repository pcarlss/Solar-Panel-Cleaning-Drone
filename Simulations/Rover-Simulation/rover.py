from components import IMU, LimitSwitch, RotaryEncoder, TrackMotor, CleaningMotor, SimpleMotor, DCMotorDiscrete, PIDController
from common import DecisionStates, InnerLoopStates, OuterLoopStates, RadioMessage, SearchForCornerStates
from dataclasses import dataclass
from typing import List, Literal
from scipy.ndimage import rotate
import numpy as np


@dataclass()
class PositionalInformation():
    position: np.ndarray[2, float] # XY position, (m,m)
    orientation: np.ndarray[2, float] # Unit vector orientation
    distance_track: float = 0 # Variable to track current forward distance travelled in a single "go straight" operation, m
    azimuth: float = 0 # Actual azimuth angle, same as unit vector orientation, rad
    linear_velocity: float = 0 # Linear forward velocity, m/s
    turn_rate: float = 0 # Turn rate, rad/s
    linear_accel: float = 0 # Linear forward acceleration, m/s^2
    turn_accel: float = 0 # Turning acceleration, rad/s^2
    l_speed: float = 0 # Left track speed, rad/s
    r_speed: float = 0 # Right track speed, rad/s

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
            orientation=np.array([0,1]))

        # Physical constants
        self.axle_length = 0.170 #170mm
        self.wheel_radius = 0.025 #25mm
        self.top_speed = 1 # 1 m/s
        self.top_turn_rate = 1 # rad/s

        # Simulation constants
        self.time_step = time_step

        # Sensor definitions
        self.imu = IMU()
        self.rotary_encoder_l = RotaryEncoder(resolution=20, time_step = self.time_step, zero_time=0.25)
        self.rotary_encoder_r = RotaryEncoder(resolution=20, time_step = self.time_step, zero_time=0.25)
        
        # Limit switches identified by last three chars: l/r for left/right, f/b for front/back, i/o for inboard/outboard
        self.limit_switch_lfo = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,0.105]))
        self.limit_switch_lfi = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,0.1]))
        self.limit_switch_rfo = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,0.105]))
        self.limit_switch_rfi = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,0.1]))
        self.limit_switch_lbo = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,-0.105]))
        self.limit_switch_lbi = LimitSwitch(solar_panel_area, relative_pos=np.array([-0.1,-0.1]))
        self.limit_switch_rbo = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,-0.105]))
        self.limit_switch_rbi = LimitSwitch(solar_panel_area, relative_pos=np.array([0.1,-0.1]))
        
        self.track_motor_l = DCMotorDiscrete(K=1.14, dt=time_step)
        self.track_motor_r = DCMotorDiscrete(K=1.14, dt=time_step)
        self.cleaning_motors = CleaningMotor()

        self.track_pid_l = PIDController(Kp=0.15, Ki=3.5, Kd=0.025, dt=time_step)
        self.track_pid_r = PIDController(Kp=0.15, Ki=3.5, Kd=0.025, dt=time_step)
        
        self.orientation_pid = PIDController(Kp=0.5, Ki=0.2, Kd=0.05, dt=time_step)
        self.turn_compensation_factor = 1.08 # determined through trial and error 

        # Decision States
        self.decision_state = DecisionStates.IDLE
        self.search_for_corner_state = SearchForCornerStates.MOVEBACKWARDSUNTILEDGE
        self.outer_loop_states = OuterLoopStates.FOLLOWEDGE
        self.inner_loop_states = InnerLoopStates.FOLLOWPATH
        self.radio_message = RadioMessage.NOMESSAGE

        # Desired setpoints for PIDs
        self.l_desired_speed = 0
        self.r_desired_speed = 0   
        
        self.positional_information.azimuth = self.get_azimuth()
        
        self.desired_azimuth = self.positional_information.azimuth # Used to track to correct azimuthal angle in a rotation operation
        self.desired_distance = 0 # Used to go the correct distance forwards in a straight-line operation
        
    def set_desired_azimuth(self, desired_azimuth):
        """Tell the robot to move to a certain orientation"""
        self.desired_azimuth = desired_azimuth
        pass
    
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

    def set_desired_distance(self, desired_distance):
        self.desired_distance = desired_distance
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
        
        self.estimated_pos.position, self.estimated_pos.orientation, self.estimated_pos.azimuth = self.compute_travel()
        
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
        self.imu.reset_position()
        self.cleaning_motors.start()
        self.decision_state = DecisionStates.CLEANOUTERLOOP

    def clean_outer_loop(self):
        """DECISION TREE LEVEL (2)"""
        match self.outer_loop_states:
            case OuterLoopStates.FOLLOWEDGE:
                self.follow_edge()
            case OuterLoopStates.TURNLEFT:
                self.turn_left(90)
            case OuterLoopStates.DONE:
                self.decision_state = DecisionStates.CLEANINNERLOOPS

    def clean_inner_loops(self):
        """DECISION TREE LEVEL (2)"""
        match self.inner_loop_states:
            case InnerLoopStates.FOLLOWEDGE:
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
        elif self.radio_message == RadioMessage.STARTCLEANINGOK:
            self.decision_state = DecisionStates.SEARCHFORCORNER

      
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
        #track1 backup on gradual pickup speed to 25% of max speed
        #track2 backup on gradual pickup speed to 25% of max speed
        # if any back limit switch hits, stop immediately
        self.set_trajectory()
        pass

    def align_with_edge(self):
        """CONCRETE ACTION"""
        #if both back limit switch hits, stop
        #if back right limit switch pair detects edge, stop, rotate ccw (might have to shift a lil fwd)
        #if back left limit switch pair detects edge, stop, rotate cw
        #adjust fwd and back so that the limit switches are on the edge
        pass

    def turn_right(self,deg):
        """CONCRETE ACTION"""
        #turn cw (might have to shift a lil fwd)
        pass
    def turn_left(self,deg):
        """CONCRETE ACTION"""
        #turn ccw (might have to shift a lil fwd)
        pass

    def adjust_back_and_forth(self):
        """CONCRETE ACTION"""
        #adjust back and forth a lil bit until half of the pair of back sensors read panel, other half reads outside
        pass

    def move_backward_until_corner(self):
        """CONCRETE ACTION"""
        #track1 & track2 backup until hits corner
        pass

    def follow_edge(self):
        """CONCRETE ACTION"""
        #move fwd while maintaining alignment with limit switches
        #use imu to validate alignment & turning angles
        #track distance traveled
        pass

    def follow_inner_path(self):
        """CONCRETE ACTION"""
        #use imu to verify alignment
        #track distance traveled
        pass

    def stop_motors(self): 
        """CONCRETE ACTION"""
        self.cleaning_motors.stop()
        self.track_motor_l.stop()
        self.track_motor_r.stop()
    
 
            



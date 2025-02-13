from components import IMU, LimitSwitch, RotaryEncoder, TrackMotor, CleaningMotor, SimpleMotor, DCMotorDiscrete, PIDController
from common import DecisionStates, InnerLoopStates, OuterLoopStates, RadioMessage, SearchForCornerStates
from dataclasses import dataclass
from typing import List
from scipy.ndimage import rotate
import numpy as np


@dataclass()
class PositionalInformation():
    position: np.ndarray[2, float]
    orientation: np.ndarray[2, float]
    linear_velocity: float = 0
    turn_rate: float = 0
    linear_accel: float = 0
    turn_accel: float = 0

class Rover:
    def __init__(self, solar_panel_area, time_step):

        # X,Y,Theta and their derivatives
        self.positional_information = PositionalInformation(
            position=np.array([0,0]), 
            orientation=np.array([1,0]))

        self.imu = IMU()
        self.rotary_encoder_l = RotaryEncoder()
        self.rotary_encoder_r = RotaryEncoder()
        self.limit_switch_array = [LimitSwitch(solar_panel_area) for _ in range(8)]
        self.track_motor_l = DCMotorDiscrete(K=1.14, dt=time_step)
        self.track_motor_r = DCMotorDiscrete(K=1.14, dt=time_step)
        self.cleaning_motors = CleaningMotor()

        self.track_pid_l = PIDController(Kp=3, Ki=10, Kd=0.05, dt=time_step)
        self.track_pid_r = PIDController(Kp=3, Ki=10, Kd=0.05, dt=time_step)        

        #decision states
        self.decision_state = DecisionStates.IDLE
        self.search_for_corner_state = SearchForCornerStates.MOVEBACKWARDSUNTILEDGE
        self.outer_loop_states = OuterLoopStates.FOLLOWEDGE
        self.inner_loop_states = InnerLoopStates.FOLLOWPATH
        self.radio_message = RadioMessage.NOMESSAGE

        #physical constants
        self.axle_length = 0.170 #170mm
        self.wheel_radius = 0.025 #25mm
        self.top_speed = 1 # 1 m/s
        self.top_turn_rate = 1 # rad/s

        #Simulation constants
        self.time_step = time_step

        #desired setpoints for various things
        self.l_desired_speed = 0
        self.r_desired_speed = 0        

    def update_position(self):
        l_speed = self.track_motor_l.get_speed()
        r_speed = self.track_motor_r.get_speed()

        l_accel = self.track_motor_l.get_angular_acceleration()
        r_accel = self.track_motor_r.get_angular_acceleration()

        linear_velocity = (l_speed + r_speed) * self.wheel_radius / 2
        turn_rate = (r_speed - l_speed) * self.wheel_radius / self.axle_length

        turn_radius = self.axle_length/2 * (r_speed + l_speed) / (r_speed - l_speed)

        linear_accel = (l_accel + r_accel) * self.wheel_radius / 2
        turn_accel = (l_accel - r_accel) * self.wheel_radius / 2

        d_theta = turn_rate * self.time_step # rotation amount, radians
        d_s = linear_velocity * self.time_step # distance amount, m

        # Rotate d_theta degrees
        x, y = self.positional_information.orientation
        new_x = x*np.cos(d_theta) - y*np.sin(d_theta)
        new_y = x*np.sin(d_theta) + y*np.cos(d_theta)
        self.positional_information.orientation = np.array([new_x, new_y])


        # Move d_s distance along a circle
        # self.positional_information.position = self.positional_information.position + self.positional_information.orientation * d_s
        # This is equivalent for some reason???
        dist_x = x*np.cos(d_theta/2) - y*np.sin(d_theta/2)
        dist_y = x*np.sin(d_theta/2) + y*np.cos(d_theta/2)
        
        self.positional_information.position = self.positional_information.position + np.array([dist_x, dist_y]) * 2*turn_radius*np.sin(d_theta/2)

        # Update the rest of the values

        self.positional_information.linear_accel = linear_accel
        self.positional_information.linear_velocity = linear_velocity
        self.positional_information.turn_rate = turn_rate
        self.positional_information.turn_accel = turn_accel

        return self.positional_information

    def set_trajectory(self, desired_speed, desired_turn_rate):
        """
        Sets the velocity of the left and right motors

        Args:
            desired_speed (_type_): _description_
            desired_turn_rate (_type_): _description_
        """
        # from the equations: 
        # Vavg = r (wl + wr) / 2
        # w = r (wr - wl) / 
        self.l_desired_speed = (2*desired_speed - (self.axle_length * desired_turn_rate)) / self.wheel_radius
        self.r_desired_speed = (2*desired_speed + (self.axle_length * desired_turn_rate)) / self.wheel_radius
        return self.l_desired_speed, self.r_desired_speed    

    def update_motors(self):
        # For now, using absolute information on speed
        control_voltage_l = self.track_pid_l.calculate(self.l_desired_speed, self.track_motor_l.get_speed())
        control_voltage_r = self.track_pid_r.calculate(self.r_desired_speed, self.track_motor_r.get_speed())
        self.track_motor_l.update(control_voltage_l)
        self.track_motor_r.update(control_voltage_r)
        return control_voltage_l, control_voltage_r


    def update_sensors(self):
        pass
    
    def get_actual_data(self):
        #update position,velocity,acceleration,
        #get rover center point
        return self.positional_information

    def get_sensor_data(self):
        """CONCRETE ACTION"""
        """Fetch sensor readings for decision-making."""
        imu_data = self.imu.get_imu_data()
        encoder_l_pos = self.rotary_encoder_l.get_track_velocity()
        encoder_r_pos = self.rotary_encoder_r.get_track_velocity()
        limit_switches = [switch.is_pressed() for switch in self.limit_switch_array]
        return imu_data, encoder_l_pos, encoder_r_pos, limit_switches

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
    
 
            



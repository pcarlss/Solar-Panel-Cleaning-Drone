from components import IMU, LimitSwitch, RotaryEncoder, TrackMotor, CleaningMotor, SimpleMotor, DCMotorDiscrete
from common import DecisionStates, InnerLoopStates, OuterLoopStates, RadioMessage, SearchForCornerStates


class Rover:
    def __init__(self, solar_panel_area, time_step):

        # X,Y,Theta and their derivatives
        self.positional_information = {
            "center_mass_position" : [0,0],
            "orientation" : [1,0], #unit vector
            "directional_vector" : [0,0], #should this be an unit vector or an adjustable vector (acceleration * orientation vector.... will leave to you @aidan)
            "linear_velocity": 0, #V, velocity of the center of mass
            "turn_rate": 0, #omega, d(theta)/dt of the orientation vector
            "linear_accel": 0, #Linear acceleration, dV/dt of the center of mass
            "turn_accel": 0 #alpha, d(omega)/dt of the orientation vector
        }

        self.imu = IMU()
        self.rotary_encoder_l = RotaryEncoder()
        self.rotary_encoder_r = RotaryEncoder()
        self.limit_switch_array = [LimitSwitch(solar_panel_area) for _ in range(8)]
        self.track_motor_l = SimpleMotor()
        self.track_motor_r = SimpleMotor()
        self.cleaning_motors = CleaningMotor()

        #decision states
        self.decision_state = DecisionStates.IDLE
        self.search_for_corner_state = SearchForCornerStates.MOVEBACKWARDSUNTILEDGE
        self.outer_loop_states = OuterLoopStates.FOLLOWEDGE
        self.inner_loop_states = InnerLoopStates.FOLLOWPATH
        self.radio_message = RadioMessage.NOMESSAGE

        #physical constants
        self.axle_length = 0.170 #170mm
        self.wheel_radius = 0.005 #5mm
        self.top_speed = 1 # 1 m/s
        self.top_turn_rate = 1 # rad/s

        #Simulation constants
        self.time_step = time_step

    def update_position(self):
        l_speed = self.track_motor_l.get_speed()
        r_speed = self.track_motor_l.get_speed()

        linear_speed = (l_speed + r_speed) / 2
        turn_rate = (l_speed - r_speed) / 2


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

    def update_motors(self):
        
        pass

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
    
 
            



from components import IMU, LimitSwitch, RotaryEncoder, TrackMotor, CleaningMotor, SimpleMotor, DCMotorDiscrete
from common import DecisionStates, InnerLoopStates, OuterLoopStates, RadioMessage, SearchForCornerStates


class Rover:
    def __init__(self, solar_panel_area):
        self.imu = IMU()
        self.rotary_encoder_1 = RotaryEncoder()
        self.rotary_encoder_2 = RotaryEncoder()
        self.limit_switch_array = [LimitSwitch() for _ in range(8)]
        self.track_motor_1 = TrackMotor()
        self.track_motor_2 = TrackMotor()
        self.cleaning_motors = CleaningMotor()

        self.decision_state = DecisionStates.IDLE
        self.search_for_corner_state = SearchForCornerStates.MOVEBACKWARDSUNTILEDGE
        self.outer_loop_states = OuterLoopStates.FOLLOWEDGE
        self.inner_loop_states = InnerLoopStates.FOLLOWEDGE
        self.radio_message = RadioMessage.NOMESSAGE

        self.axle_length = 0.170 #170mm
        self.wheel_radius = 0.005 #5mm


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

    def get_actual_data(self,solar_panel_area):
        pass

    def get_sensor_data(self):
        """Fetch sensor readings for decision-making."""
        imu_data = self.imu.get_data()
        encoder_1_pos = self.rotary_encoder_1.get_position()
        encoder_2_pos = self.rotary_encoder_2.get_position()
        limit_switches = [switch.is_pressed() for switch in self.limit_switch_array]
        return imu_data, encoder_1_pos, encoder_2_pos, limit_switches

    def set_radio_message(self, radio_message):
        """Set the current radio message."""
        self.radio_message = radio_message

    def search_for_corner(self):
        """Perform movements to find a corner of the solar panel."""
        match self.search_for_corner_state:
            case SearchForCornerStates.MOVEBACKWARDSUNTILEDGE:
                self.move_backward_until_edge()
            case SearchForCornerStates.ALIGNWITHEDGE:
                self.align_with_edge()
            case SearchForCornerStates.TURNRIGHT:
                self.turn_right(90)
            case SearchForCornerStates.MOVEFORWARDSLIGHTLY:
                self.move_forward_slightly()
            case SearchForCornerStates.MOVEBACKWARDSUNTILCORNER:
                self.move_backward_until_corner()
            case SearchForCornerStates.DONE:
                self.decision_state = DecisionStates.BEGINCLEANING
 
        #self.decision_state = DecisionStates.BEGINCLEANING -> move this inside move backwards until corner

    def initialize_cleaning(self):
        """Initialize IMU, start cleaning motors, and prepare for cleaning."""
        self.imu.reset_position()
        self.cleaning_motors.start()
        self.decision_state = DecisionStates.CLEANOUTERLOOP

    def clean_outer_loop(self):
        """Perform outer loop cleaning while tracking the panel edge."""
        match self.outer_loop_states:
            case OuterLoopStates.FOLLOWEDGE:
                self.follow_edge()
            case OuterLoopStates.TURNLEFT:
                self.turn_left(90)
            case OuterLoopStates.DONE:
                self.decision_state = DecisionStates.CLEANINNERLOOPS

    def clean_inner_loops(self):
        """Perform inner loop cleaning while maintaining alignment."""
        match self.inner_loop_states:
            case InnerLoopStates.FOLLOWEDGE:
                self.follow_inner_path()
            case InnerLoopStates.TURNLEFT:
                self.turn_left(90)
            case InnerLoopStates.DONE:
                self.decision_state = DecisionStates.DONE


    def when_done(self):
        """Send completion message and stop all movements."""
        self.set_radio_message(RadioMessage.CLEANINGDONETAKEMEAWAY)
        self.stop_motors()
        self.decision_state = DecisionStates.DONE

    def make_decision(self):
        """Control rover actions based on its current decision state."""
        if self.radio_message == RadioMessage.NOMESSAGE:
            return
        elif self.radio_message == RadioMessage.STARTCLEANINGOK:
            self.decision_state = DecisionStates.SEARCHFORCORNER

        match self.decision_state:
            case DecisionStates.IDLE:
                pass
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
        #track1 backup on gradual pickup speed to 25% of max speed
        #track2 backup on gradual pickup speed to 25% of max speed
        # if any back limit switch hits, stop immediately
        pass
    def align_with_edge(self):
        #if both back limit switch hits, stop
        #if back right limit switch pair detects edge, stop, rotate ccw (might have to shift a lil fwd)
        #if back left limit switch pair detects edge, stop, rotate cw
        #adjust fwd and back so that the limit switches are on the edge
        pass

    def turn_right(self,deg):
        #turn cw (might have to shift a lil fwd)
        pass
    def turn_left(self,deg):
        #turn ccw (might have to shift a lil fwd)
        pass

    def move_forward_slightly(self):
        pass

    def move_backward_until_corner(self):

        pass

    def follow_edge(self):
        pass

    def follow_inner_path(self):
        pass

    def stop_motors(self): 
        self.cleaning_motors.stop()
        self.track_motor_1.stop()
        self.track_motor_2.stop()
    
    def get_actual_data(self,solar_panel_area):
        pass

    def get_sensor_data(self):
        pass


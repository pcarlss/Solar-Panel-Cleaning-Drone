from Sensors.IMU import IMU
from Sensors.Motor import TrackMotor,CleaningMotor
from Sensors.LimitSwitch import LimitSwitch
from Sensors.RotaryEncoder import RotaryEncoder
from Common import *


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
        self.radio_message = RadioMessage.NOMESSAGE

    def get_actual_data(self,solar_panel_area):
        pass

    def get_sensor_data(self):
        pass

    def set_radio_message(self,radio_message):
        self.radio_message =radio_message

    def search_for_corner(self):
        #back up until edge
        # align rover ass to edge
        # turn right 90 deg (might have to move fwd slgihtly to avoid falling)
        # go backwards until hits corner, while staying clsoe to edge
        self.decision_state = DecisionStates.BEGINCLEANING
        pass

    def initialize_cleaning(self):
        #initialize IMU to 0,0
        #start cleaning motors
        #
        self.decision_state = DecisionStates.CLEANOUTERLOOP
        pass

    def clean_outer_loop(self):
        #go fwd while staying near the edge (use 2 edge detect array to stay at set distance from edge)
        #at edge, turn 90 deg left to face up, go straight & clean, repeat
        # validate with provided panel edge
        # use imu to validate orientation
        # if exit condition is met (aka traveled req distance)
        self.decision_state = DecisionStates.CLEANINNERLOOPS
        pass

    def clean_inner_loops(self):
        #go fwd while tracking distance (use 2 edge detect array to stay at set distance from edge)
        #at edge, turn 90 deg left to face up, go straight & clean, repeat
        # validate with provided panel edge
        # use imu to validate orientation
        # if exit condition is met (aka traveled req distance)
        self.decision_state = DecisionStates.DONE
        pass

    def when_done(self):
        self.set_radio_message(RadioMessage.CLEANINGDONETAKEMEAWAY)
        self.decision_state = DecisionStates.DONE
        pass 




    def make_decision(self):
        if self.radio_message == RadioMessage.NOMESSAGE:
            pass
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



        

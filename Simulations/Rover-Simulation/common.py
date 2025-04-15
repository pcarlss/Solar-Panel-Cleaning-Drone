from enum import IntEnum
import math
import random
import numpy as np
from dataclasses import dataclass

IMU_ERROR = 0.05  # Example error factor for IMU
TOP_SPEED_MAPPING = 20  # mm/s
TOP_SPEED_CLEANING = 50  # mm/s
ROVER_DIMENSIONS = (170, 200)  # mm (WIDTH, LENGTH)
PANEL_DIMENSIONS = (500,700) #MM (width, length)
CLEANER_WIDTH = 150  # mm
ACCELERATION_STEP = 5  # mm/s^2

LOCKOUT_COUNTDOWN = 20


TIME_STEP = 10/1000 # s


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

@dataclass
class Point:
    coord: tuple[int, int]
    is_clean: bool = False     

class OutOfBoundsError(Exception):
    pass

class SearchForCornerStates(IntEnum):
    MOVEBACKWARDSUNTILEDGE = 1
    ALIGNWITHEDGE = 2
    TURNRIGHT = 3
    ADJUSTBACKANDFORTH = 4
    MOVEBACKWARDSUNTILCORNER = 5
    DONE = 6

class OuterLoopStates(IntEnum):
    FOLLOWEDGE = 1
    TURNLEFT = 2
    REVERSE = 3
    DONE = 4
    
class InnerLoopStates(IntEnum):
    FOLLOWPATH = 1
    TURNLEFT = 2
    DONE = 3

class DecisionStates(IntEnum):
    IDLE = 0
    SEARCHFORCORNER = 1
    BEGINCLEANING = 2
    CLEANOUTERLOOP = 3
    CLEANINNERLOOPS = 4
    DONE = 5

class RadioMessage(IntEnum):
    STARTCLEANINGOK = 1
    STOPCLEANING = 2
    CLEANINGDONETAKEMEAWAY = 3
    NOMESSAGE = 4
    ERROR = 5

class MotorStates():
    MOTOR_EN = 1
    MOTOR_DIS = 0

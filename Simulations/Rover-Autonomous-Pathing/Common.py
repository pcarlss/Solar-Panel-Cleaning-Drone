from enum import Enum
import math
import random
from dataclasses import dataclass

IMU_ERROR = 0.05  # Example error factor for IMU
TOP_SPEED_MAPPING = 20  # mm/s
TOP_SPEED_CLEANING = 50  # mm/s
ROVER_DIMENSIONS = (200, 170)  # mm (length, width)
CLEANER_WIDTH = 150  # mm
ACCELERATION_STEP = 5  # mm/s^2
TIME_STEP = 1

@dataclass
class Point:
    coord: tuple[int, int]
    is_clean: bool = False     

class OutOfBoundsError(Exception):
    pass

class SearchForCornerStates(Enum):
    MOVEBACKWARDSUNTILEDGE = 1
    ALIGNWITHEDGE = 2
    TURNRIGHT = 3
    MOVEFORWARDSLIGHTLY = 4
    MOVEBACKWARDSUNTILCORNER = 5


class DecisionStates(Enum):
    IDLE = 1
    SEARCHFORCORNER = 2
    BEGINCLEANING = 3
    CLEANOUTERLOOP = 4
    CLEANINNERLOOPS = 5
    DONE = 6

class RadioMessage(Enum):
    STARTCLEANINGOK = 1
    STOPCLEANING = 2
    CLEANINGDONETAKEMEAWAY = 3
    NOMESSAGE = 4


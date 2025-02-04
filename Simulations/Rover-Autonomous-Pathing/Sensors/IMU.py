import math

class IMU:
    def __init__(self):
        self.prev_acceleration = [0, 0]  # [ax, ay] in mm/s²
        self.prev_velocity = 0          # Scalar velocity in mm/s
        self.prev_orientation = 0       # Angle in degrees (yaw)
        self.prev_position = [0, 0]     # [x, y] position in mm

    def getInfo(self):
        return 
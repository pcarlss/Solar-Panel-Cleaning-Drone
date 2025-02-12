class IMU:
    def __init__(self):
        self.prev_acceleration = [0, 0]  # [ax, ay] in mm/s²
        self.prev_velocity = 0          # Scalar velocity in mm/s
        self.prev_orientation = 0       # Angle in degrees (yaw)
        self.prev_position = [0, 0]     # [x, y] position in mm

    def get_info(self):
        return {
            "acceleration": self.prev_acceleration,
            "velocity": self.prev_velocity,
            "orientation": self.prev_orientation,
            "position": self.prev_position,
        }

    def reset_position(self):
        self.prev_position = [0, 0]
        self.prev_orientation = 0

    def adjust_alignment(self):
        pass

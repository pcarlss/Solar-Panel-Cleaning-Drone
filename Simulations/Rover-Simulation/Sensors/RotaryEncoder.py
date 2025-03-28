class RotaryEncoder:
    def __init__(self):
        self.position = 0

    def get_position(self):
        """will have to integrate velocity"""
        return self.position

    def get_velocity(self):
        return 0
    def reset(self):
        self.position = 0

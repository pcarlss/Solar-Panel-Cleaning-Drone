class TrackMotor:
    def __init__(self):
        self.pwm = 0
        self.direction = "stopped"

    def power(self, pwm, direction):
        self.pwm = pwm
        self.direction = direction

    def move_forward(self):
        self.power(100, "forward")

    def move_backward(self):
        self.power(100, "backward")

    def stop(self):
        self.power(0, "stopped")

    def rotate(self, angle, direction):
        self.power(100, direction)  


class CleaningMotor:
    def __init__(self):
        self.is_cleaning = False

    def power(self, is_on: bool):
        self.is_cleaning = is_on

    def is_cleaning(self):
        return self.is_cleaning

    def start(self):
        self.power(True)

    def stop(self):
        self.power(False)

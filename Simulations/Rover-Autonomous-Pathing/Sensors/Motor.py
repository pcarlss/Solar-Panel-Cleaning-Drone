class TrackMotor:
    def power(self, pwm,direction):
       return

   
class CleaningMotor:
    def __init__(self):
        self.is_cleaning = False


    def power(self, is_on:bool):
       self.is_cleaning = is_on


    def is_cleaning(self):
        return self.is_cleaning
 
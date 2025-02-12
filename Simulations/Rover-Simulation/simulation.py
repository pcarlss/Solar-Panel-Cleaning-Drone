from time import sleep
from area import SolarPanelArea
from rover import Rover
from common import RadioMessage,TIME_STEP,PANEL_DIMENSIONS,TOP_SPEED_MAPPING ,TOP_SPEED_CLEANING , ROVER_DIMENSIONS ,CLEANER_WIDTH , ACCELERATION_STEP  


class Simulation:
    def __init__(self, width, length, visual=False, visualize_options=None):
        self.solar_panel_area = SolarPanelArea(width, length)
        self.rover = Rover(self.solar_panel_area, TIME_STEP)
        self.visual = visual
        self.visualize_options = visualize_options

        self.sim_runtime_ms = 0
    
    def run(self):
        self.rover.set_radio_message(RadioMessage.STARTCLEANINGOK)
        while(self.rover.radio_message() != RadioMessage.CLEANINGDONETAKEMEAWAY):
            self.solar_panel_area.update_rover_on_panel(self.rover.get_actual_data())
            self.rover.update_sensors()
            self.rover.make_decision()
            self.sim_runtime_ms += TIME_STEP 
            if self.visual:
                self.visualize()
        print(f"Simulation completed in {self.sim_runtime_ms} ms")
        return

    def visualize(self):
        return

# Define the main function
def main():
    # Create and run the simulation
    simulation = Simulation(width = PANEL_DIMENSIONS[0], length = PANEL_DIMENSIONS[1])
    simulation.run()

# Entry point for the script
if __name__ == "__main__":
    main()

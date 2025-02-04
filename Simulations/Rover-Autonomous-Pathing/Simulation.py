from SolarPanelArea import SolarPanelArea
from Rover import Rover
from Common import Common


class Simulation:
    def __init__(self, width, length):
        self.solar_panel_area = SolarPanelArea(width, length)
        self.rover = Rover(self.solar_panel_area)
    def place(self,rover,solar_panel):
        return
    
    def run(self):
        return



# Define the main function
def main():
    # Define the dimensions of the solar panel area (e.g., 1000mm x 800mm)
    width = 1000  # in millimeters
    length = 800  # in millimeters

    # Create and run the simulation
    simulation = Simulation(width, length)
    simulation.run()

# Entry point for the script
if __name__ == "__main__":
    main()

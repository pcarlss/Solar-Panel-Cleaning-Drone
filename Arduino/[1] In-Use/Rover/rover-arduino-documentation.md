# Rover Arduino Documentation

## Pin Connection Guide

### Motor Pins
- Left Motor:
  - PWM: D9
  - Direction 1: D8
  - Direction 2: D7
- Right Motor:
  - PWM: D10
  - Direction 1: D11
  - Direction 2: D12
- Cleaning Motor:
  - Enable: D13

### Limit Switch Pins (CD4051B Multiplexer)
- Selector 0: D2
- Selector 1: D3
- Selector 2: D4
- Output: D5

### IMU (ICM-20948) I2C Pins
- SDA: A4
- SCL: A5

### Rotary Encoders
- Left Encoder:
  - Channel A: D2
  - Channel B: D3
- Right Encoder:
  - Channel A: D4
  - Channel B: D5

### Radio Control (nRF24L01)
- CE: D6
- CSN: D7

## Function Documentation

### Core Functions
- `setup()`: Initializes all hardware components including motors, encoders, IMU, and radio communication.
- `loop()`: Main program loop that handles state machine, sensor updates, and motor control.
- `updateData()`: Updates all sensor readings and system state.
- `updatePosition()`: Calculates current position based on encoder and IMU data.

### Motor Control
- `stopMotors()`: Stops all motors safely.
- `updateMotors()`: Updates motor speeds based on PID control.
- `setTrajectory(float linearVelocity, float turnRate)`: Sets desired linear velocity and turn rate.

### Sensor Management
- `getIMUData()`: Reads and processes IMU data.
- `performIMUCalibration()`: Performs IMU calibration sequence.
- `updateLimitSwitches()`: Updates limit switch states using multiplexer.
- `getLimitSwitchPositions()`: Returns current limit switch states.

### Encoder Functions
- `encoderISR_L()`: Interrupt service routine for left encoder.
- `encoderISR_R()`: Interrupt service routine for right encoder.
- `getLinearVelocity()`: Calculates current linear velocity from encoder data.

### Navigation Functions
- `makeDecision()`: Main decision-making function implementing the state machine.
- `searchForCorner()`: Implements corner search behavior.
- `cleanOuterLoop()`: Handles outer loop cleaning behavior.
- `cleanInnerLoops()`: Handles inner loop cleaning behavior.
- `followEdge()`: Implements edge following behavior.
- `followInnerPath()`: Implements inner path following behavior.

### Helper Functions
- `calculatePID()`: Implements PID control algorithm.
- `computeTravel()`: Calculates travel distance and direction.
- `mapInnerPath()`: Calculates inner cleaning path parameters.
- `calculateDistance()`: Calculates distance between two points.

## Setup Instructions

1. Hardware Setup:
   - Connect all motors to their respective pins
   - Connect limit switches through the CD4051B multiplexer
   - Connect IMU via I2C
   - Connect rotary encoders
   - Connect nRF24L01 radio module

2. Software Setup:
   - Install required libraries:
     - Wire.h (built-in)
     - PID_v1.h
     - ICM_20948.h
     - SPI.h (built-in)
     - nRF24L01.h
     - RF24.h

3. Calibration:
   - The system will automatically perform IMU calibration on startup
   - Keep the rover stationary during calibration
   - Calibration takes approximately 5 seconds

4. Operation:
   - Power on the rover
   - Wait for initialization to complete
   - Use the controller to start/stop the cleaning operation
   - The system will automatically handle the cleaning sequence

## Safety Features
- Emergency stop on radio signal loss
- Limit switch protection
- Motor current monitoring
- Automatic stop on error conditions

## Troubleshooting
1. If motors don't respond:
   - Check motor connections
   - Verify PWM pins are correctly connected
   - Check motor driver power supply

2. If IMU fails to initialize:
   - Check I2C connections
   - Verify IMU power supply
   - Ensure correct I2C address

3. If limit switches don't work:
   - Check multiplexer connections
   - Verify switch wiring
   - Test individual switches

4. If radio communication fails:
   - Check CE and CSN connections
   - Verify radio module power supply
   - Ensure correct address matching with controller 
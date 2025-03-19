#include <Wire.h>
#include <PID_v1.h>

// Define motor pins
#define MOTOR_L_PWM 9    // D9 - Left motor PWM
#define MOTOR_L_IN1 8    // D8 - Left motor direction 1
#define MOTOR_L_IN2 7    // D7 - Left motor direction 2

#define MOTOR_R_PWM 10   // D10 - Right motor PWM
#define MOTOR_R_IN1 11   // D11 - Right motor direction 1
#define MOTOR_R_IN2 12   // D12 - Right motor direction 2

// Define cleaning motor pin
#define CLEANING_MOTOR_ENABLE 13  // D13 - Cleaning motor enable

// Define limit switch pins (CD4051B multiplexer)
#define LIMIT_SWITCH_COUNT 8
#define LIMIT_SWITCH_PIN_SELECTOR_0 2  // D2 - Multiplexer address bit 0
#define LIMIT_SWITCH_PIN_SELECTOR_1 3  // D3 - Multiplexer address bit 1
#define LIMIT_SWITCH_PIN_SELECTOR_2 4  // D4 - Multiplexer address bit 2
#define LIMIT_SWITCH_PIN_OUTPUT 5      // D5 - Multiplexer output

// IMU (ICM-20948) I2C pins
#define IMU_SDA A4  // A4 - IMU I2C data
#define IMU_SCL A5  // A5 - IMU I2C clock

// Rotary Encoders
#define ENCODER_L_A 2    // D2 - Left encoder channel A
#define ENCODER_L_B 3    // D3 - Left encoder channel B
#define ENCODER_R_A 4    // D4 - Right encoder channel A
#define ENCODER_R_B 5    // D5 - Right encoder channel B

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

volatile long previousLeftEncoderCount = 0;
volatile long previousRightEncoderCount = 0;
volatile unsigned long previousEncoderTime = 0;  // For tracking time between encoders

// Define variables to store encoder velocity (calculated in ISR)
volatile float leftEncoderVelocity = 0.0;
volatile float rightEncoderVelocity = 0.0;

// PID control variables with feed-forward term
double pidInputL, pidOutputL, pidSetpointL;
double pidInputR, pidOutputR, pidSetpointR;
double pidKp = 0.05;  // Proportional gain
double pidKi = 0.5;   // Integral gain
double pidKd = 0.05;  // Derivative gain
double pidKff = 3.0;  // Feed-forward gain

// Initialize PID controllers for left and right motors with feed-forward
PID pidLeft(&pidInputL, &pidOutputL, &pidSetpointL, pidKp, pidKi, pidKd, DIRECT);
PID pidRight(&pidInputR, &pidOutputR, &pidSetpointR, pidKp, pidKi, pidKd, DIRECT);

// Physical constants from simulation
const float AXLE_LENGTH = 0.170;  // 170mm
const float WHEEL_RADIUS = 0.025; // 25mm
const float TOP_SPEED = 0.04;     // 4 cm/s
const float TOP_TURN_RATE = 1.0;  // rad/s

// Position and orientation tracking
struct PositionalInformation {
    float position[2];      // [x, y]
    float orientation[2];   // [x, y] unit vector
    float azimuth;         // Current heading angle
    float linear_velocity; // Current linear velocity
    float turn_rate;      // Current turn rate
    float linear_accel;   // Current linear acceleration
    float turn_accel;     // Current turn acceleration
    float l_speed;        // Left wheel speed
    float r_speed;        // Right wheel speed
};

PositionalInformation currentPosition;
PositionalInformation estimatedPosition;

// Enums for state management
enum DecisionStates { IDLE, SEARCHFORCORNER, BEGINCLEANING, CLEANOUTERLOOP, CLEANINNERLOOPS, DONE };
enum SearchForCornerStates { MOVEBACKWARDSUNTILEDGE, ALIGNWITHEDGE, TURNRIGHT, ADJUSTBACKANDFORTH, MOVEBACKWARDSUNTILCORNER, SEARCHDONE };
enum OuterLoopStates { FOLLOWEDGE, TURNLEFT, OUTERDONE };
enum InnerLoopStates { FOLLOWINNERPATH, INNERDONE };
enum RadioMessage { NOMESSAGE, STARTCLEANINGOK, CLEANINGDONETAKEMEAWAY };

// State variables
DecisionStates decisionState = IDLE;
SearchForCornerStates searchForCornerState = MOVEBACKWARDSUNTILEDGE;
OuterLoopStates outerLoopState = FOLLOWEDGE;
InnerLoopStates innerLoopState = FOLLOWINNERPATH;
RadioMessage radioMessage = NOMESSAGE;

// Motor speed control variables
int l_speed = 0;
int r_speed = 0;


unsigned long lastTime = 0;  // For tracking time delta for integration
struct LimitSwitchPair {
    bool innerLimitSwitch;
    bool outerLimitSwitch;
};

// Struct to store sensor data
struct SensorData {
    LimitSwitchPair limitSwitches[LIMIT_SWITCH_COUNT];
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    long leftEncoder, rightEncoder;
    long leftEncoderVelocity, rightEncoderVelocity;
    float lastPosition = 0.0, imuVelocity = 0.0, imuPosition = 0.0;    
};

SensorData sensorData;

// IMU data structure
struct IMUData {
    float accel[3];    // [x, y, z] acceleration in m/s²
    float gyro[3];     // [x, y, z] angular velocity in rad/s
    float orientation[3]; // [roll, pitch, yaw] in radians
};

IMUData imuData;

// Limit switch configuration
struct LimitSwitchConfig {
    float relativePos[2];  // [x, y] relative to rover center
    bool isPressed;
};

// Define limit switch configurations (relative positions in meters)
LimitSwitchConfig limitSwitchConfigs[LIMIT_SWITCH_COUNT] = {
    {{0.1, 0.105}, false},  // LFO
    {{0.1, 0.1}, false},    // LFI
    {{0.1, -0.105}, false}, // RFO
    {{0.1, -0.1}, false},   // RFI
    {{-0.1, 0.105}, false}, // LBO
    {{-0.1, 0.1}, false},   // LBI
    {{-0.1, -0.105}, false},// RBO
    {{-0.1, -0.1}, false}   // RBI
};

// Function prototypes
void updateData();
float getVelocityFromEncoder(long currentEncoder, long previousEncoder, unsigned long currentTime);
void getVelocityIMU();
void getPositionIMU();
void updateLimitSwitches();

void encoderISR_L();
void encoderISR_R();
void updatePosition();
void setTrajectory(float desiredSpeed, float desiredTurnRate);

void moveBackwardUntilEdge();
void alignWithEdge();
void turnRight(int deg);
void turnLeft(int deg);
void adjustBackAndForth();
void moveBackwardUntilCorner();
void followEdge();
void followInnerPath();

void stopMotors();
void updateMotors();

void makeDecision();
void searchForCorner();

void initializeCleaning();
void cleanOuterLoop();
void cleanInnerLoops();
void whenDone();

void getIMUData();
void getLimitSwitchPositions();

void setup() {
    Serial.begin(115200);
    Wire.begin();

    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

    pinMode(LIMIT_SWITCH_PIN_SELECTOR_0, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_1, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_2, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_OUTPUT, INPUT_PULLUP);

    pinMode(ENCODER_L_A, INPUT_PULLUP);
    pinMode(ENCODER_L_B, INPUT_PULLUP);
    pinMode(ENCODER_R_A, INPUT_PULLUP);
    pinMode(ENCODER_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderISR_R, CHANGE);

    pidLeft.SetMode(AUTOMATIC);
    pidRight.SetMode(AUTOMATIC);

    Serial.println("Rover Initialized.");
}

void loop() {
    updateData();
    updatePosition();
    makeDecision();
}

// Function to update data (integrates velocity and position from IMU)
void updateData() {
    // Update IMU data
    getIMUData();
    
    // Update limit switch states
    updateLimitSwitches();
    
    // Update limit switch positions
    getLimitSwitchPositions();
    
    // Update sensor data structure
    sensorData.accelX = imuData.accel[0];
    sensorData.accelY = imuData.accel[1];
    sensorData.accelZ = imuData.accel[2];
    sensorData.gyroX = imuData.gyro[0];
    sensorData.gyroY = imuData.gyro[1];
    sensorData.gyroZ = imuData.gyro[2];
    
    // Update encoder data
    sensorData.leftEncoder = leftEncoderCount;
    sensorData.rightEncoder = rightEncoderCount;
    sensorData.leftEncoderVelocity = leftEncoderVelocity;
    sensorData.rightEncoderVelocity = rightEncoderVelocity;
    
    // Update IMU-based position and velocity
    sensorData.imuVelocity = sqrt(imuData.accel[0] * imuData.accel[0] + 
                                 imuData.accel[1] * imuData.accel[1]);
    sensorData.imuPosition = currentPosition.position[0]; // Using x position as example

    // Debug print sensor data
    Serial.print("Left Velocity: ");
    Serial.print(leftEncoderVelocity);
    Serial.print(", Right Velocity: ");
    Serial.println(rightEncoderVelocity);

    // Get IMU data (acceleration and velocity)
    getVelocityIMU();
    getPositionIMU();
    
    // Debug print sensor data
    Serial.print("Encoders: L=");
    Serial.print(sensorData.leftEncoder);
    Serial.print(", R=");
    Serial.print(sensorData.rightEncoder);
    Serial.print(" | Left Velocity: ");
    Serial.print(sensorData.leftEncoderVelocity);
    Serial.print(", Right Velocity: ");
    Serial.print(sensorData.rightEncoderVelocity);
    Serial.print(" | IMU Velocity: ");
    Serial.print(sensorData.imuVelocity);
    Serial.print(", IMU Position: ");
    Serial.println(sensorData.imuPosition);
}

// Method to calculate velocity based on encoder counts (using time delta)
float getVelocityFromEncoder(long currentEncoder, long previousEncoder, unsigned long currentTime) {
    float deltaEncoder = currentEncoder - previousEncoder;
    float deltaTime = (currentTime - lastTime) / 1000.0;  // Time in seconds
    lastTime = currentTime;  // Update last time for the next calculation
    float velocity = deltaEncoder / deltaTime;  // Encoder ticks per second
    return velocity;
}

// Method to compute velocity from IMU (integrating acceleration over time)
void getVelocityIMU() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;  // Time in seconds
    float accelX = sensorData.accelX;  // Assume sensorData.accelX is read from IMU
    sensorData.imuVelocity += accelX * deltaTime;  // Integrate acceleration to get velocity
    lastTime = currentTime;  // Update last time
}

// Method to compute position from IMU (integrating velocity over time)
void getPositionIMU() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0;  // Time in seconds
    sensorData.imuPosition += sensorData.imuVelocity * deltaTime;  // Integrate velocity to get position
    lastTime = currentTime;  // Update last time
}

// Function to get IMU data
void getIMUData() {
    // Read raw IMU data (placeholder - replace with actual IMU reading code)
    Wire.beginTransmission(0x68); // IMU I2C address
    Wire.write(0x3B); // Start register for accelerometer data
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6); // Request 6 bytes for accelerometer data
    
    // Read accelerometer data
    imuData.accel[0] = (Wire.read() << 8 | Wire.read()) / 16384.0; // Convert to m/s²
    imuData.accel[1] = (Wire.read() << 8 | Wire.read()) / 16384.0;
    imuData.accel[2] = (Wire.read() << 8 | Wire.read()) / 16384.0;
    
    // Read gyroscope data
    Wire.beginTransmission(0x68);
    Wire.write(0x43); // Start register for gyroscope data
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    
    imuData.gyro[0] = (Wire.read() << 8 | Wire.read()) / 131.0; // Convert to rad/s
    imuData.gyro[1] = (Wire.read() << 8 | Wire.read()) / 131.0;
    imuData.gyro[2] = (Wire.read() << 8 | Wire.read()) / 131.0;
    
    // Update orientation using gyroscope data
    float deltaTime = (millis() - lastTime) / 1000.0;
    imuData.orientation[0] += imuData.gyro[0] * deltaTime;
    imuData.orientation[1] += imuData.gyro[1] * deltaTime;
    imuData.orientation[2] += imuData.gyro[2] * deltaTime;
}

// Function to update limit switch states
void updateLimitSwitches() {
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        // Set multiplexer address
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_0, (i & 0x01) ? HIGH : LOW);
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_1, (i & 0x02) ? HIGH : LOW);
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_2, (i & 0x04) ? HIGH : LOW);
        
        delay(5); // Wait for multiplexer to settle
        
        // Read limit switch state
        limitSwitchConfigs[i].isPressed = (digitalRead(LIMIT_SWITCH_PIN_OUTPUT) == LOW);
    }
}

// Function to get limit switch positions relative to rover center
void getLimitSwitchPositions() {
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        float x = currentPosition.position[0] + 
                 limitSwitchConfigs[i].relativePos[0] * cos(currentPosition.azimuth) -
                 limitSwitchConfigs[i].relativePos[1] * sin(currentPosition.azimuth);
                 
        float y = currentPosition.position[1] + 
                 limitSwitchConfigs[i].relativePos[0] * sin(currentPosition.azimuth) +
                 limitSwitchConfigs[i].relativePos[1] * cos(currentPosition.azimuth);
                 
        // Store positions in sensorData if needed
        // sensorData.limitSwitchPositions[i][0] = x;
        // sensorData.limitSwitchPositions[i][1] = y;
    }
}

//add aidans encoder velocity processing
void encoderISR_L() {
    unsigned long currentTime = millis();
    
    // Calculate delta time in seconds
    float deltaTime = (currentTime - previousEncoderTime) / 1000.0; // Time in seconds
    
    // Calculate the change in encoder count for the left encoder
    long deltaEncoder = leftEncoderCount - previousLeftEncoderCount;

    // Compute velocity (change in encoder count divided by time delta)
    leftEncoderVelocity = deltaEncoder / deltaTime;

    // Update previous encoder count and time for next calculation
    previousLeftEncoderCount = leftEncoderCount;
    previousEncoderTime = currentTime;
}

void encoderISR_R() {
    unsigned long currentTime = millis();

    // Calculate delta time in seconds
    float deltaTime = (currentTime - previousEncoderTime) / 1000.0; // Time in seconds
    
    // Calculate the change in encoder count for the right encoder
    long deltaEncoder = rightEncoderCount - previousRightEncoderCount;

    // Compute velocity (change in encoder count divided by time delta)
    rightEncoderVelocity = deltaEncoder / deltaTime;

    // Update previous encoder count and time for next calculation
    previousRightEncoderCount = rightEncoderCount;
    previousEncoderTime = currentTime;
}

// **Update position based on encoder readings**
void updatePosition() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    
    // Update linear velocity from encoders
    float leftVelocity = leftEncoderVelocity * WHEEL_RADIUS;
    float rightVelocity = rightEncoderVelocity * WHEEL_RADIUS;
    
    // Calculate linear velocity and turn rate
    currentPosition.linear_velocity = (leftVelocity + rightVelocity) / 2;
    currentPosition.turn_rate = (rightVelocity - leftVelocity) / AXLE_LENGTH;
    
    // Update position using velocity
    currentPosition.position[0] += currentPosition.linear_velocity * cos(currentPosition.azimuth) * deltaTime;
    currentPosition.position[1] += currentPosition.linear_velocity * sin(currentPosition.azimuth) * deltaTime;
    
    // Update orientation using turn rate
    currentPosition.azimuth += currentPosition.turn_rate * deltaTime;
    currentPosition.orientation[0] = cos(currentPosition.azimuth);
    currentPosition.orientation[1] = sin(currentPosition.azimuth);
    
    // Update last time
    lastTime = currentTime;
}

// **Set motion trajectory**
void setTrajectory(float desiredSpeed, float desiredTurnRate) {
    // Convert desired speed and turn rate to wheel speeds
    float l_speed = (desiredSpeed - (AXLE_LENGTH * desiredTurnRate) / 2) / WHEEL_RADIUS;
    float r_speed = (desiredSpeed + (AXLE_LENGTH * desiredTurnRate) / 2) / WHEEL_RADIUS;
    
    // Apply speed limits
    l_speed = constrain(l_speed, -TOP_SPEED/WHEEL_RADIUS, TOP_SPEED/WHEEL_RADIUS);
    r_speed = constrain(r_speed, -TOP_SPEED/WHEEL_RADIUS, TOP_SPEED/WHEEL_RADIUS);
    
    // Update setpoints for PID controllers
    pidSetpointL = l_speed;
    pidSetpointR = r_speed;
    
    // Store current speeds for position tracking
    currentPosition.l_speed = l_speed;
    currentPosition.r_speed = r_speed;
}

void moveBackwardUntilEdge() {
    setTrajectory(-0.05, 0); // Move backward at 5 cm/s
    
    // Check if either back limit switch is pressed
    if (limitSwitchConfigs[4].isPressed || limitSwitchConfigs[5].isPressed || // LBO or LBI
        limitSwitchConfigs[6].isPressed || limitSwitchConfigs[7].isPressed) { // RBO or RBI
        stopMotors();
        searchForCornerState = ALIGNWITHEDGE;
    }
}

void alignWithEdge() {
    // Check if both back limit switches are pressed
    if ((limitSwitchConfigs[4].isPressed && limitSwitchConfigs[5].isPressed) || // Both left back switches
        (limitSwitchConfigs[6].isPressed && limitSwitchConfigs[7].isPressed)) { // Both right back switches
        searchForCornerState = TURNRIGHT;
        return;
    }
    
    // Adjust position based on which switches are pressed
    if (limitSwitchConfigs[4].isPressed || limitSwitchConfigs[5].isPressed) {
        setTrajectory(0, 0.1); // Turn right
    } else if (limitSwitchConfigs[6].isPressed || limitSwitchConfigs[7].isPressed) {
        setTrajectory(0, -0.1); // Turn left
    }
}

void turnRight(int deg) {
    float targetAzimuth = currentPosition.azimuth - (deg * PI / 180.0);
    float error = targetAzimuth - currentPosition.azimuth;
    
    // Normalize error to [-PI, PI]
    while (error > PI) error -= 2 * PI;
    while (error < -PI) error += 2 * PI;
    
    // Use proportional control for turn rate
    float turnRate = constrain(error * 0.5, -TOP_TURN_RATE, TOP_TURN_RATE);
    setTrajectory(0, turnRate);
    
    // Check if we've reached the target angle
    if (abs(error) < 0.1) { // Within 5.7 degrees
        stopMotors();
        searchForCornerState = ADJUSTBACKANDFORTH;
    }
}

void turnLeft(int deg) {
    float targetAzimuth = currentPosition.azimuth + (deg * PI / 180.0);
    float error = targetAzimuth - currentPosition.azimuth;
    
    // Normalize error to [-PI, PI]
    while (error > PI) error -= 2 * PI;
    while (error < -PI) error += 2 * PI;
    
    // Use proportional control for turn rate
    float turnRate = constrain(error * 0.5, -TOP_TURN_RATE, TOP_TURN_RATE);
    setTrajectory(0, turnRate);
    
    // Check if we've reached the target angle
    if (abs(error) < 0.1) { // Within 5.7 degrees
        stopMotors();
        searchForCornerState = MOVEBACKWARDSUNTILCORNER;
    }
}

void adjustBackAndForth() {
    // Check if both back limit switches are pressed
    if ((limitSwitchConfigs[4].isPressed && limitSwitchConfigs[5].isPressed) || // Both left back switches
        (limitSwitchConfigs[6].isPressed && limitSwitchConfigs[7].isPressed)) { // Both right back switches
        setTrajectory(0.01, 0); // Move forward slightly
    } else if (!limitSwitchConfigs[4].isPressed && !limitSwitchConfigs[5].isPressed && // No left back switches
               !limitSwitchConfigs[6].isPressed && !limitSwitchConfigs[7].isPressed) { // No right back switches
        setTrajectory(-0.01, 0); // Move backward slightly
    } else {
        stopMotors();
        searchForCornerState = MOVEBACKWARDSUNTILCORNER;
    }
}

void moveBackwardUntilCorner() {
    setTrajectory(-0.05, 0); // Move backward at 5 cm/s
    
    // Check if all back limit switches are pressed
    if (limitSwitchConfigs[4].isPressed && limitSwitchConfigs[5].isPressed && // Both left back switches
        limitSwitchConfigs[6].isPressed && limitSwitchConfigs[7].isPressed) { // Both right back switches
        stopMotors();
        searchForCornerState = SEARCHDONE;
    }
}

void followEdge() {
    float speed = 0.05; // 5 cm/s
    float turnRate = 0;
    
    // Check front limit switches for edge following
    if (limitSwitchConfigs[0].isPressed && limitSwitchConfigs[1].isPressed) {
        turnRate = 0; // Both front switches pressed, maintain straight line
    } else if (limitSwitchConfigs[0].isPressed) {
        turnRate = -0.05; // Left front switch pressed, turn right
    } else if (limitSwitchConfigs[1].isPressed) {
        turnRate = 0.05; // Right front switch pressed, turn left
    }
    
    setTrajectory(speed, turnRate);
}

void followInnerPath() {
    float speed = 0.05; // 5 cm/s
    float turnRate = 0.1; // Constant turn rate for inner path
    
    // Check limit switches to adjust path if needed
    if (limitSwitchConfigs[0].isPressed || limitSwitchConfigs[1].isPressed) {
        turnRate = -0.1; // Reverse turn direction if hitting edge
    }
    
    setTrajectory(speed, turnRate);
}

void stopMotors() {
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
}

void updateMotors() {
    // Calculate feed-forward terms
    float ffL = pidKff * pidSetpointL;
    float ffR = pidKff * pidSetpointR;
    
    // Update PID inputs
    pidInputL = currentPosition.l_speed;
    pidInputR = currentPosition.r_speed;
    
    // Compute PID outputs
    pidLeft.Compute();
    pidRight.Compute();
    
    // Combine PID and feed-forward outputs
    float finalOutputL = pidOutputL + ffL;
    float finalOutputR = pidOutputR + ffR;
    
    // Apply outputs to motors
    analogWrite(MOTOR_L_PWM, constrain(finalOutputL, 0, 255));
    analogWrite(MOTOR_R_PWM, constrain(finalOutputR, 0, 255));
}

void makeDecision() {
    //check limit switches to see if action is required
    
    switch (decisionState) {
        case SEARCHFORCORNER:
            searchForCorner();
            break;
        case BEGINCLEANING:
            initializeCleaning();
            break;
        case CLEANOUTERLOOP:
            cleanOuterLoop();
            break;
        case CLEANINNERLOOPS:
            cleanInnerLoops();
            break;
        case DONE:
            whenDone();
            break;
        default:
            break;
    }
    updateMotors();
}

void searchForCorner() {
    switch (searchForCornerState) {
        case MOVEBACKWARDSUNTILEDGE:
            moveBackwardUntilEdge();
            break;
        case ALIGNWITHEDGE:
            alignWithEdge();
            break;
        case TURNRIGHT:
            turnRight(90);
            break;
        case ADJUSTBACKANDFORTH:
            adjustBackAndForth();
            break;
        case MOVEBACKWARDSUNTILCORNER:
            moveBackwardUntilCorner();
            break;
        case SEARCHDONE:
            decisionState = BEGINCLEANING;
            break;
    }
}

void initializeCleaning() {
    Serial.println("Initializing cleaning...");
    digitalWrite(CLEANING_MOTOR_ENABLE, HIGH); // Turn on cleaning motor
    decisionState = CLEANOUTERLOOP;
}

void cleanOuterLoop() {
    Serial.println("Cleaning outer loop...");

    switch (outerLoopState) {
        case FOLLOWEDGE:
            followEdge();
            break;
        case TURNLEFT:
            turnLeft(90);
            break;
        case OUTERDONE:
            stopMotors();
            decisionState = CLEANINNERLOOPS;
            break;
    }
}

void cleanInnerLoops() {
    Serial.println("Cleaning inner loops...");

    switch (innerLoopState) {
        case FOLLOWINNERPATH:
            followInnerPath();
            break;
        case INNERDONE:
            stopMotors();
            decisionState = DONE;
            break;
    }
}

void whenDone() {
    Serial.println("Cleaning done. Sending radio message.");
    stopMotors();
    digitalWrite(CLEANING_MOTOR_ENABLE, LOW); // Turn off cleaning motor
    decisionState = DONE;
}

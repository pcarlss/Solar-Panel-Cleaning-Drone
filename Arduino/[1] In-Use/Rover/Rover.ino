#include <Wire.h>
#include <PID_v1.h>
#include <ICM_20948.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ==================== Pin Definitions ====================
// Motor pins
#define MOTOR_L_PWM 9    // D9 - Left motor PWM
#define MOTOR_L_IN1 8    // D8 - Left motor direction 1
#define MOTOR_L_IN2 7    // D7 - Left motor direction 2
#define MOTOR_R_PWM 10   // D10 - Right motor PWM
#define MOTOR_R_IN1 11   // D11 - Right motor direction 1
#define MOTOR_R_IN2 12   // D12 - Right motor direction 2
#define CLEANING_MOTOR_ENABLE 13  // D13 - Cleaning motor enable

// Limit switch pins
#define LIMIT_SWITCH_COUNT 8
#define LIMIT_SWITCH_PIN_SELECTOR_0 4  // D4 - Multiplexer address bit 0
#define LIMIT_SWITCH_PIN_SELECTOR_1 5  // D5 - Multiplexer address bit 1
#define LIMIT_SWITCH_PIN_SELECTOR_2 A2 // A2 - Multiplexer address bit 2
#define LIMIT_SWITCH_PIN_OUTPUT A3     // A3 - Multiplexer output

// IMU pins
#define IMU_SDA A4  // A4 - IMU I2C data
#define IMU_SCL A5  // A5 - IMU I2C clock

// Encoder pins
#define ENCODER_L_A 2    // D2 - Left encoder channel A (interrupt)
#define ENCODER_L_B 3    // D3 - Left encoder channel B (interrupt)
#define ENCODER_R_A A0   // A0 - Right encoder channel A
#define ENCODER_R_B A1   // A1 - Right encoder channel B

// Radio pins
#define CE_PIN 6    // D6 - Radio CE pin
#define CSN_PIN 7   // D7 - Radio CSN pin

// ==================== Constants ====================
// Physical constants
const float AXLE_LENGTH = 0.170;  // 170mm
const float WHEEL_RADIUS = 0.025; // 25mm
const float TOP_SPEED = 0.04;     // 4 cm/s
const float TOP_TURN_RATE = 1.0;  // rad/s
const float MIN_ANGLE = 0.1;      // Minimum angle change for velocity calculation
const float ZERO_TIME = 0.5;      // Time in seconds before zeroing velocity

// IMU Configuration
#define SAMPLE_RATE 100  // Hz
#define CALIBRATION_SAMPLES 250
#define VALIDATION_SAMPLES 100
#define STABILITY_THRESHOLD 0.1  // degrees/s for gyro, mg for accel

// Add lockout constant after other constants
#define LOCKOUT_COUNTDOWN 20  // Matches simulation

// Add debounce constant after other constants
#define ENCODER_DEBOUNCE_TIME 10  // Minimum time between readings in ms

// ==================== Type Definitions ====================
// Enums for state management
enum DecisionStates { IDLE, SEARCHFORCORNER, BEGINCLEANING, CLEANOUTERLOOP, CLEANINNERLOOPS, DONE };
enum SearchForCornerStates { MOVEBACKWARDSUNTILEDGE, ALIGNWITHEDGE, TURNRIGHT, ADJUSTBACKANDFORTH, MOVEBACKWARDSUNTILCORNER, SEARCHDONE };
enum OuterLoopStates { FOLLOWEDGE, TURNLEFT, OUTERDONE };
enum InnerLoopStates { FOLLOWINNERPATH, TURNLEFTINNER, INNERDONE };
enum RadioMessage { NOMESSAGE, STARTCLEANINGOK, CLEANINGDONETAKEMEAWAY, ERROR };

// Structs
struct Point {
    float x;
    float y;
};

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

struct LimitSwitchPair {
    bool innerLimitSwitch;
    bool outerLimitSwitch;
};

struct SensorData {
    LimitSwitchPair limitSwitches[LIMIT_SWITCH_COUNT];
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    long leftEncoder, rightEncoder;
    long leftEncoderVelocity, rightEncoderVelocity;
    float lastPosition = 0.0, imuVelocity = 0.0, imuPosition = 0.0;    
};

struct IMUData {
    float accel[3];    // [x, y, z] acceleration in m/s²
    float gyro[3];     // [x, y, z] angular velocity in rad/s
    float orientation[3]; // [roll, pitch, yaw] in radians
    float accelBias[3];
    float gyroBias[3];
    bool isCalibrated;
};

struct LimitSwitchConfig {
    float relativePos[2];  // [x, y] relative to rover center
    bool isPressed;
};

struct ControllerData {
  int roll;
  int pitch;
  int throttle;
  int yaw;
  int angle;
  int safety;
  bool AUX;
};

// ==================== Global Variables ====================
// State variables
DecisionStates decisionState = IDLE;
SearchForCornerStates searchForCornerState = MOVEBACKWARDSUNTILEDGE;
OuterLoopStates outerLoopState = FOLLOWEDGE;
InnerLoopStates innerLoopState = FOLLOWINNERPATH;
RadioMessage radioMessage = NOMESSAGE;

// Motor control variables
int l_speed = 0;
int r_speed = 0;

// PID control variables
double pidInputL, pidOutputL, pidSetpointL;
double pidInputR, pidOutputR, pidSetpointR;
double pidKp = 0.05;  // Proportional gain
double pidKi = 0.5;   // Integral gain
double pidKd = 0.05;  // Derivative gain
double pidKff = 3.0;  // Feed-forward gain

// Add after other PID variables
double posKp = 0.25;  // Position proportional gain
double posKi = 0.05;  // Position integral gain
double posKd = 0.0;   // Position derivative gain
double oriKp = 0.8;   // Orientation proportional gain
double oriKi = 0.0;   // Orientation integral gain
double oriKd = 0.0;   // Orientation derivative gain
double posPidInput, posPidOutput, posPidSetpoint;
double oriPidInput, oriPidOutput, oriPidSetpoint;
PID pidPosition(&posPidInput, &posPidOutput, &posPidSetpoint, posKp, posKi, posKd, DIRECT);
PID pidOrientation(&oriPidInput, &oriPidOutput, &oriPidSetpoint, oriKp, oriKi, oriKd, DIRECT);

// Encoder variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long previousLeftEncoderCount = 0;
volatile long previousRightEncoderCount = 0;
volatile float leftEncoderVelocity = 0.0;
volatile float rightEncoderVelocity = 0.0;
volatile float timeBetweenL = 0;
volatile float timeBetweenR = 0;
volatile long prevStepL = 0;
volatile long prevStepR = 0;
volatile float discretePosL = 0;
volatile float discretePosR = 0;
volatile unsigned long sinceLastChangeL = 0;
volatile unsigned long sinceLastChangeR = 0;

// Timing variables
unsigned long lastTime = 0;
unsigned long lastUpdateTime = 0;
unsigned long lastEncoderTime = 0;
unsigned long lastIMUTime = 0;
unsigned long lastSignalTime = 0;

// Panel mapping variables
Point cornerLocations[5];
int cornerCount = 0;
float panelHeight = 0;
float panelWidth = 0;
float desiredDistance = 0;
float desiredAzimuth = 0;
String cleaningAxis = "height";
int panelHeightNodes = 0;
int panelWidthNodes = 0;

// Radio variables
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";
bool radioInitialized = false;
bool signalLost = false;
ControllerData receivedData;

// Sensor objects and data
ICM_20948_I2C ICM;
IMUData imuData;
SensorData sensorData;
PositionalInformation currentPosition;
PositionalInformation estimatedPosition;
PID pidLeft(&pidInputL, &pidOutputL, &pidSetpointL, pidKp, pidKi, pidKd, DIRECT);
PID pidRight(&pidInputR, &pidOutputR, &pidSetpointR, pidKp, pidKi, pidKd, DIRECT);

// Limit switch configurations
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

// Add lockout variable to global variables section
int lockoutCountdown = 0;

// ==================== Function Prototypes ====================
// Core functions
void updateData();
void updatePosition();
void makeDecision();

// Motor control functions
void stopMotors();
void updateMotors();
void setTrajectory(float linearVelocity, float turnRate);

// Sensor functions
void getIMUData();
void performIMUCalibration();
void updateLimitSwitches();
void getLimitSwitchPositions();

// Navigation functions
void searchForCorner();
void moveBackwardUntilEdge();
void alignWithEdge();
void turnRight(int deg);
void turnLeft(int deg);
void adjustBackAndForth();
void moveBackwardUntilCorner();
void followEdge();
void followInnerPath();
void initializeCleaning();
void cleanOuterLoop();
void cleanInnerLoops();
void whenDone();

// Helper functions
float getLinearVelocity();
void computeTravel(float linear_velocity, float turn_rate, float deltaTime);
bool checkLimitSwitches();
float getCurrentAzimuth();
void updatePositionalTrajectory();
void updateTurningTrajectory();
float calculatePID(float setpoint, float input);
void mapInnerPath();
float calculateDistance(Point p1, Point p2);

// Interrupt handlers
void encoderISR_L();
void encoderISR_R();

// ==================== Setup and Main Loop ====================
void setup() {
  // Initialize serial communication
    Serial.begin(115200);
  while (!Serial) {
     // Wait for serial port to connect
  }

  // Initialize IMU
    Wire.begin();
  if (!ICM.begin()) {
    // Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Initialize IMU calibration
  // Serial.println("IMU Calibration Starting...");
  // Serial.println("Please keep the IMU stationary during calibration.");
  performIMUCalibration();

  // Initialize radio
  if (!radio.begin()) {
    // Radio hardware not responding
    while (1); // Stop if radio fails
  }
  
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(sizeof(ControllerData));
  radio.setChannel(100);
  radio.startListening();
  
  radioInitialized = true;
  
  // Serial.println("Rover Initialized.");

  // Initialize other pins
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
  pidPosition.SetMode(AUTOMATIC);
  pidOrientation.SetMode(AUTOMATIC);
  pidPosition.SetOutputLimits(-TOP_SPEED, TOP_SPEED);
  pidOrientation.SetOutputLimits(-TOP_TURN_RATE, TOP_TURN_RATE);
}

void loop() {
    // Check for radio signals
    if (radio.available()) {
        lastSignalTime = millis();
        signalLost = false;
        radio.read(&receivedData, sizeof(receivedData));
        
        // Handle start/stop signals
        if (receivedData.AUX) {
            // Start signal received
            if (decisionState == IDLE) {
                decisionState = SEARCHFORCORNER;
                searchForCornerState = MOVEBACKWARDSUNTILEDGE;
            }
        } else {
            // Stop signal received
            if (decisionState != IDLE) {
                decisionState = IDLE;
                stopMotors();
            }
        }
    } else {
        // Check for signal loss
        if (millis() - lastSignalTime > 50) {
            if (!signalLost) {
                signalLost = true;
            }
            // Emergency stop on signal loss
            if (decisionState != IDLE) {
                decisionState = IDLE;
                stopMotors();
            }
        }
    }
    
    updateData();
    updatePosition();
    makeDecision();
    
    // Inner loop state machine
    switch (innerLoopState) {
        case FOLLOWINNERPATH:
            followInnerPath();
            break;
        case TURNLEFTINNER:
            turnLeft(90);
            break;
        case DONE:
            // Inner loop complete, move to next state
            decisionState = DONE;
            break;
    }
}

// ==================== Function Implementations ====================
// Core functions
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
}

void performIMUCalibration() {
    float accelSum[3] = {0, 0, 0};
    float gyroSum[3] = {0, 0, 0};
    
    // Serial.println("Collecting calibration data...");
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        if (ICM.dataReady()) {
            ICM.getAGMT();
            
            // Accumulate sensor readings
            accelSum[0] += ICM.accX();
            accelSum[1] += ICM.accY();
            accelSum[2] += ICM.accZ() - 1000.0; // Subtract 1g from Z
            
            gyroSum[0] += ICM.gyrX();
            gyroSum[1] += ICM.gyrY();
            gyroSum[2] += ICM.gyrZ();
            
            delay(1000 / SAMPLE_RATE);
            
            if (i % 50 == 0) {
                // Serial.print("Calibration progress: ");
                // Serial.print(i * 100 / CALIBRATION_SAMPLES);
                // Serial.println("%");
            }
        }
    }
    
    // Calculate biases
    for (int i = 0; i < 3; i++) {
        imuData.accelBias[i] = accelSum[i] / CALIBRATION_SAMPLES;
        imuData.gyroBias[i] = gyroSum[i] / CALIBRATION_SAMPLES;
    }
    
    // Apply calibration
    ICM.setBiasAccelX((int32_t)(imuData.accelBias[0] * 16384.0f));
    ICM.setBiasAccelY((int32_t)(imuData.accelBias[1] * 16384.0f));
    ICM.setBiasAccelZ((int32_t)(imuData.accelBias[2] * 16384.0f));
    
    ICM.setBiasGyroX((int32_t)(imuData.gyroBias[0] * 16.4f));
    ICM.setBiasGyroY((int32_t)(imuData.gyroBias[1] * 16.4f));
    ICM.setBiasGyroZ((int32_t)(imuData.gyroBias[2] * 16.4f));
    
    imuData.isCalibrated = true;
    // Serial.println("IMU calibration complete.");
}

void getIMUData() {
    if (ICM.dataReady()) {
    unsigned long currentTime = millis();
        float deltaTime = (currentTime - lastIMUTime) / 1000.0; // Convert to seconds
        
        ICM.getAGMT();
        
        // Read accelerometer data
        imuData.accel[0] = ICM.accX() - imuData.accelBias[0];
        imuData.accel[1] = ICM.accY() - imuData.accelBias[1];
        imuData.accel[2] = ICM.accZ() - imuData.accelBias[2] - 1000.0; // Subtract 1g from Z
        
        // Read gyroscope data
        imuData.gyro[0] = ICM.gyrX() - imuData.gyroBias[0];
        imuData.gyro[1] = ICM.gyrY() - imuData.gyroBias[1];
        imuData.gyro[2] = ICM.gyrZ() - imuData.gyroBias[2];
        
        // Update orientation using gyroscope data with actual elapsed time
        imuData.orientation[0] += imuData.gyro[0] * deltaTime;
        imuData.orientation[1] += imuData.gyro[1] * deltaTime;
        imuData.orientation[2] += imuData.gyro[2] * deltaTime;
        
        lastIMUTime = currentTime;
    }
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
                 
    }
}

//add aidans encoder velocity processing
void encoderISR_L() {
    unsigned long currentTime = millis();
    
    // Debounce check - ignore readings too close together
    if (currentTime - lastEncoderTime < ENCODER_DEBOUNCE_TIME) {
        return;
    }
    
    float deltaTime = (currentTime - lastEncoderTime) / 1000.0; // Time in seconds
    
    // Calculate the change in encoder count for the left encoder
    long deltaEncoder = leftEncoderCount - previousLeftEncoderCount;
    float step = deltaEncoder / MIN_ANGLE;
    
    // Handle immediate reversal (first reading)
    if (prevStepL == 0) {
        prevStepL = step;
        discretePosL += step * MIN_ANGLE;
        return;
    }
    
    // Handle zero-crossing
    if ((prevStepL != 0) && (prevStepL == -step)) {
        discretePosL += step * MIN_ANGLE;
        leftEncoderVelocity = 0;
        timeBetweenL = 0;
        prevStepL = step;
        sinceLastChangeL = 0;
        return;
    }
    
    // Handle complete step
    if (step != 0) {
        discretePosL += step * MIN_ANGLE;
        leftEncoderVelocity = step * MIN_ANGLE / deltaTime;
        timeBetweenL = 0;
        prevStepL = step;
        sinceLastChangeL = 0;
    }
    
    // Zero velocity timeout
    if (timeBetweenL > ZERO_TIME) {
        leftEncoderVelocity = 0;
        timeBetweenL = ZERO_TIME;
    }
    
    timeBetweenL += deltaTime;
    sinceLastChangeL++;
    previousLeftEncoderCount = leftEncoderCount;
    lastEncoderTime = currentTime;
}

void encoderISR_R() {
    unsigned long currentTime = millis();

    // Debounce check - ignore readings too close together
    if (currentTime - lastEncoderTime < ENCODER_DEBOUNCE_TIME) {
        return;
    }
    
    float deltaTime = (currentTime - lastEncoderTime) / 1000.0; // Time in seconds
    
    // Calculate position change
    long deltaEncoder = rightEncoderCount - previousRightEncoderCount;
    float step = deltaEncoder / MIN_ANGLE;
    
    // Handle immediate reversal (first reading)
    if (prevStepR == 0) {
        prevStepR = step;
        discretePosR += step * MIN_ANGLE;
        return;
    }
    
    // Handle zero-crossing
    if ((prevStepR != 0) && (prevStepR == -step)) {
        discretePosR += step * MIN_ANGLE;
        rightEncoderVelocity = 0;
        timeBetweenR = 0;
        prevStepR = step;
        sinceLastChangeR = 0;
        return;
    }
    
    // Handle complete step
    if (step != 0) {
        discretePosR += step * MIN_ANGLE;
        rightEncoderVelocity = step * MIN_ANGLE / deltaTime;
        timeBetweenR = 0;
        prevStepR = step;
        sinceLastChangeR = 0;
    }
    
    // Zero velocity timeout
    if (timeBetweenR > ZERO_TIME) {
        rightEncoderVelocity = 0;
        timeBetweenR = ZERO_TIME;
    }
    
    timeBetweenR += deltaTime;
    sinceLastChangeR++;
    previousRightEncoderCount = rightEncoderCount;
    lastEncoderTime = currentTime;
}

// **Update position based on encoder readings**
void updatePosition() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
    
    // Get track speeds from encoders
    float l_speed = leftEncoderVelocity;
    float r_speed = rightEncoderVelocity;
    
    // Store speeds in position information
    currentPosition.l_speed = l_speed;
    currentPosition.r_speed = r_speed;
    
    // Calculate accelerations (using velocity changes)
    float l_accel = (l_speed - currentPosition.l_speed) / deltaTime;
    float r_accel = (r_speed - currentPosition.r_speed) / deltaTime;
    
    // Compute center of mass velocities and turn rates
    float linear_velocity = getLinearVelocity();
    float turn_rate = (r_speed - l_speed) * WHEEL_RADIUS / AXLE_LENGTH;
    
    // Compute accelerations
    float linear_accel = (l_accel + r_accel) * WHEEL_RADIUS / 2;
    float turn_accel = (l_accel - r_accel) * WHEEL_RADIUS / 2;
    
    // Update position information
    currentPosition.linear_accel = linear_accel;
    currentPosition.linear_velocity = linear_velocity;
    currentPosition.turn_rate = turn_rate;
    currentPosition.turn_accel = turn_accel;
    
    // Compute full travel using actual elapsed time
    computeTravel(linear_velocity, turn_rate, deltaTime);
    
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
    // Read back limit switch states
    bool left_back_off = !limitSwitchConfigs[4].isPressed && !limitSwitchConfigs[5].isPressed;  // LBO and LBI
    bool right_back_off = !limitSwitchConfigs[6].isPressed && !limitSwitchConfigs[7].isPressed; // RBO and RBI
    
    // If both back limit switches are off, we are aligned
    if (left_back_off && right_back_off) {
        searchForCornerState = TURNRIGHT;
        stopMotors();
        return;
    }
    
    // If only the left back switch is off, rotate counterclockwise (CCW)
    if (left_back_off) {
        setTrajectory(0.01, -0.1);  // Small forward + left turn
    }
    // If only the right back switch is off, rotate clockwise (CW)
    else if (right_back_off) {
        setTrajectory(0.01, 0.1);  // Small forward + right turn
    }
    else {
        setTrajectory(-0.01, 0);  // Move backward slightly
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
    // Read limit switch states
    bool rfo = limitSwitchConfigs[2].isPressed;  // RFO
    bool rfi = limitSwitchConfigs[3].isPressed;  // RFI
    bool rbo = limitSwitchConfigs[6].isPressed;  // RBO
    bool rbi = limitSwitchConfigs[7].isPressed;  // RBI
    
    if (!rbo && !rbi) {  // Both back switches are off
        if (!rfo && !rfi) {  // All switches are off
            setTrajectory(0.02, 0.1);  // Forward and CCW
        } else {  // One or both front switches are on
            setTrajectory(0.02, -0.1);  // Forward and CW
        }
    }
    else if (rbo && rbi) {  // Both back switches are on
        setTrajectory(-0.02, 0.1);  // Backward and CCW
    }
    else {  // One back switch is off and one is on
        if (rfo && rfi) {  // Both front switches are on
            setTrajectory(-0.02, 0);  // Reverse straight back
        }
        else if (!rfo && !rfi) {  // Neither front switch is on
            setTrajectory(-0.02, 0);  // Reverse straight back
        }
        else {
        stopMotors();
        searchForCornerState = MOVEBACKWARDSUNTILCORNER;
        }
    }
}

void moveBackwardUntilCorner() {
    setTrajectory(-0.05, 0);  // Move backward at 5 cm/s
    
    // Check for corner condition: both left and right back outer switches pressed
    if (!limitSwitchConfigs[4].isPressed && !limitSwitchConfigs[6].isPressed) {  // LBO and RBO
        stopMotors();
        searchForCornerState = SEARCHDONE;
    }
}

void followEdge() {
    float speed = 0.03;  // Move forward slowly
    
    // Read limit switch states
    bool rfo = limitSwitchConfigs[2].isPressed;  // RFO
    bool rfi = limitSwitchConfigs[3].isPressed;  // RFI
    bool lfo = limitSwitchConfigs[0].isPressed;  // LFO
    bool lfi = limitSwitchConfigs[1].isPressed;  // LFI
    
    // If the left front switches went off, we reached a corner
    if (!lfo || !lfi) {
        stopMotors();
        
        // Store corner location
        if (cornerCount < 5) {
            cornerLocations[cornerCount].x = currentPosition.position[0];
            cornerLocations[cornerCount].y = currentPosition.position[1];
            cornerCount++;
        }
        
        // If this is the last corner
        if (cornerCount == 5) {
            // Compute the number of inner nodes
            mapInnerPath();
            outerLoopState = OUTERDONE;
        } else {
            outerLoopState = TURNLEFT;
        }
        return;
    }
    
    // Adjust turn rate based on limit switch readings
    if (!rfi) {
        // Inner limit switch went off, adjust slight CCW
        setTrajectory(speed/2, 0.1);
    }
    else if (rfo) {
        // Outer limit switch went on, adjust slight CW
        setTrajectory(speed/2, -0.1);
    }
    else {
        setTrajectory(speed, 0);
    }
}

void followInnerPath() {
    // Check limit switches for safety first
    if (!checkLimitSwitches()) {
        stopMotors();
        radioMessage = ERROR;
        return;
    }
    
    // If no desired distance set, calculate next path
    if (desiredDistance == 0) {
        if (cleaningAxis == "height") {
            desiredDistance = panelHeightNodes * AXLE_LENGTH * 0.75;
        } else {
            desiredDistance = panelWidthNodes * AXLE_LENGTH * 0.75;
        }
        
        // Check if we're done with this axis
        if (desiredDistance == 0) {
            innerLoopState = INNERDONE;
            stopMotors();
            return;
        }
        
        // Set initial azimuth for this path
        desiredAzimuth = getCurrentAzimuth();
    }
    
    // Update trajectory
    updatePositionalTrajectory();
    
    // Check if we've reached the end of this path
    if (desiredDistance <= 0) {
        desiredDistance = 0;
        desiredAzimuth = 0;
        stopMotors();
        innerLoopState = TURNLEFTINNER;
    }
}

void stopMotors() {
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
    pidLeft.SetMode(MANUAL);
    pidRight.SetMode(MANUAL);
    lockoutCountdown = LOCKOUT_COUNTDOWN;
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
    // Handle motor lockout
    if (lockoutCountdown > 0) {
        lockoutCountdown--;
        return;
    }
    
    // Re-enable PID when lockout is done
    if (lockoutCountdown == 0) {
        pidLeft.SetMode(AUTOMATIC);
        pidRight.SetMode(AUTOMATIC);
    }
    
    switch (decisionState) {
        case IDLE:
            // Wait for start command
            if (radioMessage == STARTCLEANINGOK) {
                decisionState = SEARCHFORCORNER;
                searchForCornerState = MOVEBACKWARDSUNTILEDGE;
            }
            break;
            
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
    // Serial.println("Initializing cleaning...");
    digitalWrite(CLEANING_MOTOR_ENABLE, HIGH); // Turn on cleaning motor
    decisionState = CLEANOUTERLOOP;
}

void cleanOuterLoop() {
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
    switch (innerLoopState) {
        case FOLLOWINNERPATH:
            followInnerPath();
            break;
            
        case TURNLEFT:
            turnLeft(90);
            break;
            
        case DONE:
            stopMotors();
            decisionState = DONE;
            break;
    }
}

void whenDone() {
    // Serial.println("Cleaning complete. Sending radio message.");
    stopMotors();
    digitalWrite(CLEANING_MOTOR_ENABLE, LOW);
    radioMessage = CLEANINGDONETAKEMEAWAY;
    decisionState = DONE;
}

bool checkLimitSwitches() {
    // Check front limit switches (first 4 switches)
    bool frontSwitchesOk = limitSwitchConfigs[0].isPressed && // LFO
                          limitSwitchConfigs[1].isPressed && // LFI
                          limitSwitchConfigs[2].isPressed && // RFO
                          limitSwitchConfigs[3].isPressed;   // RFI
                          
    return frontSwitchesOk;
}

float getCurrentAzimuth() {
    // Get current azimuth from IMU orientation
    return imuData.orientation[2];  // Yaw angle in radians
}

void updatePositionalTrajectory() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
    
    // Update position PID
    posPidInput = desiredDistance;
    posPidSetpoint = 0;  // We want to reduce distance to zero
    pidPosition.Compute();
    float desiredSpeed = posPidOutput;
    
    // Update orientation PID
    oriPidInput = getCurrentAzimuth();
    oriPidSetpoint = desiredAzimuth;
    pidOrientation.Compute();
    float turnRate = oriPidOutput;
    
    // Update desired distance using actual elapsed time
    desiredDistance -= getLinearVelocity() * deltaTime;
    
    // Set motor speeds
    setTrajectory(desiredSpeed, turnRate);
    
    // Update last time
    lastUpdateTime = currentTime;
}

void updateTurningTrajectory() {
    // Update orientation PID
    oriPidInput = getCurrentAzimuth();
    oriPidSetpoint = desiredAzimuth;
    pidOrientation.Compute();
    float turnRate = oriPidOutput;
    
    setTrajectory(0, turnRate);
}

float calculatePID(float setpoint, float input) {
    // Simple proportional control for now
    // Can be expanded to full PID if needed
    return (setpoint - input) * 0.5;  // Proportional gain of 0.5
}

void mapInnerPath() {
    // Calculate panel dimensions from corner locations
    panelHeight = (calculateDistance(cornerLocations[0], cornerLocations[1]) + 
                  calculateDistance(cornerLocations[2], cornerLocations[3])) / 2;
    panelWidth = (calculateDistance(cornerLocations[1], cornerLocations[2]) + 
                 calculateDistance(cornerLocations[3], cornerLocations[0])) / 2;
    
    // Calculate number of nodes for inner cleaning pattern
    panelHeightNodes = (int)(panelHeight / (AXLE_LENGTH * 0.75)) - 1;
    panelWidthNodes = (int)(panelWidth / (AXLE_LENGTH * 0.75)) - 2;
    
    // Initialize cleaning axis
    cleaningAxis = "height";
}

float calculateDistance(Point p1, Point p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrt(dx*dx + dy*dy);
}

float getLinearVelocity() {
    // Calculate linear velocity from wheel speeds
    return (leftEncoderVelocity + rightEncoderVelocity) * WHEEL_RADIUS / 2;
}

void computeTravel(float linear_velocity, float turn_rate, float deltaTime) {
    float d_theta = turn_rate * deltaTime;  // rotation amount, radians
    float d_s = linear_velocity * deltaTime; // distance amount, m
    
    // Update azimuth
    currentPosition.azimuth = fmod((currentPosition.azimuth + d_theta), (2 * PI));
    
    // Update orientation vector
    float x = currentPosition.orientation[0];
    float y = currentPosition.orientation[1];
    currentPosition.orientation[0] = x*cos(d_theta) - y*sin(d_theta);
    currentPosition.orientation[1] = x*sin(d_theta) + y*cos(d_theta);
    
    // If track speeds are equal (within small threshold), move straight
    if (abs(currentPosition.r_speed - currentPosition.l_speed) < 0.001) {
        currentPosition.position[0] += linear_velocity * cos(currentPosition.azimuth) * deltaTime;
        currentPosition.position[1] += linear_velocity * sin(currentPosition.azimuth) * deltaTime;
    } else {
        // Move along circle perimeter
        float turn_radius = AXLE_LENGTH/2 * (currentPosition.r_speed + currentPosition.l_speed) / 
                          (currentPosition.r_speed - currentPosition.l_speed);
        
        // Calculate displacement vector
        float displacement_x = x*cos(d_theta/2) - y*sin(d_theta/2);
        float displacement_y = x*sin(d_theta/2) + y*cos(d_theta/2);
        float displacement_mag = 2 * turn_radius * sin(d_theta/2);
        
        currentPosition.position[0] += displacement_x * displacement_mag;
        currentPosition.position[1] += displacement_y * displacement_mag;
    }
}

#include <Wire.h>
#include <PID_v1.h>
#include <MPU6050.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>



//debug
bool SkipRadio = true;

// ==================== Pin Definitions ====================
// Motor pins
#define MOTOR_L_PWM 5    // D5 - Left motor PWM (EN_A)
#define MOTOR_L_IN1 4    // D4 - Left motor direction 1
#define MOTOR_L_IN2 A1   // A1 - Left motor direction 2
#define MOTOR_R_PWM 6    // D6 - Right motor PWM (EN_B)
#define MOTOR_R_IN1 8    // D8 - Right motor direction 1
#define MOTOR_R_IN2 7    // D7 - Right motor direction 2
#define CLEANING_MOTOR_ENABLE 13  // D13 - Cleaning motor enable

// ESP32 Serial Communication
SoftwareSerial espSerial(A0, 255);  // RX = A0, TX unused (changed from D8 to avoid conflict)
bool LFO, RFI, RFO, LBO, LFI, RBO, LBI, RBI;  // Limit switch states

// Limit switch count
#define LIMIT_SWITCH_COUNT 8


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
const float TOP_VOLTAGE = 12;     // 12 V
const float TOP_SPEED = 0.08;     // 8 cm/s
const float TOP_TURN_RATE = 1.0;  // rad/s
const float MIN_ANGLE = 0.31415926535;      // Minimum angle change for velocity calculation
const float ZERO_TIME = 0.5;      // Time in seconds before zeroing velocity

// IMU Configuration
#define SAMPLE_RATE 100  // Hz
#define CALIBRATION_SAMPLES 250
#define VALIDATION_SAMPLES 100
#define STABILITY_THRESHOLD 0.1  // degrees/s for gyro, mg for accel
#define MPU 0x68  // MPU6050 I2C address

// Add IMU error variables
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;

// Add lockout constant after other constants
#define LOCKOUT_COUNTDOWN 20  // Matches simulation

// Add debounce constant after other constants
#define ENCODER_DEBOUNCE_TIME 10  // Minimum time between readings in ms

// Add MIN_TIME constant after other constants
#define MIN_TIME 20  // Define MIN_TIME as needed

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

struct SensorData {
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
const int numReadings = 20;
float readingsL[numReadings] = {0};
float readingsR[numReadings] = {0};
int indexL = 0;
int indexR = 0;
bool filledL = false;
bool filledR = false;

volatile float leftEncoderVelocity = 0.0;
volatile float rightEncoderVelocity = 0.0;
volatile long timeBetweenL = 0;
volatile long timeBetweenR = 0;
volatile short prevStepL = 0;
volatile short prevStepR = 0;
volatile unsigned long previousTimeL = millis();
volatile unsigned long previousTimeR = millis();

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
MPU6050 mpu;
IMUData imuData;
SensorData sensorData;
PositionalInformation currentPosition;
PositionalInformation estimatedPosition;
PID pidLeft(&pidInputL, &pidOutputL, &pidSetpointL, pidKp, pidKi, pidKd, DIRECT);
PID pidRight(&pidInputR, &pidOutputR, &pidSetpointR, pidKp, pidKi, pidKd, DIRECT);

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

// Encoder Reading Functions
float getLeftEncoderVelocity();
float getAvgLeftEncoderVelocity();
float getRightEncoderVelocity();
float getAvgRightEncoderVelocity();

// ==================== Setup and Main Loop ====================
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    espSerial.begin(38400);  // Initialize communication with ESP32
    
    while (!Serial) {
      Serial.println("no serial");
    }


    Serial.println("mpu init");

    // Initialize IMU
    Wire.begin();
    
    // Configure MPU6050
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Set to zero to wake up
    Wire.endTransmission(true);
    
    // Configure Accelerometer Sensitivity - Full Scale Range (+/- 8g)
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);  // ACCEL_CONFIG register
    Wire.write(0x10);  // Set to 0x10 for +/- 8g
    Wire.endTransmission(true);
    
    // Configure Gyro Sensitivity - Full Scale Range (1000deg/s)
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);  // GYRO_CONFIG register
    Wire.write(0x10);  // Set to 0x10 for 1000deg/s
    Wire.endTransmission(true);
    
    delay(20);
    
    // Calculate IMU error values
    calculate_IMU_error();
    delay(20);

    // Initialize radio
    if(SkipRadio){
        Serial.println("skip radio");
        decisionState = SEARCHFORCORNER;
    }
    else if (!radio.begin()) {
        // Radio hardware not responding
        while (1); // Stop if radio fails
        
    }else{
        radio.openReadingPipe(0, address);
        radio.setAutoAck(false);
        radio.setDataRate(RF24_1MBPS);
        radio.setPALevel(RF24_PA_MIN);
        radio.setPayloadSize(sizeof(ControllerData));
        radio.setChannel(100);
        radio.startListening();
        
        radioInitialized = true;
    }
    
    

    // Initialize other pins
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

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
    pidLeft.SetOutputLimits(-TOP_VOLTAGE, TOP_VOLTAGE);
    pidRight.SetOutputLimits(-TOP_VOLTAGE, TOP_VOLTAGE);
    pidPosition.SetOutputLimits(-TOP_SPEED, TOP_SPEED);
    pidOrientation.SetOutputLimits(-TOP_TURN_RATE, TOP_TURN_RATE);

    Serial.println("setup done");
}

void loop() {
    Serial.println("loopin?");
    // Check for radio signals
    if(SkipRadio){
      //
    }
    else if (radio.available()) {
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
                decisionState =  IDLE;
                stopMotors();
            }
        }
    }
    Serial.println("loopin?");
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

    
    // Update sensor data structure
    sensorData.accelX = imuData.accel[0];
    sensorData.accelY = imuData.accel[1];
    sensorData.accelZ = imuData.accel[2];
    sensorData.gyroX = imuData.gyro[0];
    sensorData.gyroY = imuData.gyro[1];
    sensorData.gyroZ = imuData.gyro[2];
    
    // Update encoder data
    sensorData.leftEncoderVelocity = getAvgLeftEncoderVelocity();
    sensorData.rightEncoderVelocity = getAvgRightEncoderVelocity();
    
    // Update IMU-based position and velocity
    sensorData.imuVelocity = sqrt(imuData.accel[0] * imuData.accel[0] + 
                                 imuData.accel[1] * imuData.accel[1]);
    sensorData.imuPosition = currentPosition.position[0]; // Using x position as example
}

void calculate_IMU_error() {
    Serial.println("calculating mpu error");
    int c = 0;
    
    // Read accelerometer values 200 times
    while (c < 200) {
        Wire.beginTransmission(MPU);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        float AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
        float AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
        float AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
        
        // Sum all readings
        AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
        AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
        c++;
    }
    
    //Divide the sum by 200 to get the error value
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    c = 0;
    
    // Read gyro values 200 times
    while (c < 200) {
        Wire.beginTransmission(MPU);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU, 6, true);
        float GyroX = Wire.read() << 8 | Wire.read();
        float GyroY = Wire.read() << 8 | Wire.read();
        float GyroZ = Wire.read() << 8 | Wire.read();
        
        // Sum all readings
        GyroErrorX = GyroErrorX + (GyroX / 131.0);
        GyroErrorY = GyroErrorY + (GyroY / 131.0);
        GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
        c++;
    }
    
    //Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;
}

void getIMUData() {
    // Read accelerometer data
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);  // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    // Convert raw values to actual acceleration in g
    imuData.accel[0] = (Wire.read() << 8 | Wire.read()) / 16384.0;  // X-axis
    imuData.accel[1] = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Y-axis
    imuData.accel[2] = (Wire.read() << 8 | Wire.read()) / 16384.0;  // Z-axis
    
    // Calculate angles from accelerometer
    accAngleX = (atan(imuData.accel[1] / sqrt(pow(imuData.accel[0], 2) + pow(imuData.accel[2], 2))) * 180 / PI) - AccErrorX;
    accAngleY = (atan(-1 * imuData.accel[0] / sqrt(pow(imuData.accel[1], 2) + pow(imuData.accel[2], 2))) * 180 / PI) - AccErrorY;
    
    // Read gyroscope data
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
    
    Wire.beginTransmission(MPU);
    Wire.write(0x43);  // Gyro data first register address
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    // Convert raw values to degrees per second and apply error correction
    imuData.gyro[0] = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorX;
    imuData.gyro[1] = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorY;
    imuData.gyro[2] = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorZ;
    
    // Convert gyro to radians/s
    imuData.gyro[0] *= PI / 180.0;
    imuData.gyro[1] *= PI / 180.0;
    imuData.gyro[2] *= PI / 180.0;
    
    // Calculate gyro angles
    gyroAngleX = gyroAngleX + imuData.gyro[0] * elapsedTime;
    gyroAngleY = gyroAngleY + imuData.gyro[1] * elapsedTime;
    yaw = yaw + imuData.gyro[2] * elapsedTime;
    
    // Complementary filter - combine accelerometer and gyro angle values
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
    
    // Update IMU orientation data
    imuData.orientation[0] = roll * PI / 180.0;  // Convert to radians
    imuData.orientation[1] = pitch * PI / 180.0;
    imuData.orientation[2] = yaw;
    
    // Update estimated position based on IMU data
    float linear_accel = sqrt(imuData.accel[0] * imuData.accel[0] + 
                             imuData.accel[1] * imuData.accel[1]);
    
    // Zero-velocity update (ZUPT) when acceleration is low and no movement
    if (linear_accel <= 0.25 && 
        abs(estimatedPosition.l_speed) < 0.001 && 
        abs(estimatedPosition.r_speed) < 0.001) {
        estimatedPosition.linear_accel = 0;
        estimatedPosition.linear_velocity = 0;
    } else {
        // Update velocity and position based on acceleration
        estimatedPosition.linear_velocity += linear_accel * elapsedTime;
        estimatedPosition.position[0] += estimatedPosition.orientation[0] * 
                                       estimatedPosition.linear_velocity * elapsedTime;
        estimatedPosition.position[1] += estimatedPosition.orientation[1] * 
                                       estimatedPosition.linear_velocity * elapsedTime;
    }
    
    // Update orientation vector
    float x = estimatedPosition.orientation[0];
    float y = estimatedPosition.orientation[1];
    estimatedPosition.orientation[0] = x*cos(imuData.gyro[2] * elapsedTime) - y*sin(imuData.gyro[2] * elapsedTime);
    estimatedPosition.orientation[1] = x*sin(imuData.gyro[2] * elapsedTime) + y*cos(imuData.gyro[2] * elapsedTime);
    
    // Update azimuth (yaw)
    estimatedPosition.azimuth = yaw;
    
    // Update turn rate
    estimatedPosition.turn_rate = imuData.gyro[2];
    
    lastIMUTime = currentTime;
}

// Function to update limit switch states
void updateLimitSwitches() {
    if (espSerial.available()) {
        String line = espSerial.readStringUntil('\n');
        
        // Convert characters directly to boolean values
        LFO = (int) line[0] - '0' == 1 ? true : false;  // Convert char to int
        RFI = (int) line[1] - '0' == 1 ? true : false;
        RFO = (int) line[2] - '0' == 1 ? true : false;
        LBO = (int) line[3] - '0' == 1 ? true : false;
        LFI = (int) line[4] - '0' == 1 ? true : false;
        RBO = (int) line[5] - '0' == 1 ? true : false;
        LBI = (int) line[6] - '0' == 1 ? true : false;
        RBI = (int) line[7] - '0' == 1 ? true : false;
    }
}

//add aidans encoder velocity processing
void encoderISR_L() {
    unsigned long currentTime = millis(); // get current time
    long timeBetween = currentTime - previousTimeL; // find delta T, only internal and don't update the global
    short stepL = 0; // define current step
  
    // if the time between is too short, act as software debounce and reject the input
    if (timeBetween < MIN_TIME) {
      return;
    }
    // Handle ghost backwards inputs
    if (digitalRead(ENCODER_L_B) != digitalRead(ENCODER_L_A)) {
      stepL = 1; // Invert these for the right
    }
  
    else {
      stepL = -1;
    }
  
    // Set previous time to current time, update global timeBetweenL
    timeBetweenL = timeBetween;
    previousTimeL = currentTime;  
  
    // Handle immediate reversal (first reading)
    if (prevStepL == 0) {
        prevStepL = stepL;
        return;
    }
      
    // Handle zero-crossing
    // (if the sign of the direction changes, assume the new current velocity is zero)
    if ((prevStepL != 0) && (prevStepL == -stepL)) {
        leftEncoderVelocity = 0;
        prevStepL = stepL;
        return;
    }
    // Handle complete step
    if (stepL != 0) {
        leftEncoderVelocity = 1000 * stepL * MIN_ANGLE / timeBetweenL;
        prevStepL = stepL;
    }
    return;
}

void encoderISR_R() {
    unsigned long currentTime = millis(); // get current time
    long timeBetween = currentTime - previousTimeR; // find delta T, only internal and don't update the global
    short stepR = 0; // define current step
  
    // if the time between is too short, act as software debounce and reject the input
    if (timeBetween < MIN_TIME) {
      return;
    }
    // Handle ghost backwards inputs
    if (digitalRead(ENCODER_R_B) != digitalRead(ENCODER_R_A)) {
      stepR = -1; // Invert these for the right
    }
  
    else {
      stepR = +1;
    }
  
    // Set previous time to current time, update global timeBetweenL
    timeBetweenR = timeBetween;
    previousTimeR = currentTime;  
  
    // Handle immediate reversal (first reading)
    if (prevStepR == 0) {
        prevStepR = stepR;
        return;
    }
      
    // Handle zero-crossing
    // (if the sign of the direction changes, assume the new current velocity is zero)
    if ((prevStepR != 0) && (prevStepR == -stepR)) {
        rightEncoderVelocity = 0;
        prevStepR = stepR;
        return;
    }
    // Handle complete step
    if (stepR != 0) {
        rightEncoderVelocity = 1000 * stepR * MIN_ANGLE / timeBetweenR;
        prevStepR = stepR;
    }
    return;
}

float getLeftEncoderVelocity() {
    unsigned long currentTime = millis();
    long timeBetween = currentTime - previousTimeL;
    if (timeBetween > ZERO_TIME) {
      leftEncoderVelocity = 0;
      previousTimeL = millis();
    }
    return leftEncoderVelocity;
}

float getRightEncoderVelocity() {
    unsigned long currentTime = millis();
    long timeBetween = currentTime - previousTimeR;
    if (timeBetween > ZERO_TIME) {
        rightEncoderVelocity = 0;
        previousTimeR = millis();
    }
    return rightEncoderVelocity;
}

float getAvgLeftEncoderVelocity() {
    readingsL[indexL] = getLeftEncoderVelocity();
    indexL = (indexL + 1) % numReadings;

    if (indexL == 0) filledL = true;

    float sum = 0;
    int count = filledL ? numReadings : indexL;
    for (int i = 0; i<count; i++) {
        sum += readingsL[i];
    }
    return sum / count;
}

float getAvgRightEncoderVelocity() {
    readingsR[indexR] = getRightEncoderVelocity();
    indexR = (indexR + 1) % numReadings;

    if (indexR == 0) filledR = true;

    float sum = 0;
    int count = filledR ? numReadings : indexR;
    for (int i = 0; i<count; i++) {
        sum += readingsR[i];
    }
    return sum / count;
}

// **Update position based on encoder readings**
void updatePosition() {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
    
    // Get track speeds from encoders
    float l_speed = getAvgLeftEncoderVelocity();
    float r_speed = getAvgRightEncoderVelocity();
    
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
    if (LBO || LBI || RBO || RBI) {
        stopMotors();
        searchForCornerState = ALIGNWITHEDGE;
    }
}

void alignWithEdge() {
    // Read back limit switch states
    bool left_back_off = !LBO && !LBI;
    bool right_back_off = !RBO && !RBI;
    
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
    if (!RBO && !RBI) {  // Both back switches are off
        if (!RFO && !RFI) {  // All switches are off
            setTrajectory(0.02, 0.1);  // Forward and CCW
        } else {  // One or both front switches are on
            setTrajectory(0.02, -0.1);  // Forward and CW
        }
    }
    else if (RBO && RBI) {  // Both back switches are on
        setTrajectory(-0.02, 0.1);  // Backward and CCW
    }
    else {  // One back switch is off and one is on
        if (RFO && RFI) {  // Both front switches are on
            setTrajectory(-0.02, 0);  // Reverse straight back
        }
        else if (!RFO && !RFI) {  // Neither front switch is on
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
    if (!LBO && !RBO) {
        stopMotors();
        searchForCornerState = SEARCHDONE;
    }
}

void followEdge() {
    float speed = 0.03;  // Move forward slowly
    
    // If the left front switches went off, we reached a corner
    if (!LFO || !LFI) {
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
    if (!RFI) {
        // Inner limit switch went off, adjust slight CCW
        setTrajectory(speed/2, 0.1);
    }
    else if (RFO) {
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
    // Stop both motors by setting all direction pins LOW
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
    // Set PWM to 0
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
    float finalOutputL = (pidOutputL + ffL);
    float finalOutputR = (pidOutputR + ffR);
    
    // Convert to PWM values (0-255)
    int pwmL = (int)constrain(abs(finalOutputL) * 255 / 12, 0, 255);
    int pwmR = (int)constrain(abs(finalOutputR) * 255 / 12, 0, 255);
    
    // Set direction pins based on sign of output
    digitalWrite(MOTOR_L_IN1, finalOutputL >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_L_IN2, finalOutputL >= 0 ? LOW : HIGH);
    digitalWrite(MOTOR_R_IN1, finalOutputR >= 0 ? HIGH : LOW);
    digitalWrite(MOTOR_R_IN2, finalOutputR >= 0 ? LOW : HIGH);
    
    // Apply PWM
    analogWrite(MOTOR_L_PWM, pwmL);
    analogWrite(MOTOR_R_PWM, pwmR);
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
    return LFO && RFI && RFO && LFI;
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
    return (getAvgLeftEncoderVelocity() + getAvgRightEncoderVelocity()) * WHEEL_RADIUS / 2;
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

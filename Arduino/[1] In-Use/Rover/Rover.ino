#include <Wire.h>
#include <PID_v1.h>

//next steps -> sensor processing -> sensor decision making -> edge detection handling (stop immediately, continue, or turn) -> motor pid / control -> line up pins to the right pinout (deal with analog pins appropriately), 

// Define motor pins
#define MOTOR_L_PWM 5
#define MOTOR_L_IN1 4
#define MOTOR_L_IN2 3

#define MOTOR_R_PWM 6
#define MOTOR_R_IN1 7
#define MOTOR_R_IN2 8

// Define cleaning motor pin
#define CLEANING_MOTOR_ENABLE 8
// Define limit switch pins
#define LIMIT_SWITCH_COUNT 8
#define LIMIT_SWITCH_PIN_SELECTOR_0 0
#define LIMIT_SWITCH_PIN_SELECTOR_1 1
#define LIMIT_SWITCH_PIN_SELECTOR_2 2
#define LIMIT_SWITCH_PIN_OUTPUT 3

// IMU (Placeholder for actual IMU library)
#define IMU_SDA A4
#define IMU_SCL A5

// Rotary Encoders
#define ENCODER_L_A 18
#define ENCODER_L_B 19
#define ENCODER_R_A 20
#define ENCODER_R_B 21

volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

volatile long previousLeftEncoderCount = 0;
volatile long previousRightEncoderCount = 0;
volatile unsigned long previousEncoderTime = 0;  // For tracking time between encoders

// Define variables to store encoder velocity (calculated in ISR)
volatile float leftEncoderVelocity = 0.0;
volatile float rightEncoderVelocity = 0.0;

// PID control variables
double pidInputL, pidOutputL, pidSetpointL;
double pidInputR, pidOutputR, pidSetpointR;

// Initialize PID controllers for left and right motors
PID pidLeft(&pidInputL, &pidOutputL, &pidSetpointL, 2.0, 5.0, 1.0, DIRECT);
PID pidRight(&pidInputR, &pidOutputR, &pidSetpointR, 2.0, 5.0, 1.0, DIRECT);

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

// Struct to store sensor data
struct SensorData {
    bool limitSwitches[LIMIT_SWITCH_COUNT];
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    long leftEncoder, rightEncoder;
    long leftEncoderVelocity, rightEncoderVelocity;
    float lastPosition = 0.0, imuVelocity = 0.0, imuPosition = 0.0;    
};

SensorData sensorData;

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
    // Update limit switch states (no changes)
    updateLimitSwitches();

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

// Method to update limit switch states (no changes)
void updateLimitSwitches() {
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        int selectorIndex = i / 2;  // Select 1 bit for each multiplexer channel
        int selectorBit = i % 2;   // Select low or high bit
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_0, (selectorIndex & 0x01) ? HIGH : LOW);  // Selector 0
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_1, (selectorIndex & 0x02) ? HIGH : LOW);  // Selector 1
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_2, (selectorBit & 0x01) ? HIGH : LOW);    // Selector 2
        delay(5);  // make sure to read correct channel from mux
        sensorData.limitSwitches[i] = (digitalRead(LIMIT_SWITCH_PIN_OUTPUT) == LOW);  // Assumes active-low switches
    }
}

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
    Serial.print("Updating position: L Speed = ");
    Serial.print(l_speed);
    Serial.print(", R Speed = ");
    Serial.println(r_speed);
}

// **Set motion trajectory**
void setTrajectory(float desiredSpeed, float desiredTurnRate) {
    l_speed = (desiredSpeed - (0.170 * desiredTurnRate) / 2) / 0.005;
    r_speed = (desiredSpeed + (0.170 * desiredTurnRate) / 2) / 0.005;
}

void moveBackwardUntilEdge() {
    setTrajectory(-0.05, 0);
    if (sensorData.limitSwitches[6] || sensorData.limitSwitches[7]) {
        stopMotors();
        searchForCornerState = ALIGNWITHEDGE;
    }
}

void alignWithEdge() {
    if (sensorData.limitSwitches[6] && sensorData.limitSwitches[7]) {
        searchForCornerState = TURNRIGHT;
        return;
    }
    if (sensorData.limitSwitches[6]) setTrajectory(0, 0.1);
    else if (sensorData.limitSwitches[7]) setTrajectory(0, -0.1);
}

void turnRight(int deg) {
    setTrajectory(0, -0.1);
    if (abs(deg - 90) < 5) {
        stopMotors();
        searchForCornerState = ADJUSTBACKANDFORTH;
    }
}

void turnLeft(int deg) {
    setTrajectory(0, 0.1);
    if (abs(deg - 90) < 5) {
        stopMotors();
        searchForCornerState = MOVEBACKWARDSUNTILCORNER;
    }
}

void adjustBackAndForth() {
    if (sensorData.limitSwitches[6] && sensorData.limitSwitches[7]) {
        setTrajectory(0.01, 0);
    } else if (!sensorData.limitSwitches[6] && !sensorData.limitSwitches[7]) {
        setTrajectory(-0.01, 0);
    } else {
        stopMotors();
        searchForCornerState = MOVEBACKWARDSUNTILCORNER;
    }
}

void moveBackwardUntilCorner() {
    setTrajectory(-0.05, 0);
    if (sensorData.limitSwitches[6] && sensorData.limitSwitches[7]) {
        stopMotors();
        searchForCornerState = SEARCHDONE;
    }
}

void followEdge() {
    float speed = 0.05;
    float turnRate = 0;
    if (sensorData.limitSwitches[0] && sensorData.limitSwitches[1]) turnRate = 0;
    else if (sensorData.limitSwitches[0]) turnRate = -0.05;
    else if (sensorData.limitSwitches[1]) turnRate = 0.05;
    setTrajectory(speed, turnRate);
}

void followInnerPath() {
    float speed = 0.05;
    float turnRate = 0.1;
    setTrajectory(speed, turnRate);
}

void stopMotors() {
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
}

void updateMotors() {
    pidInputL = l_speed;
    pidInputR = r_speed;
    pidSetpointL = l_speed;
    pidSetpointR = r_speed;
    pidLeft.Compute();
    pidRight.Compute();
    analogWrite(MOTOR_L_PWM, pidOutputL);
    analogWrite(MOTOR_R_PWM, pidOutputR);
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

#include <Wire.h>
#include <PID_v1.h>

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

// Struct to store sensor data
struct SensorData {
    bool limitSwitches[LIMIT_SWITCH_COUNT];
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    long leftEncoder;
    long rightEncoder;
};

SensorData sensorData;

// Function prototypes
void updateData();

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
// **Update All Sensor Data**
void updateData() {
    // Read limit switch states
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        int selectorIndex = i / 2; // Select 1 bit for each multiplexer channel
        int selectorBit = i % 2;   // Select low or high bit

        // Write the selector pin values to the multiplexer
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_0, (selectorIndex & 0x01) ? HIGH : LOW); // Selector 0
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_1, (selectorIndex & 0x02) ? HIGH : LOW); // Selector 1
        digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_2, (selectorBit & 0x01) ? HIGH : LOW);   // Selector 2
        delay(5); // make sure to read correct channel from mux
        sensorData.limitSwitches[i] = (digitalRead(LIMIT_SWITCH_PIN_OUTPUT) == LOW); // Assumes active-low switches
    }
    // Placeholder IMU Data (Replace with actual IMU readings)
    sensorData.accelX = 0.0;
    sensorData.accelY = 0.0;
    sensorData.accelZ = 0.0;
    sensorData.gyroX = 0.0;
    sensorData.gyroY = 0.0;
    sensorData.gyroZ = 0.0;

    // Read encoder counts
    sensorData.leftEncoder = leftEncoderCount;
    sensorData.rightEncoder = rightEncoderCount;

    // Debug print sensor data
    Serial.print("Encoders: L=");
    Serial.print(sensorData.leftEncoder);
    Serial.print(", R=");
    Serial.print(sensorData.rightEncoder);
    Serial.print(" | Limit Switches: ");
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        Serial.print(sensorData.limitSwitches[i]);
        Serial.print(" ");
    }
    Serial.println();
}

// **Interrupt Service Routines for Encoders**
void encoderISR_L() {
    if (digitalRead(ENCODER_L_A) == digitalRead(ENCODER_L_B)) {
        leftEncoderCount++;
    } else {
        leftEncoderCount--;
    }
}

void encoderISR_R() {
    if (digitalRead(ENCODER_R_A) == digitalRead(ENCODER_R_B)) {
        rightEncoderCount++;
    } else {
        rightEncoderCount--;
    }
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

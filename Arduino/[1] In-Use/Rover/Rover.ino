#include <Wire.h>

// Define motor pins
#define MOTOR_L_PWM 5
#define MOTOR_L_DIR 4
#define MOTOR_R_PWM 6
#define MOTOR_R_DIR 7

// Define limit switch pins
#define LIMIT_SWITCH_COUNT 8
const int limitSwitchPins[LIMIT_SWITCH_COUNT] = {2, 3, 8, 9, 10, 11, 12, 13};

// IMU (Placeholder for actual IMU library)
#define IMU_SDA A4
#define IMU_SCL A5

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

// Function prototypes
void updatePosition();
void setTrajectory(float desiredSpeed, float desiredTurnRate);
void updateSensors();
void makeDecision();
void updateMotors();
void searchForCorner();
void initializeCleaning();
void cleanOuterLoop();
void cleanInnerLoops();
void whenDone();
void moveBackwardUntilEdge();
void alignWithEdge();
void turnRight(int deg);
void turnLeft(int deg);
void adjustBackAndForth();
void moveBackwardUntilCorner();
void followEdge();
void followInnerPath();
void stopMotors();

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C for IMU

    // Set motor pins as output
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_DIR, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_DIR, OUTPUT);

    // Set limit switch pins as input
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        pinMode(limitSwitchPins[i], INPUT_PULLUP);
    }

    Serial.println("Rover Initialized.");
}

void loop() {
    updateSensors();
    updatePosition();
    makeDecision();
}

// Update position based on motor speed
void updatePosition() {
    Serial.print("Updating position: L Speed = ");
    Serial.print(l_speed);
    Serial.print(", R Speed = ");
    Serial.println(r_speed);
}

// Set trajectory for motion
void setTrajectory(float desiredSpeed, float desiredTurnRate) {
    l_speed = (desiredSpeed - (0.170 * desiredTurnRate) / 2) / 0.005;
    r_speed = (desiredSpeed + (0.170 * desiredTurnRate) / 2) / 0.005;
}

// Read sensors (limit switches)
void updateSensors() {
    Serial.print("Limit Switches: ");
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        Serial.print(digitalRead(limitSwitchPins[i]));
        Serial.print(" ");
    }
    Serial.println();
}

// Main decision-making function
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

    // Update motors based on decision logic (to be implemented)
    updateMotors();
}

// Motor update logic (to be implemented later)
void updateMotors() {
    // TODO: Implement motor control based on state
}

// Search for the corner logic
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

// Initialize cleaning process
void initializeCleaning() {
    Serial.println("Starting cleaning process.");
    decisionState = CLEANOUTERLOOP;
}

// Outer loop cleaning
void cleanOuterLoop() {
    switch (outerLoopState) {
        case FOLLOWEDGE:
            followEdge();
            break;
        case TURNLEFT:
            turnLeft(90);
            break;
        case OUTERDONE:
            decisionState = CLEANINNERLOOPS;
            break;
    }
}

// Inner loop cleaning
void cleanInnerLoops() {
    switch (innerLoopState) {
        case FOLLOWINNERPATH:
            followInnerPath();
            break;
        case INNERDONE:
            decisionState = DONE;
            break;
    }
}

// End of cleaning process
void whenDone() {
    Serial.println("Cleaning Done.");
    stopMotors();
}

// Concrete actions

void moveBackwardUntilEdge() {
    Serial.println("Moving backward until edge detected.");
    setTrajectory(-0.25, 0);
    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        if (digitalRead(limitSwitchPins[i]) == LOW) {
            stopMotors();
            searchForCornerState = ALIGNWITHEDGE;
            return;
        }
    }
}

void alignWithEdge() {
    Serial.println("Aligning with edge.");
    searchForCornerState = TURNRIGHT;
}

void turnRight(int deg) {
    Serial.print("Turning right ");
    Serial.print(deg);
    Serial.println(" degrees.");
    setTrajectory(0, 0.5);
    delay(1000);
    stopMotors();
    searchForCornerState = ADJUSTBACKANDFORTH;
}

void turnLeft(int deg) {
    Serial.print("Turning left ");
    Serial.print(deg);
    Serial.println(" degrees.");
    setTrajectory(0, -0.5);
    delay(1000);
    stopMotors();
}

void adjustBackAndForth() {
    Serial.println("Adjusting back and forth.");
    searchForCornerState = MOVEBACKWARDSUNTILCORNER;
}

void moveBackwardUntilCorner() {
    Serial.println("Moving backward until corner detected.");
    searchForCornerState = SEARCHDONE;
}

void followEdge() {
    Serial.println("Following edge.");
}

void followInnerPath() {
    Serial.println("Following inner path.");
}

void stopMotors() {
    Serial.println("Stopping motors.");
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
}

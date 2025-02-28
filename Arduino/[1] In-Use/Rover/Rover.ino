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
PID pidLeft(&pidInputL, &pidOutputL, &pidSetpointL, 2.0, 5.0, 1.0, DIRECT);  // Adjust PID constants based on your needs
PID pidRight(&pidInputR, &pidOutputR, &pidSetpointR, 2.0, 5.0, 1.0, DIRECT);  // Adjust PID constants based on your needs

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
    bool limitSwitches[LIMIT_SWITCH_COUNT];  // Store limit switch states
    float accelX, accelY, accelZ;            // IMU accelerometer values
    float gyroX, gyroY, gyroZ;               // IMU gyroscope values
    long leftEncoder;                         // Left encoder count
    long rightEncoder;                        // Right encoder count
};

SensorData sensorData; // Global sensor data struct

// Function prototypes
void updateData();
void updatePosition();
void setTrajectory(float desiredSpeed, float desiredTurnRate);
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
void encoderISR_L();
void encoderISR_R();

void setup() {
    Serial.begin(115200);
    Wire.begin(); // Initialize I2C for IMU

    // Set motor pins as output
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);

    // Set limit switch pins as input
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_0, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_1, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_2, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_OUTPUT, INPUT_PULLUP);

    // Set up rotary encoders with interrupts
    pinMode(ENCODER_L_A, INPUT_PULLUP);
    pinMode(ENCODER_L_B, INPUT_PULLUP);
    pinMode(ENCODER_R_A, INPUT_PULLUP);
    pinMode(ENCODER_R_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderISR_R, CHANGE);

    // Initialize PID controllers
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

// **Main decision-making function**
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

// **Motor update logic (to be implemented)**
void updateMotors() {
    pidInputL = l_speed;
    pidInputR = r_speed;

    pidSetpointL = l_desired_speed;
    pidSetpointR = r_desired_speed;

    pidLeft.Compute();  // Update motor control for left track
    pidRight.Compute(); // Update motor control for right track

    // Set the motor PWM based on PID outputs
    analogWrite(MOTOR_L_PWM, pidOutputL);  // Adjust PWM range (0-255)
    analogWrite(MOTOR_R_PWM, pidOutputR);  // Adjust PWM range (0-255)
}

// **Stopping Motors**
void stopMotors() {
    Serial.println("Stopping motors.");
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
}

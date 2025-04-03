#include <SoftwareSerial.h>

// ESP32 Serial Communication
SoftwareSerial espSerial(A0, 255);  // RX = A0, TX unused
bool LFO, RFI, RFO, LBO, LFI, RBO, LBI, RBI;  // Limit switch states

// Motor pins
#define MOTOR_L_PWM 5    // D5 - Left motor PWM (EN_A)
#define MOTOR_L_IN1 4    // D4 - Left motor direction 1
#define MOTOR_L_IN2 A1   // A1 - Left motor direction 2
#define MOTOR_R_PWM 6    // D6 - Right motor PWM (EN_B)
#define MOTOR_R_IN1 8    // D8 - Right motor direction 1
#define MOTOR_R_IN2 7    // D7 - Right motor direction 2

// Constants
const float FORWARD_SPEED = 0.02;  // 2 cm/s (reduced from 3)
const float TURN_SPEED = 0.015;    // 1.5 cm/s (reduced from 2)
const float TURN_RATE = 0.3;       // rad/s (reduced from 0.5)
const float AXLE_LENGTH = 0.170;   // 170mm

// State machine
enum EdgeTracerState {
    FOLLOW_EDGE,
    TURN_LEFT,
    STOPPED
};

EdgeTracerState currentState = FOLLOW_EDGE;

// Add hysteresis variables
bool lastRFI = false;
bool lastRFO = false;
unsigned long lastAdjustTime = 0;
const unsigned long ADJUST_DELAY = 500;  // Minimum time between adjustments (ms)

void setup() {
    Serial.begin(115200);
    espSerial.begin(38400);  // Initialize communication with ESP32
    
    // Initialize motor control pins
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);
    
    // Set initial PWM values
    analogWrite(MOTOR_L_PWM, 150);  // 150/255 ≈ 59% duty cycle
    analogWrite(MOTOR_R_PWM, 150);
    
    // Start moving forward
    setMotors(FORWARD_SPEED, 0);
    
    Serial.println("Edge tracer initialized");
}

void loop() {
    // Update limit switch states
    updateLimitSwitches();
    
    // State machine
    switch (currentState) {
        case FOLLOW_EDGE:
            followEdge();
            break;
            
        case TURN_LEFT:
            turnLeft();
            break;
            
        case STOPPED:
            stopMotors();
            break;
    }
}

void updateLimitSwitches() {
    if (espSerial.available()) {
        String line = espSerial.readStringUntil('\n');
        
        // Convert characters directly to boolean values
        LFO = (int) line[0] - '0' == 1 ? true : false;
        RFI = (int) line[1] - '0' == 1 ? true : false;
        RFO = (int) line[2] - '0' == 1 ? true : false;
        LBO = (int) line[3] - '0' == 1 ? true : false;
        LFI = (int) line[4] - '0' == 1 ? true : false;
        RBO = (int) line[5] - '0' == 1 ? true : false;
        LBI = (int) line[6] - '0' == 1 ? true : false;
        RBI = (int) line[7] - '0' == 1 ? true : false;
        
        // Debug print
        Serial.print("LFO: "); Serial.print(LFO);
        Serial.print(" RFI: "); Serial.print(RFI);
        Serial.print(" RFO: "); Serial.print(RFO);
        Serial.print(" LFI: "); Serial.println(LFI);
    }
}

void followEdge() {
    // If we detect a corner (left front switches go off)
    if (!LFO || !LFI) {
        Serial.println("Corner detected!");
        currentState = TURN_LEFT;
        return;
    }
    
    // Only adjust if enough time has passed since last adjustment
    if (millis() - lastAdjustTime < ADJUST_DELAY) {
        return;
    }
    
    // Check for significant changes in limit switch states
    bool rfiChanged = (RFI != lastRFI);
    bool rfoChanged = (RFO != lastRFO);
    
    // Update last states
    lastRFI = RFI;
    lastRFO = RFO;
    
    // Adjust turn rate based on limit switch readings with hysteresis
    if (!RFI && rfiChanged) {
        // Inner limit switch went off, adjust slight CCW
        setMotors(TURN_SPEED, TURN_RATE);
        lastAdjustTime = millis();
    }
    else if (RFO && rfoChanged) {
        // Outer limit switch went on, adjust slight CW
        setMotors(TURN_SPEED, -TURN_RATE);
        lastAdjustTime = millis();
    }
    else {
        // Both switches are off or no significant change, move straight
        setMotors(FORWARD_SPEED, 0);
    }
}

void turnLeft() {
    static unsigned long turnStartTime = 0;
    static bool turnStarted = false;
    
    if (!turnStarted) {
        turnStartTime = millis();
        turnStarted = true;
        setMotors(0, TURN_RATE);  // Turn in place
    }
    
    // Turn for approximately 1.57 seconds (90 degrees at TURN_RATE)
    if (millis() - turnStartTime > 1570) {
        turnStarted = false;
        currentState = FOLLOW_EDGE;
        setMotors(FORWARD_SPEED, 0);
    }
}

void setMotors(float speed, float turnRate) {
    // Convert speed and turn rate to individual motor speeds
    float leftSpeed = speed - (turnRate * AXLE_LENGTH / 2);
    float rightSpeed = speed + (turnRate * AXLE_LENGTH / 2);
    
    // Set motor directions (reversed from previous version)
    digitalWrite(MOTOR_L_IN1, leftSpeed >= 0 ? LOW : HIGH);   // Changed from HIGH to LOW
    digitalWrite(MOTOR_L_IN2, leftSpeed >= 0 ? HIGH : LOW);   // Changed from LOW to HIGH
    digitalWrite(MOTOR_R_IN1, rightSpeed >= 0 ? LOW : HIGH);  // Changed from HIGH to LOW
    digitalWrite(MOTOR_R_IN2, rightSpeed >= 0 ? HIGH : LOW);  // Changed from LOW to HIGH
}

void stopMotors() {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, LOW);
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, LOW);
    analogWrite(MOTOR_L_PWM, 0);
    analogWrite(MOTOR_R_PWM, 0);
} 
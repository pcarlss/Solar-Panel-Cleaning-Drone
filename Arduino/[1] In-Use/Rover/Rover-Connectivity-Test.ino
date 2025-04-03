#include <Wire.h>

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

// Encoder variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile long previousLeftEncoderCount = 0;
volatile long previousRightEncoderCount = 0;

// Test state
enum TestState {
    INITIALIZING,
    TESTING_LIMIT_SWITCHES,
    TESTING_LEFT_MOTOR,
    TESTING_RIGHT_MOTOR,
    TESTING_CLEANING_MOTOR,
    TESTING_IMU,
    TESTING_ENCODERS,
    COMPLETE
};

TestState currentState = INITIALIZING;
unsigned long lastStateChange = 0;
const unsigned long STATE_DURATION = 5000; // 5 seconds per test
const unsigned long MOTOR_TEST_DURATION = 2000; // 2 seconds for motor tests

// Limit switch names for better readability
const char* limitSwitchNames[] = {
    "Left Front Outer", "Left Front Inner",
    "Right Front Outer", "Right Front Inner",
    "Left Back Outer", "Left Back Inner",
    "Right Back Outer", "Right Back Inner"
};

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize pins
    pinMode(MOTOR_L_PWM, OUTPUT);
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_PWM, OUTPUT);
    pinMode(MOTOR_R_IN1, OUTPUT);
    pinMode(MOTOR_R_IN2, OUTPUT);
    pinMode(CLEANING_MOTOR_ENABLE, OUTPUT);
    
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_0, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_1, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_2, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_OUTPUT, INPUT_PULLUP);
    
    pinMode(ENCODER_L_A, INPUT_PULLUP);
    pinMode(ENCODER_L_B, INPUT_PULLUP);
    pinMode(ENCODER_R_A, INPUT_PULLUP);
    pinMode(ENCODER_R_B, INPUT_PULLUP);
    
    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), encoderISR_L, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), encoderISR_R, CHANGE);
    
    Serial.println("\n=== Rover Connectivity Test ===");
    Serial.println("Starting initialization...");
}

void loop() {
    unsigned long currentTime = millis();
    
    switch (currentState) {
        case INITIALIZING:
            if (currentTime - lastStateChange > 2000) {
                Serial.println("\nInitialization complete. Starting tests...");
                currentState = TESTING_LIMIT_SWITCHES;
                lastStateChange = currentTime;
            }
            break;
            
        case TESTING_LIMIT_SWITCHES:
            if (currentTime - lastStateChange > STATE_DURATION) {
                Serial.println("\n=== Testing Limit Switches ===");
                Serial.println("Press each limit switch and verify the output:");
                
                for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
                    // Set multiplexer address
                    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_0, (i & 0x01) ? HIGH : LOW);
                    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_1, (i & 0x02) ? HIGH : LOW);
                    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_2, (i & 0x04) ? HIGH : LOW);
                    
                    delay(5); // Wait for multiplexer to settle
                    
                    bool isPressed = (digitalRead(LIMIT_SWITCH_PIN_OUTPUT) == LOW);
                    Serial.print(limitSwitchNames[i]);
                    Serial.print(": ");
                    Serial.println(isPressed ? "PRESSED" : "NOT PRESSED");
                }
                
                currentState = TESTING_LEFT_MOTOR;
                lastStateChange = currentTime;
            }
            break;
            
        case TESTING_LEFT_MOTOR:
            if (currentTime - lastStateChange > MOTOR_TEST_DURATION) {
                Serial.println("\n=== Testing Left Motor ===");
                Serial.println("Left motor will run forward for 2 seconds...");
                
                // Run left motor forward
                digitalWrite(MOTOR_L_IN1, HIGH);
                digitalWrite(MOTOR_L_IN2, LOW);
                analogWrite(MOTOR_L_PWM, 128); // 50% speed
                
                delay(2000);
                
                // Stop motor
                analogWrite(MOTOR_L_PWM, 0);
                Serial.println("Left motor test complete.");
                
                currentState = TESTING_RIGHT_MOTOR;
                lastStateChange = currentTime;
            }
            break;
            
        case TESTING_RIGHT_MOTOR:
            if (currentTime - lastStateChange > MOTOR_TEST_DURATION) {
                Serial.println("\n=== Testing Right Motor ===");
                Serial.println("Right motor will run forward for 2 seconds...");
                
                // Run right motor forward
                digitalWrite(MOTOR_R_IN1, HIGH);
                digitalWrite(MOTOR_R_IN2, LOW);
                analogWrite(MOTOR_R_PWM, 128); // 50% speed
                
                delay(2000);
                
                // Stop motor
                analogWrite(MOTOR_R_PWM, 0);
                Serial.println("Right motor test complete.");
                
                currentState = TESTING_CLEANING_MOTOR;
                lastStateChange = currentTime;
            }
            break;
            
        case TESTING_CLEANING_MOTOR:
            if (currentTime - lastStateChange > MOTOR_TEST_DURATION) {
                Serial.println("\n=== Testing Cleaning Motor ===");
                Serial.println("Cleaning motor will run for 2 seconds...");
                
                // Run cleaning motor
                digitalWrite(CLEANING_MOTOR_ENABLE, HIGH);
                
                delay(2000);
                
                // Stop motor
                digitalWrite(CLEANING_MOTOR_ENABLE, LOW);
                Serial.println("Cleaning motor test complete.");
                
                currentState = TESTING_IMU;
                lastStateChange = currentTime;
            }
            break;
            
        case TESTING_IMU:
            if (currentTime - lastStateChange > STATE_DURATION) {
                Serial.println("\n=== Testing IMU ===");
                Serial.println("Reading IMU data...");
                
                // Read IMU data
                Wire.beginTransmission(0x68);
                Wire.write(0x3B);
                Wire.endTransmission();
                Wire.requestFrom(0x68, 6);
                
                int16_t ax = Wire.read() << 8 | Wire.read();
                int16_t ay = Wire.read() << 8 | Wire.read();
                int16_t az = Wire.read() << 8 | Wire.read();
                
                Serial.print("Accelerometer: X=");
                Serial.print(ax);
                Serial.print(" Y=");
                Serial.print(ay);
                Serial.print(" Z=");
                Serial.println(az);
                
                currentState = TESTING_ENCODERS;
                lastStateChange = currentTime;
            }
            break;
            
        case TESTING_ENCODERS:
            if (currentTime - lastStateChange > STATE_DURATION) {
                Serial.println("\n=== Testing Encoders ===");
                Serial.println("Move the wheels and verify encoder counts:");
                
                Serial.print("Left Encoder: ");
                Serial.print(leftEncoderCount);
                Serial.print(" (Delta: ");
                Serial.print(leftEncoderCount - previousLeftEncoderCount);
                Serial.println(")");
                
                Serial.print("Right Encoder: ");
                Serial.print(rightEncoderCount);
                Serial.print(" (Delta: ");
                Serial.print(rightEncoderCount - previousRightEncoderCount);
                Serial.println(")");
                
                previousLeftEncoderCount = leftEncoderCount;
                previousRightEncoderCount = rightEncoderCount;
                
                currentState = COMPLETE;
                lastStateChange = currentTime;
            }
            break;
            
        case COMPLETE:
            if (currentTime - lastStateChange > 2000) {
                Serial.println("\n=== Test Complete ===");
                Serial.println("All components have been tested.");
                Serial.println("Please verify all readings are correct.");
                
                // Reset state to start over
                currentState = INITIALIZING;
                lastStateChange = currentTime;
            }
            break;
    }
}

// Encoder ISR functions
void encoderISR_L() {
    leftEncoderCount++;
}

void encoderISR_R() {
    rightEncoderCount++;
}

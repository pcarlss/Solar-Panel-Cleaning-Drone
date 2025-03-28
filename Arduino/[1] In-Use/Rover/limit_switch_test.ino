#include <Wire.h>

// ==================== Pin Definitions ====================
// Limit switch pins
#define LIMIT_SWITCH_COUNT 8
#define LIMIT_SWITCH_PIN_SELECTOR_0 8    // D8 - Multiplexer address bit 0
#define LIMIT_SWITCH_PIN_SELECTOR_1 A1   // A1 - Multiplexer address bit 1
#define LIMIT_SWITCH_PIN_SELECTOR_2 A0   // A0 - Multiplexer address bit 2
#define LIMIT_SWITCH_PIN_OUTPUT A7       // A7 - Multiplexer output

// Reference voltage monitoring
#define REFERENCE_PIN A6  // Use A6 to monitor reference voltage

// IR threshold values
#define IR_THRESHOLD 875  // Threshold for binary conversion
#define IR_MAX 1023      // Maximum IR reading (unblocked)
#define IR_MIN 0         // Minimum IR reading (blocked)

// Reading stability
#define SAMPLES_PER_READ 5  // Number of samples to average per reading
#define MUX_SETTLE_TIME 10  // Milliseconds to wait for multiplexer to settle

// ==================== Type Definitions ====================
struct SwitchPair {
    int innerRaw;     // Raw IR sensor reading
    int outerRaw;     // Raw IR sensor reading
    int innerBinary;  // Binary value (0/1)
    int outerBinary;  // Binary value (0/1)
    float average;    // Average of inner and outer readings
    float compensatedAverage;  // Temperature/power compensated average
};

// ==================== Global Variables ====================
SwitchPair switchPairs[4];  // Array of 4 pairs: [FR, FL, BR, BL]
int referenceReadings[10];  // Store last 10 reference readings
int refIndex = 0;          // Index for reference readings array

// ==================== Helper Functions ====================
float getReferenceVoltage() {
    // Take average of last 10 reference readings
    long sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += referenceReadings[i];
    }
    return sum / 10.0;
}

int readStableValue(int muxAddress) {
    // Set multiplexer address
    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_0, (muxAddress & 0x01) ? HIGH : LOW);
    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_1, (muxAddress & 0x02) ? HIGH : LOW);
    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_2, (muxAddress & 0x04) ? HIGH : LOW);
    
    delay(MUX_SETTLE_TIME); // Wait for multiplexer to settle
    
    // Take multiple samples and average them
    long sum = 0;
    for (int i = 0; i < SAMPLES_PER_READ; i++) {
        sum += analogRead(LIMIT_SWITCH_PIN_OUTPUT);
        delay(1); // Small delay between samples
    }
    return sum / SAMPLES_PER_READ;
}

// ==================== Setup and Main Loop ====================
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        // Wait for serial port to connect
    }
    
    // Initialize limit switch pins
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_0, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_1, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_SELECTOR_2, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_OUTPUT, INPUT);
    pinMode(REFERENCE_PIN, INPUT);
    
    // Set initial multiplexer state
    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_0, LOW);
    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_1, LOW);
    digitalWrite(LIMIT_SWITCH_PIN_SELECTOR_2, LOW);
    
    // Initialize reference readings
    for (int i = 0; i < 10; i++) {
        referenceReadings[i] = analogRead(REFERENCE_PIN);
        delay(10);
    }
    
    Serial.println("IR Limit Switch Test Program");
    Serial.println("Reading pairs: [FR, FL, BR, BL] where each pair is [inner, outer]");
    Serial.println("Format: [raw_inner(raw_bin), raw_outer(raw_bin)] avg=value comp=value ref=value");
    Serial.println("Example: [1023(1), 800(0)] avg=911.5 comp=900.0 ref=1023 means inner=1023 (not triggered), outer=800 (triggered)");
    Serial.println("----------------------------------------");
}

void readLimitSwitches() {
    // Read the reference voltage
    int referenceReading = analogRead(REFERENCE_PIN); // Assuming REFERENCE_PIN is defined
    int dynamicIRThreshold = referenceReading + 35; // Set dynamic threshold

    for (int i = 0; i < LIMIT_SWITCH_COUNT; i++) {
        // Read IR sensor value with averaging
        int irValue = readStableValue(i);
        bool isTriggered = (irValue < dynamicIRThreshold); // Use dynamic threshold
        
        // // Print raw IR value and dynamic threshold for debugging
        // Serial.print("IR Value for switch ");
        // Serial.print(i);
        // Serial.print(": ");
        // Serial.print(irValue);
        // Serial.print(" | Dynamic Threshold: ");
        // Serial.println(dynamicIRThreshold);
        
        // Map the 8 switches to 4 pairs in correct order [FR, FL, BR, BL]
        switch(i) {
            case 0: // RFI
                switchPairs[0].innerRaw = irValue;
                switchPairs[0].innerBinary = isTriggered ? 0 : 1;
                break;
            case 1: // RFO
                switchPairs[0].outerRaw = irValue;
                switchPairs[0].outerBinary = isTriggered ? 0 : 1;
                break;
            case 2: // LFI
                switchPairs[1].innerRaw = irValue;
                switchPairs[1].innerBinary = isTriggered ? 0 : 1;
                break;
            case 3: // LFO
                switchPairs[1].outerRaw = irValue;
                switchPairs[1].outerBinary = isTriggered ? 0 : 1;
                break;
            case 4: // RBI
                switchPairs[2].innerRaw = irValue;
                switchPairs[2].innerBinary = isTriggered ? 0 : 1;
                break;
            case 5: // RBO
                switchPairs[2].outerRaw = irValue;
                switchPairs[2].outerBinary = isTriggered ? 0 : 1;
                break;
            case 6: // LBI
                switchPairs[3].innerRaw = irValue;
                switchPairs[3].innerBinary = isTriggered ? 0 : 1;
                break;
            case 7: // LBO
                switchPairs[3].outerRaw = irValue;
                switchPairs[3].outerBinary = isTriggered ? 0 : 1;
                break;
        }
    }
    
    // Calculate averages and scaled values for each pair
    for (int i = 0; i < 4; i++) {
        switchPairs[i].average = (switchPairs[i].innerRaw + switchPairs[i].outerRaw) / 2.0;
        // Scale the average to 5V reference
        switchPairs[i].compensatedAverage = switchPairs[i].average * (1023.0 / referenceReading);
    }
}

void loop() {
    readLimitSwitches();
    
    // Print all pairs with both raw and binary values, plus averages
    Serial.print("Switch Pairs: ");
    for (int i = 0; i < 4; i++) {
        Serial.print("[");
        // Serial.print(switchPairs[i].innerRaw);
        Serial.print("(");
        Serial.print(switchPairs[i].innerBinary);
        Serial.print("),");
        // Serial.print(switchPairs[i].outerRaw);
        Serial.print("(");
        Serial.print(switchPairs[i].outerBinary);
        Serial.print(")] ");
    }
    Serial.println();
    
    // Print averages and reference voltage
    Serial.print("Averages: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(switchPairs[i].average);
        Serial.print(" ");
    }
    Serial.print("Compensated: ");
    for (int i = 0; i < 4; i++) {
        Serial.print(switchPairs[i].compensatedAverage);
        Serial.print(" ");
    }
    Serial.print("Ref: ");
    Serial.print(getReferenceVoltage());
    Serial.println();
    
    // delay(0); // 1 second delay between readings
} 
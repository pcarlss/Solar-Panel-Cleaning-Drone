#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

// ====== MPU6050 Setup ======
MPU6050 mpu;

// ====== ESP32 Serial Communication ======
SoftwareSerial espSerial(A0, 255);  // RX = A0, TX unused

// ====== Encoder Pins and Variables ======
#define CLK1 2
#define DT1 A2
#define CLK2 3
#define DT2 A3

volatile int encoderValue1 = 0;
volatile int encoderValue2 = 0;

// ====== Motor Control Pins ======
#define IN1 4
#define IN2 A1
#define IN3 8
#define IN4 7

#define EN_A 5
#define EN_B 6

int speedA = 150;
int speedB = 150;

// ====== Timing Variables ======
unsigned long previousMotorMillis = 0;
unsigned long previousPrintMillis = 0;
const unsigned long motorInterval = 5000;
const unsigned long printInterval = 200;

int motorState = 0; // 0 = forward, 1 = reverse, 2 = stop

void setup() {
    Serial.begin(115200);
    espSerial.begin(9600);
    Wire.begin();

    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    if (mpu.testConnection()) {
        Serial.println("MPU6050 connected!");
    } else {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    // Encoder Setup
    pinMode(CLK1, INPUT);
    pinMode(DT1, INPUT);
    attachInterrupt(digitalPinToInterrupt(CLK1), readEncoder1, CHANGE);

    pinMode(CLK2, INPUT);
    pinMode(DT2, INPUT);
    attachInterrupt(digitalPinToInterrupt(CLK2), readEncoder2, CHANGE);

    // Motor Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN_A, OUTPUT);
    pinMode(EN_B, OUTPUT);
    analogWrite(EN_A, speedA);
    analogWrite(EN_B, speedB);
}

void loop() {
    unsigned long currentMillis = millis();

    // ---- ESP32 Serial Read ----
    String espLine = "";
    if (espSerial.available()) {
        espLine = espSerial.readStringUntil('\n');
    }

    // ---- MPU6050 Data ----
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // ---- Print All Data ----
    if (currentMillis - previousPrintMillis >= printInterval) {
        previousPrintMillis = currentMillis;

        Serial.print("ESP32: ");
        Serial.print(espLine);
        Serial.print(" | Encoder 1: ");
        Serial.print(encoderValue1);
        Serial.print(" | Encoder 2: ");
        Serial.print(encoderValue2);

        Serial.print(" | Acc (mg): ");
        printFormattedFloat(ax / 16384.0, 5, 2); Serial.print(", ");
        printFormattedFloat(ay / 16384.0, 5, 2); Serial.print(", ");
        printFormattedFloat(az / 16384.0, 5, 2);

        Serial.print(" | Gyro (DPS): ");
        printFormattedFloat(gx / 131.0, 5, 2); Serial.print(", ");
        printFormattedFloat(gy / 131.0, 5, 2); Serial.print(", ");
        printFormattedFloat(gz / 131.0, 5, 2);
        Serial.println();
    }

    // ---- Motor State Machine ----
    if (currentMillis - previousMotorMillis >= motorInterval) {
        previousMotorMillis = currentMillis;

        switch (motorState) {
            case 0: // Forward
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, HIGH);
                motorState = 1;
                break;

            case 1: // Reverse
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
                motorState = 2;
                break;

            case 2: // Stop
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, LOW);
                analogWrite(EN_A, 0);
                analogWrite(EN_B, 0);
                delay(50); // brief brake
                analogWrite(EN_A, speedA);
                analogWrite(EN_B, speedB);
                motorState = 0;
                break;
        }
    }
}

// ====== Encoder Interrupts ======
void readEncoder1() {
    int stateCLK1 = digitalRead(CLK1);
    encoderValue1 += (digitalRead(DT1) != stateCLK1) ? 1 : -1;
}

void readEncoder2() {
    int stateCLK2 = digitalRead(CLK2);
    encoderValue2 += (digitalRead(DT2) != stateCLK2) ? 1 : -1;
}

// ====== Float Formatting Helper ======
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
    float absVal = abs(val);
    if (val < 0) Serial.print("-");
    else Serial.print(" ");

    for (uint8_t i = 0; i < leading; i++) {
        uint32_t tenPow = 1;
        for (uint8_t c = 0; c < (leading - 1 - i); c++) tenPow *= 10;
        if (absVal < tenPow) Serial.print("0");
        else break;
    }
    Serial.print(absVal, decimals);
}

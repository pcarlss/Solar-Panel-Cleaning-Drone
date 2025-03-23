#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
  
    // Initialize the MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();

    // Check if the connection was successful
    if (mpu.testConnection()) {
        Serial.println("MPU6050 connected!");
    } else {
        Serial.println("MPU6050 connection failed!");
        while (1); // Halt if sensor is not found
    }
}

void loop() {
    int16_t ax, ay, az, gx, gy, gz;

    // Get accelerometer and gyroscope data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Print scaled sensor values
    Serial.print("Acc (mg): ");
    printFormattedFloat(ax / 16384.0, 5, 2);
    Serial.print(", ");
    printFormattedFloat(ay / 16384.0, 5, 2);
    Serial.print(", ");
    printFormattedFloat(az / 16384.0, 5, 2);
    Serial.print(" | Gyro (DPS): ");
    printFormattedFloat(gx / 131.0, 5, 2);
    Serial.print(", ");
    printFormattedFloat(gy / 131.0, 5, 2);
    Serial.print(", ");
    printFormattedFloat(gz / 131.0, 5, 2);
    Serial.println();

}

// Helper function to format float values
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

/****************************************************************
 * Example1_Basics_I2C.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 ***************************************************************/

#include <Wire.h>
#include <MPU6050.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define MPU_ADDR 0x68  // MPU6050 I2C address

// IMU Configuration
#define SAMPLE_RATE 50  // Hz
#define CALIBRATION_SAMPLES 100
#define VALIDATION_SAMPLES 50
#define STABILITY_THRESHOLD 0.1  // mg for accel

// MPU6050 Register Addresses
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A

// Simplified data structures
struct IMUCalibrationData {
    float accelBias[3];
    // float gyroBias[3];  // Commented out gyro bias
};

struct IMUErrorMetrics {
    float accelRMS[3];
    // float gyroRMS[3];  // Commented out gyro RMS
};

// Global variables
IMUCalibrationData calData;
IMUErrorMetrics errorMetrics;
bool isCalibrated = false;
unsigned long lastSampleTime = 0;

// Function to read raw values from MPU6050
void readRawData(int16_t* accel, int16_t* gyro) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    
    accel[0] = Wire.read() << 8 | Wire.read();
    accel[1] = Wire.read() << 8 | Wire.read();
    accel[2] = Wire.read() << 8 | Wire.read();
    Wire.read(); // Skip temperature
    gyro[0] = Wire.read() << 8 | Wire.read();
    gyro[1] = Wire.read() << 8 | Wire.read();
    gyro[2] = Wire.read() << 8 | Wire.read();
}

// Function to calculate angles from accelerometer data
void calculateAngles(float accel[3], float* pitch, float* roll, float* yaw) {
    // Calculate pitch (rotation around X axis)
    *pitch = atan2(accel[1], accel[2]) * 180.0f / PI;
    
    // Calculate roll (rotation around Y axis)
    *roll = atan2(-accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2])) * 180.0f / PI;
    
    // Note: Yaw cannot be calculated from accelerometer alone
    // This is a placeholder that will always show 0
    *yaw = 0.0f;
}

// Function to initialize MPU6050
void initializeMPU() {
    Wire.begin();
    Wire.setClock(400000);
    
    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    
    // Set sample rate divider
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(SMPLRT_DIV);
    Wire.write(0x07);  // Sample rate = 1kHz/(1+7) = 125Hz
    Wire.endTransmission();
    
    // Set accelerometer range to ±4g
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_CONFIG);
    Wire.write(0x08);  // 0x08 = ±4g
    Wire.endTransmission();
    
    // Set gyroscope range to ±250°/s (commented out but kept for reference)
    /*
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_CONFIG);
    Wire.write(0x00);  // 0x00 = ±250°/s
    Wire.endTransmission();
    */
    
    // Set DLPF configuration
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(CONFIG);
    Wire.write(0x06);  // DLPF_CFG = 6 (44Hz bandwidth, 1kHz sampling)
    Wire.endTransmission();
}

void setup() {
    SERIAL_PORT.begin(115200);
    while (!SERIAL_PORT) {};
    
    initializeMPU();
    
    SERIAL_PORT.println("\nMPU6050 initialized. Starting calibration...");
    SERIAL_PORT.println("Keep IMU stationary during calibration.");
    
    performCalibration();
    validateCalibration();
    
    SERIAL_PORT.println("\nCommands:");
    SERIAL_PORT.println("c - Check calibration");
    SERIAL_PORT.println("v - Validate calibration");
    SERIAL_PORT.println("r - Reset calibration");
    SERIAL_PORT.println("a - Run accelerometer test");
    SERIAL_PORT.println("h - Print help");
}

void loop() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
        analyzeStability();
        lastSampleTime = currentTime;
    }
    
    if (SERIAL_PORT.available()) {
        char cmd = SERIAL_PORT.read();
        switch (cmd) {
            case 'c':
                printCalibrationStatus();
                break;
            case 'v':
                validateCalibration();
                break;
            case 'r':
                initializeMPU();
                isCalibrated = false;
                SERIAL_PORT.println("Calibration reset.");
                break;
            case 'a':
                testAccelerometerStability();
                break;
            case 'h':
                printHelp();
                break;
        }
    }
}

void performCalibration() {
    float accelSum[3] = {0, 0, 0};
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        readRawData(rawAccel, rawGyro);
        
        // Convert raw values to actual values
        accelSum[0] += rawAccel[0] / 8192.0f;  // ±4g = 8192 LSB/g
        accelSum[1] += rawAccel[1] / 8192.0f;
        accelSum[2] += (rawAccel[2] / 8192.0f) - 1.0f;  // Subtract 1g from Z
        
        delay(1000 / SAMPLE_RATE);
        
        if (i % 25 == 0) {
            SERIAL_PORT.print("Calibration: ");
            SERIAL_PORT.print(i * 100 / CALIBRATION_SAMPLES);
            SERIAL_PORT.println("%");
        }
    }
    
    for (int i = 0; i < 3; i++) {
        calData.accelBias[i] = accelSum[i] / CALIBRATION_SAMPLES;
    }
    
    isCalibrated = true;
    SERIAL_PORT.println("Calibration complete.");
}

void validateCalibration() {
    if (!isCalibrated) {
        SERIAL_PORT.println("IMU not calibrated!");
        return;
    }
    
    float accelError[3] = {0, 0, 0};
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    
    for (int i = 0; i < VALIDATION_SAMPLES; i++) {
        readRawData(rawAccel, rawGyro);
        
        float accel[3] = {
            rawAccel[0] / 8192.0f - calData.accelBias[0],
            rawAccel[1] / 8192.0f - calData.accelBias[1],
            (rawAccel[2] / 8192.0f) - calData.accelBias[2] - 1.0f
        };
        
        for (int j = 0; j < 3; j++) {
            accelError[j] += abs(accel[j]);
        }
        
        delay(1000 / SAMPLE_RATE);
    }
    
    for (int i = 0; i < 3; i++) {
        errorMetrics.accelRMS[i] = sqrt(accelError[i] / VALIDATION_SAMPLES);
    }
    
    printResults();
}

void analyzeStability() {
    static float lastAccel[3] = {0, 0, 0};
    
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    readRawData(rawAccel, rawGyro);
    
    float accel[3] = {
        rawAccel[0] / 8192.0f - calData.accelBias[0],
        rawAccel[1] / 8192.0f - calData.accelBias[1],
        (rawAccel[2] / 8192.0f) - calData.accelBias[2] - 1.0f
    };
    
    float accelDelta[3] = {
        abs(accel[0] - lastAccel[0]),
        abs(accel[1] - lastAccel[1]),
        abs(accel[2] - lastAccel[2])
    };
    
    bool isUnstable = false;
    for (int i = 0; i < 3; i++) {
        if (accelDelta[i] > STABILITY_THRESHOLD) {
            isUnstable = true;
            break;
        }
    }
    
    if (isUnstable) {
        SERIAL_PORT.println("Warning: IMU unstable!");
    }
    
    // Calculate and print angles
    float pitch, roll, yaw;
    calculateAngles(accel, &pitch, &roll, &yaw);
    
    SERIAL_PORT.print("Angles (deg) - Pitch: ");
    SERIAL_PORT.print(pitch);
    SERIAL_PORT.print(", Roll: ");
    SERIAL_PORT.print(roll);
    SERIAL_PORT.print(", Yaw: ");
    SERIAL_PORT.println(yaw);
    
    for (int i = 0; i < 3; i++) {
        lastAccel[i] = accel[i];
    }
}

void testAccelerometerStability() {
    float accelReadings[3][15];
    float accelSum[3] = {0, 0, 0};
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    
    for (int i = 0; i < 15; i++) {
        readRawData(rawAccel, rawGyro);
        
        accelReadings[0][i] = rawAccel[0] / 8192.0f - calData.accelBias[0];
        accelReadings[1][i] = rawAccel[1] / 8192.0f - calData.accelBias[1];
        accelReadings[2][i] = (rawAccel[2] / 8192.0f) - calData.accelBias[2] - 1.0f;
        
        accelSum[0] += accelReadings[0][i];
        accelSum[1] += accelReadings[1][i];
        accelSum[2] += accelReadings[2][i];
        
        delay(1000 / SAMPLE_RATE);
    }
    
    float accelAvg[3] = {
        accelSum[0] / 15,
        accelSum[1] / 15,
        accelSum[2] / 15
    };
    
    float accelVariance[3] = {0, 0, 0};
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 3; j++) {
            accelVariance[j] += pow(accelReadings[j][i] - accelAvg[j], 2);
        }
    }
    for (int j = 0; j < 3; j++) {
        accelVariance[j] /= 15;
    }
    
    SERIAL_PORT.println("Accelerometer Stats:");
    for (int i = 0; i < 3; i++) {
        SERIAL_PORT.print("Axis "); SERIAL_PORT.print(i); SERIAL_PORT.println(":");
        SERIAL_PORT.print("Avg: "); SERIAL_PORT.println(accelAvg[i]);
        SERIAL_PORT.print("StdDev: "); SERIAL_PORT.println(sqrt(accelVariance[i]));
    }
}

void printResults() {
    SERIAL_PORT.println("\n=== IMU Results ===");
    SERIAL_PORT.println("Calibration Status:");
    SERIAL_PORT.print("Accel: "); SERIAL_PORT.println(isCalibrated ? "Calibrated" : "Not Calibrated");
    
    SERIAL_PORT.println("\nError Metrics (RMS):");
    SERIAL_PORT.println("Accelerometer (mg):");
    for (int i = 0; i < 3; i++) {
        SERIAL_PORT.print("Axis "); SERIAL_PORT.print(i); SERIAL_PORT.print(": ");
        SERIAL_PORT.println(errorMetrics.accelRMS[i]);
    }
}

void printCalibrationStatus() {
    SERIAL_PORT.println("\n=== IMU Status ===");
    SERIAL_PORT.print("Calibration: ");
    SERIAL_PORT.println(isCalibrated ? "Calibrated" : "Not Calibrated");
    
    int16_t rawAccel[3];
    int16_t rawGyro[3];
    readRawData(rawAccel, rawGyro);
    
    float accel[3] = {
        rawAccel[0] / 8192.0f - calData.accelBias[0],
        rawAccel[1] / 8192.0f - calData.accelBias[1],
        (rawAccel[2] / 8192.0f) - calData.accelBias[2] - 1.0f
    };
    
    float pitch, roll, yaw;
    calculateAngles(accel, &pitch, &roll, &yaw);
    
    SERIAL_PORT.print("Acc (mg) [ ");
    SERIAL_PORT.print(accel[0]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(accel[1]);
    SERIAL_PORT.print(", ");
    SERIAL_PORT.print(accel[2]);
    SERIAL_PORT.println(" ]");
    
    SERIAL_PORT.print("Angles (deg) - Pitch: ");
    SERIAL_PORT.print(pitch);
    SERIAL_PORT.print(", Roll: ");
    SERIAL_PORT.print(roll);
    SERIAL_PORT.print(", Yaw: ");
    SERIAL_PORT.println(yaw);
}

void printHelp() {
    SERIAL_PORT.println("\nCommands:");
    SERIAL_PORT.println("c - Check calibration");
    SERIAL_PORT.println("v - Validate calibration");
    SERIAL_PORT.println("r - Reset calibration");
    SERIAL_PORT.println("a - Run accelerometer test");
    SERIAL_PORT.println("h - Print help");
}

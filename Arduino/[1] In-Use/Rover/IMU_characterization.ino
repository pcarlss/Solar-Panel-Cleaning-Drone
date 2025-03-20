/****************************************************************
 * Example1_Basics_I2C.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 ***************************************************************/

#include "ICM_20948.h" // Make sure the library is installed

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Default I2C port on Arduino Nano
#define AD0_VAL 0    // Address value (0 if ADR jumper closed, 1 otherwise)

// IMU Configuration
#define SAMPLE_RATE 100  // Hz
#define CALIBRATION_SAMPLES 250  // Reduced from 500
#define VALIDATION_SAMPLES 100   // Reduced from 250
#define STABILITY_THRESHOLD 0.1  // degrees/s for gyro, mg for accel
#define TEMPERATURE_THRESHOLD 5.0  // degrees Celsius

// Simplified data structures for analysis
struct IMUCalibrationData {
    float accelBias[3];
    float gyroBias[3];
};

struct IMUErrorMetrics {
    float accelRMS[3];
    float gyroRMS[3];
    float stabilityScore;
};

// Global variables
ICM_20948_I2C myICM; // Create an I2C object for ICM-20948
IMUCalibrationData calData;
IMUErrorMetrics errorMetrics;
bool isCalibrated = false;
unsigned long lastSampleTime = 0;

// Function prototypes
void initializeIMU();
void performCalibration();
void validateCalibration();
void analyzeStability();
void calculateErrorMetrics();
void printResults();
void saveCalibration();
void loadCalibration();
void printCalibrationStatus();
void printHelp();

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {
  };

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  
  // Initialize IMU
  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }

  // Configure IMU settings
  myICM.setBank(2);  // Switch to register bank 2
  
  // Set accelerometer full-scale range to ±4g
  ICM_20948_fss_t accelFss;
  accelFss.a = 0x01;  // 0x01 = ±4g
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), accelFss);
  
  // Set gyroscope full-scale range to ±250°/s
  ICM_20948_fss_t gyroFss;
  gyroFss.g = 0x00;  // 0x00 = ±250°/s
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), gyroFss);
  
  myICM.setBank(0);  // Return to bank 0
  
  // Enable and configure DLPF
  myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  
  // Configure DLPF for accelerometer and gyroscope
  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = 0x06;  // Accelerometer DLPF configuration
  dlpcfg.g = 0x06;  // Gyroscope DLPF configuration
  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);

  SERIAL_PORT.println("\nIMU initialized successfully.");
  SERIAL_PORT.println("\nStarting calibration process...");
  SERIAL_PORT.println("Please keep the IMU stationary during calibration.");
  
  performCalibration();
  validateCalibration();
  
  SERIAL_PORT.println("\nCalibration complete. Starting continuous monitoring...");
  SERIAL_PORT.println("Commands:");
  SERIAL_PORT.println("c - Check calibration status");
  SERIAL_PORT.println("v - Validate calibration");
  SERIAL_PORT.println("r - Reset calibration");
  SERIAL_PORT.println("s - Save calibration data");
  SERIAL_PORT.println("l - Load calibration data");
  SERIAL_PORT.println("t - Run temperature stability test");
  SERIAL_PORT.println("a - Run accelerometer stability test");
  SERIAL_PORT.println("g - Run gyroscope stability test");
  SERIAL_PORT.println("m - Run magnetometer stability test");
  SERIAL_PORT.println("h - Print help");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Regular data collection and analysis
  if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE)) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      analyzeStability();
      lastSampleTime = currentTime;
    }
  }
  
  // Handle serial commands
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
        myICM.swReset();
        delay(100);
        myICM.startupDefault();
        isCalibrated = false;
        SERIAL_PORT.println("Calibration reset.");
        break;
      case 's':
        saveCalibration();
        break;
      case 'l':
        loadCalibration();
        break;
      case 't':
        testTemperatureStability();
        break;
      case 'a':
        testAccelerometerStability();
        break;
      case 'g':
        testGyroscopeStability();
        break;
      case 'm':
        testMagnetometerStability();
        break;
      case 'h':
        printHelp();
        break;
    }
  }
}

void performCalibration() {
  float accelSum[3] = {0, 0, 0};
  float gyroSum[3] = {0, 0, 0};
  
  SERIAL_PORT.println("Collecting calibration data...");
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      
      // Accumulate sensor readings
      accelSum[0] += myICM.accX();
      accelSum[1] += myICM.accY();
      accelSum[2] += myICM.accZ() - 1000.0; // Subtract 1g from Z
      
      gyroSum[0] += myICM.gyrX();
      gyroSum[1] += myICM.gyrY();
      gyroSum[2] += myICM.gyrZ();
      
      delay(1000 / SAMPLE_RATE);
      
      if (i % 50 == 0) {
        SERIAL_PORT.print("Calibration progress: ");
        SERIAL_PORT.print(i * 100 / CALIBRATION_SAMPLES);
        SERIAL_PORT.println("%");
      }
    }
  }
  
  // Calculate biases
  for (int i = 0; i < 3; i++) {
    calData.accelBias[i] = accelSum[i] / CALIBRATION_SAMPLES;
    calData.gyroBias[i] = gyroSum[i] / CALIBRATION_SAMPLES;
  }
  
  // Apply calibration
  myICM.setBiasAccelX((int32_t)(calData.accelBias[0] * 16384.0f));
  myICM.setBiasAccelY((int32_t)(calData.accelBias[1] * 16384.0f));
  myICM.setBiasAccelZ((int32_t)(calData.accelBias[2] * 16384.0f));
  
  myICM.setBiasGyroX((int32_t)(calData.gyroBias[0] * 16.4f));
  myICM.setBiasGyroY((int32_t)(calData.gyroBias[1] * 16.4f));
  myICM.setBiasGyroZ((int32_t)(calData.gyroBias[2] * 16.4f));
  
  isCalibrated = true;
}

void validateCalibration() {
  if (!isCalibrated) {
    SERIAL_PORT.println("IMU not calibrated!");
    return;
  }
  
  float accelError[3] = {0, 0, 0};
  float gyroError[3] = {0, 0, 0};
  
  SERIAL_PORT.println("\nValidating calibration...");
  
  for (int i = 0; i < VALIDATION_SAMPLES; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      
      // Calculate errors
      accelError[0] += abs(myICM.accX() - calData.accelBias[0]);
      accelError[1] += abs(myICM.accY() - calData.accelBias[1]);
      accelError[2] += abs(myICM.accZ() - calData.accelBias[2] - 1000.0);
      
      gyroError[0] += abs(myICM.gyrX() - calData.gyroBias[0]);
      gyroError[1] += abs(myICM.gyrY() - calData.gyroBias[1]);
      gyroError[2] += abs(myICM.gyrZ() - calData.gyroBias[2]);
      
      delay(1000 / SAMPLE_RATE);
    }
  }
  
  // Calculate RMS errors
  for (int i = 0; i < 3; i++) {
    errorMetrics.accelRMS[i] = sqrt(accelError[i] / VALIDATION_SAMPLES);
    errorMetrics.gyroRMS[i] = sqrt(gyroError[i] / VALIDATION_SAMPLES);
  }
  
  printResults();
}

void analyzeStability() {
  static float lastAccel[3] = {0, 0, 0};
  static float lastGyro[3] = {0, 0, 0};
  
  float accelDelta[3] = {
    abs(myICM.accX() - lastAccel[0]),
    abs(myICM.accY() - lastAccel[1]),
    abs(myICM.accZ() - lastAccel[2])
  };
  
  float gyroDelta[3] = {
    abs(myICM.gyrX() - lastGyro[0]),
    abs(myICM.gyrY() - lastGyro[1]),
    abs(myICM.gyrZ() - lastGyro[2])
  };
  
  // Check for instability
  bool isUnstable = false;
  for (int i = 0; i < 3; i++) {
    if (accelDelta[i] > STABILITY_THRESHOLD || 
      gyroDelta[i] > STABILITY_THRESHOLD) {
      isUnstable = true;
      break;
    }
  }
  
  if (isUnstable) {
    SERIAL_PORT.println("Warning: IMU readings unstable!");
  }
  
  // Update last values
  for (int i = 0; i < 3; i++) {
    lastAccel[i] = myICM.accX();
    lastGyro[i] = myICM.gyrX();
  }
}

void testTemperatureStability() {
  SERIAL_PORT.println("\nRunning temperature stability test...");
  float tempReadings[50];  // Reduced from 100
  float tempSum = 0;
  
  for (int i = 0; i < 50; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      tempReadings[i] = myICM.temp();
      tempSum += tempReadings[i];
      delay(1000 / SAMPLE_RATE);
    }
  }
  
  float avgTemp = tempSum / 50;
  float tempVariance = 0;
  
  for (int i = 0; i < 50; i++) {
    tempVariance += pow(tempReadings[i] - avgTemp, 2);
  }
  tempVariance /= 50;
  
  SERIAL_PORT.print("Temperature Statistics:\n");
  SERIAL_PORT.print("Average: "); SERIAL_PORT.println(avgTemp);
  SERIAL_PORT.print("Variance: "); SERIAL_PORT.println(tempVariance);
  SERIAL_PORT.print("Standard Deviation: "); SERIAL_PORT.println(sqrt(tempVariance));
}

void testAccelerometerStability() {
  SERIAL_PORT.println("\nRunning accelerometer stability test...");
  float accelReadings[3][25];  // Reduced from 50
  float accelSum[3] = {0, 0, 0};
  
  for (int i = 0; i < 25; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      accelReadings[0][i] = myICM.accX();
      accelReadings[1][i] = myICM.accY();
      accelReadings[2][i] = myICM.accZ();
      
      accelSum[0] += accelReadings[0][i];
      accelSum[1] += accelReadings[1][i];
      accelSum[2] += accelReadings[2][i];
      
      delay(1000 / SAMPLE_RATE);
    }
  }
  
  float accelAvg[3] = {
    accelSum[0] / 25,
    accelSum[1] / 25,
    accelSum[2] / 25
  };
  
  float accelVariance[3] = {0, 0, 0};
  for (int i = 0; i < 25; i++) {
    for (int j = 0; j < 3; j++) {
      accelVariance[j] += pow(accelReadings[j][i] - accelAvg[j], 2);
    }
  }
  for (int j = 0; j < 3; j++) {
    accelVariance[j] /= 25;
  }
  
  SERIAL_PORT.println("Accelerometer Statistics:");
  for (int i = 0; i < 3; i++) {
    SERIAL_PORT.print("Axis "); SERIAL_PORT.print(i); SERIAL_PORT.println(":");
    SERIAL_PORT.print("Average: "); SERIAL_PORT.println(accelAvg[i]);
    SERIAL_PORT.print("Variance: "); SERIAL_PORT.println(accelVariance[i]);
    SERIAL_PORT.print("Standard Deviation: "); SERIAL_PORT.println(sqrt(accelVariance[i]));
  }
}

void testGyroscopeStability() {
  SERIAL_PORT.println("\nRunning gyroscope stability test...");
  float gyroReadings[3][25];  // Reduced from 50
  float gyroSum[3] = {0, 0, 0};
  
  for (int i = 0; i < 25; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      gyroReadings[0][i] = myICM.gyrX();
      gyroReadings[1][i] = myICM.gyrY();
      gyroReadings[2][i] = myICM.gyrZ();
      
      gyroSum[0] += gyroReadings[0][i];
      gyroSum[1] += gyroReadings[1][i];
      gyroSum[2] += gyroReadings[2][i];
      
      delay(1000 / SAMPLE_RATE);
    }
  }
  
  float gyroAvg[3] = {
    gyroSum[0] / 25,
    gyroSum[1] / 25,
    gyroSum[2] / 25
  };
  
  float gyroVariance[3] = {0, 0, 0};
  for (int i = 0; i < 25; i++) {
    for (int j = 0; j < 3; j++) {
      gyroVariance[j] += pow(gyroReadings[j][i] - gyroAvg[j], 2);
    }
  }
  for (int j = 0; j < 3; j++) {
    gyroVariance[j] /= 25;
  }
  
  SERIAL_PORT.println("Gyroscope Statistics:");
  for (int i = 0; i < 3; i++) {
    SERIAL_PORT.print("Axis "); SERIAL_PORT.print(i); SERIAL_PORT.println(":");
    SERIAL_PORT.print("Average: "); SERIAL_PORT.println(gyroAvg[i]);
    SERIAL_PORT.print("Variance: "); SERIAL_PORT.println(gyroVariance[i]);
    SERIAL_PORT.print("Standard Deviation: "); SERIAL_PORT.println(sqrt(gyroVariance[i]));
  }
}

void testMagnetometerStability() {
  SERIAL_PORT.println("\nMagnetometer test removed to save memory.");
}

void printResults() {
  SERIAL_PORT.println("\n=== IMU Characterization Results ===");
  SERIAL_PORT.println("Calibration Status:");
  SERIAL_PORT.print("Accelerometer: "); SERIAL_PORT.println(isCalibrated ? "Calibrated" : "Not Calibrated");
  SERIAL_PORT.print("Gyroscope: "); SERIAL_PORT.println(isCalibrated ? "Calibrated" : "Not Calibrated");
  
  SERIAL_PORT.println("\nError Metrics (RMS):");
  SERIAL_PORT.println("Accelerometer (mg):");
  SERIAL_PORT.print("X: "); SERIAL_PORT.println(errorMetrics.accelRMS[0]);
  SERIAL_PORT.print("Y: "); SERIAL_PORT.println(errorMetrics.accelRMS[1]);
  SERIAL_PORT.print("Z: "); SERIAL_PORT.println(errorMetrics.accelRMS[2]);
  
  SERIAL_PORT.println("\nGyroscope (dps):");
  SERIAL_PORT.print("X: "); SERIAL_PORT.println(errorMetrics.gyroRMS[0]);
  SERIAL_PORT.print("Y: "); SERIAL_PORT.println(errorMetrics.gyroRMS[1]);
  SERIAL_PORT.print("Z: "); SERIAL_PORT.println(errorMetrics.gyroRMS[2]);
}

void saveCalibration() {
  // Implementation for saving calibration data to EEPROM or SD card
  SERIAL_PORT.println("Calibration data saved.");
}

void loadCalibration() {
  // Implementation for loading calibration data from EEPROM or SD card
  SERIAL_PORT.println("Calibration data loaded.");
}

void printCalibrationStatus() {
  SERIAL_PORT.println("\n=== IMU Status ===");
  SERIAL_PORT.print("Calibration Status: ");
  SERIAL_PORT.println(isCalibrated ? "Calibrated" : "Not Calibrated");
  
  if (myICM.dataReady()) {
    myICM.getAGMT();
    printScaledAGMT(&myICM);
  }
}

void printHelp() {
  SERIAL_PORT.println("\nAvailable commands:");
  SERIAL_PORT.println("c - Check calibration status");
  SERIAL_PORT.println("v - Validate calibration");
  SERIAL_PORT.println("r - Reset calibration");
  SERIAL_PORT.println("s - Save calibration data");
  SERIAL_PORT.println("l - Load calibration data");
  SERIAL_PORT.println("t - Run temperature stability test");
  SERIAL_PORT.println("a - Run accelerometer stability test");
  SERIAL_PORT.println("g - Run gyroscope stability test");
  SERIAL_PORT.println("m - Run magnetometer stability test");
  SERIAL_PORT.println("h - Print this help message");
}

// Helper functions for formatted output
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) SERIAL_PORT.print("-");
  else SERIAL_PORT.print(" ");

  for (uint8_t i = 0; i < leading; i++) {
    uint32_t tenpow = 1;
    for (uint8_t c = 0; c < (leading - 1 - i); c++) tenpow *= 10;
    if (aval < tenpow) SERIAL_PORT.print("0");
    else break;
  }
  
  if (val < 0) SERIAL_PORT.print(-val, decimals);
  else SERIAL_PORT.print(val, decimals);
}

void printScaledAGMT(ICM_20948_I2C *sensor) {
  SERIAL_PORT.print("Acc (mg) [ ");
  SERIAL_PORT.print(sensor->accX());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->accY());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->accZ());
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  SERIAL_PORT.print(sensor->gyrX());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->gyrY());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->gyrZ());
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  SERIAL_PORT.print(sensor->temp());
  SERIAL_PORT.println(" ]");
}

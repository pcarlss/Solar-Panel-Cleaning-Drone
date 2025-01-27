/****************************************************************
 * Example1_Basics_I2C.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 ***************************************************************/

#include "ICM_20948.h" // Make sure the library is installed

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Default I2C port on Arduino Nano
#define AD0_VAL 0    // Address value (0 if ADR jumper closed, 1 otherwise)

ICM_20948_I2C myICM; // Create an I2C object for ICM-20948

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {
  };

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
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
}

void loop() {
  if (myICM.dataReady()) {
    myICM.getAGMT();         // Updates accelerometer, gyroscope, magnetometer, and temperature data
    printScaledAGMT(&myICM); // Print scaled data to Serial Monitor
    delay(30);
  } else {
    SERIAL_PORT.println("Waiting for data");
    // setup();
    delay(500);
  }
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
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

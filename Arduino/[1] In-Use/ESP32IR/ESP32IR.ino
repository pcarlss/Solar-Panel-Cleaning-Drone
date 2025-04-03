#include <cstdlib>  // Include for qsort()

// Define the GPIOs for 8 safe analog-capable pins
const int sensorPins[8] = {32, 33, 25, 26, 27, 34, 35, 36};
int sensorValues[8];
int referenceValues[8];  // Reference values for each sensor

// Tolerance percentage (15%)
const float tolerance = 0.15;  // This is the 15% range for comparison

// Time intervals for recalculating reference values
const unsigned long referenceUpdateInterval = 5000;  // 5 seconds
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);                      // Debug via USB
  Serial1.begin(38400, SERIAL_8N1, -1, 17);   // TX on GPIO17, RX unused

  analogReadResolution(12);  // Set ADC resolution

  // Set pinMode only for GPIOs that allow it (exclude input-only pins)
  for (int i = 0; i < 8; i++) {
    if (sensorPins[i] <= 27) {
      pinMode(sensorPins[i], INPUT);
    }
  }

  // Immediately calculate the initial reference values (median of the first readings for each sensor)
  calculateReferenceValues();
}

void loop() {
  // Check if it's time to recalculate the reference values
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= referenceUpdateInterval) {
    previousMillis = currentMillis;
    calculateReferenceValues();  // Recalculate reference values every 5 seconds
  }

  String data = "";

  // Read sensor values and compare with reference values
  Serial.print("Sensor values: ");  // Start printing sensor values on the same line

  for (int i = 0; i < 8; i++) {
    int pin = sensorPins[i];
    int value = analogRead(pin);

    // Apply threshold: values below 500 become 0
    if (value < 500) {
      value = 0;
    }

    sensorValues[i] = value;

    // Print the current value of the sensor on the same line
    Serial.print(value);
    if (i < 7) {
      Serial.print(", ");  // Separate the values with a comma
    }

    // Calculate the lower limit (85% of the reference value)
    int lowerLimit = referenceValues[i] - referenceValues[i] * tolerance;

    // Check if the value is greater than or equal to 85% of the reference value
    if (value >= lowerLimit) {
      data += "1";  // In range (>= 85% of the reference)
    } else {
      data += "0";  // Out of range (< 85% of the reference)
    }

    //if (i < 7) data += ",";  // Add commas between the values
  }

  // Send the 0's and 1's string to Arduino via Serial1
  Serial1.println(data);
  delay(20);  // Adjust as needed
}

// Comparison function for qsort (required for sorting sensor readings)
int compare(const void *a, const void *b) {
  int valA = *((int *)a);
  int valB = *((int *)b);
  
  if (valA < valB) return -1;
  if (valA > valB) return 1;
  return 0;
}

// Function to calculate reference values using median (to avoid extreme cases)
void calculateReferenceValues() {
  for (int i = 0; i < 8; i++) {
    int readings[10];
    bool updateReference = true;  // Flag to check if the reference value should be updated

    // Take 10 readings to calculate the reference for each sensor
    for (int j = 0; j < 10; j++) {
      readings[j] = analogRead(sensorPins[i]);
      
      // If any reading is 500 or less, do not update the reference value
      if (readings[j] <= 500) {
        updateReference = false;  // Flag that prevents updating the reference value
      }
    }

    // Only update the reference values if no reading is 500 or less
    if (updateReference) {
      // Sort the readings to calculate the median using qsort
      qsort(readings, 10, sizeof(int), compare);  // Use qsort for sorting

      // Median is the middle element of the sorted array
      referenceValues[i] = readings[5];  // 5th index is the middle element in a sorted array of size 10
    } else {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(": Not updating reference value due to readings below 500");
    }
  }

  // Debugging: Print the reference values to the Serial Monitor
  Serial.print("New reference values (median): ");
  for (int i = 0; i < 8; i++) {
    Serial.print(referenceValues[i]);
    if (i < 7) {
      Serial.print(", ");
    }
  }
  Serial.println();
}

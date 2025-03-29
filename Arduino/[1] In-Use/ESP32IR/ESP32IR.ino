// Define the GPIOs for 8 safe analog-capable pins
const int sensorPins[8] = {32, 33, 25, 26, 27, 34, 35, 36};
int sensorValues[8];

void setup() {
  Serial.begin(115200);                      // Debug via USB
  Serial1.begin(9600, SERIAL_8N1, -1, 17);   // TX on GPIO17, RX unused

  analogReadResolution(12);  // Set ADC resolution

  // Set pinMode only for GPIOs that allow it (exclude input-only pins)
  for (int i = 0; i < 8; i++) {
    if (sensorPins[i] <= 27) {
      pinMode(sensorPins[i], INPUT);
    }
  }
}

void loop() {
  String data = "";

  for (int i = 0; i < 8; i++) {
    int pin = sensorPins[i];
    int value = analogRead(pin);

    // Apply threshold: values below 500 become 0
    if (value < 500) {
      value = 0;
    }

    sensorValues[i] = value;
    data += String(value);
    if (i < 7) data += ",";
  }

  // Send all 8 values as a comma-separated string to Arduino via Serial1
  Serial1.println(data);
  delay(15);        // Adjust as needed
}

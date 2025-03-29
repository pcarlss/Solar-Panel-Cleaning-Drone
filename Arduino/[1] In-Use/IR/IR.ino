#include <SoftwareSerial.h>

// Use A0 (digital pin 14) as RX, and TX is unused (set to unused pin or 255)
SoftwareSerial espSerial(A0, 255);  // RX = A0, TX unused

void setup() {
  Serial.begin(115200);       // Serial monitor on PC
  espSerial.begin(9600);      // Communication from ESP32
}

void loop() {
  if (espSerial.available()) {
    String line = espSerial.readStringUntil('\n');
    Serial.println(line);  // Just print the raw data, no label
  }
}

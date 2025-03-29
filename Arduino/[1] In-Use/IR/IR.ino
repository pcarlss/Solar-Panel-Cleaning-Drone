#include <SoftwareSerial.h>

SoftwareSerial espSerial(8, 255);  // RX = D8, TX unused

int sensorValues[8];

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
}

void loop() {
  if (espSerial.available()) {
    String line = espSerial.readStringUntil('\n');
    Serial.println("Received: " + line);

    // Parse comma-separated values
    int index = 0;
    char *token = strtok((char*)line.c_str(), ",");

    while (token != NULL && index < 8) {
      sensorValues[index++] = atoi(token);
      token = strtok(NULL, ",");
    }

    // sensorValues[] now holds your 8 readings
  }
}

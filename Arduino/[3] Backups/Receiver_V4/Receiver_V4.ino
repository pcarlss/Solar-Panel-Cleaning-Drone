#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9    // CE pin for RF24
#define CSN_PIN 10  // CSN pin for RF24

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

struct ControllerData {
  int throttle;
  int yaw;
  int pitch;
  int roll;
  int angle;
  int safety;
  bool AUX;
} receivedData;

void setup() {
  Serial.begin(115200);
  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1)
      ;  // Stop the program if the radio is not detected
  }

  radio.openReadingPipe(0, address);  // Open the reading pipe with the same address as the transmitter
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(sizeof(ControllerData));
  radio.setChannel(100);
  radio.startListening();  // Start listening for incoming data

  Serial.println("Receiver ready, waiting for data...");
}

void loop() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));  // Read the data into the receivedData structure

    // // Print the received values
    // Serial.print("Throttle: ");
    // Serial.print(receivedData.throttle);
    // Serial.print(", Yaw: ");
    // Serial.print(receivedData.yaw);
    // Serial.print(", Pitch: ");
    // Serial.print(receivedData.pitch);
    // Serial.print(", Roll: ");
    // Serial.print(receivedData.roll);
    // Serial.print(", Safety: ");
    // Serial.print(receivedData.safety);
    // Serial.print(", Angle: ");
    // Serial.println(receivedData.angle);
    delay(1);
  } else {
    delay(1);
  }
}

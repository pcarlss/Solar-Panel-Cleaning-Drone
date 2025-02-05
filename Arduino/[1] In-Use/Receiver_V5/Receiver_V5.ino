#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

#define CE_PIN 9   // CE pin 
#define CSN_PIN 10 // CSN pin

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;

int AUX1 = 992;
int AUX2 = 992;

struct ControllerData {
  int roll;
  int pitch;
  int throttle;
  int yaw;
  int angle;
  int safety;
  bool AUX;
} receivedData;

void ResetData() 
{
  receivedData.roll = 1500;
  receivedData.pitch = 1500;
  receivedData.throttle = 1000; 
  receivedData.yaw = 1500;                
  receivedData.angle = 90;
  receivedData.safety = 0;
  receivedData.AUX = false;                       
}

void setup() {
  Serial.begin(115200);

  ch1.attach(2);
  ch2.attach(3);
  ch3.attach(4);
  ch4.attach(5);
  ch5.attach(6);
  ch6.attach(7);

  ResetData();

  if (!radio.begin()) {
    Serial.println("Radio hardware not responding!");
    while (1);
  }

  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setPayloadSize(sizeof(ControllerData));
  radio.setChannel(100);
  radio.startListening();

  Serial.println("Receiver ready, waiting for data...");
}

void loop() {
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));

    if (receivedData.AUX == false) AUX1 = 1792;
    else AUX1 = 992;

    ch1.writeMicroseconds(receivedData.roll-7);
    ch2.writeMicroseconds(receivedData.pitch-7);
    ch3.writeMicroseconds(receivedData.throttle-5);
    ch4.writeMicroseconds(receivedData.yaw-7);
    ch5.writeMicroseconds(AUX1);
    ch6.writeMicroseconds(AUX2); 

    delay(1);
  } else {
    delay(1);
  }
}

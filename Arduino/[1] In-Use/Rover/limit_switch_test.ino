#include <SoftwareSerial.h>

SoftwareSerial espSerial(A0, 255);  // RX = A0, TX unused
bool LFO, RFI, RFO, LBO, LFI, RBO, LBI, RBI;

#define CLK1 2  // Encoder 1 Clock pin (Interrupt)
#define DT1 A2  // Encoder 1 Data pin
#define CLK2 3  // Encoder 2 Clock pin (Interrupt)
#define DT2 A3  // Encoder 2 Data pin
#define IN1 4   // Motor 1 Direction 1
#define IN2 A1  // Motor 1 Direction 2
#define IN3 8   // Motor 2 Direction 1
#define IN4 7   // Motor 2 Direction 2
#define EN_A 5  // Enable pin for Motor 1 (PWM)
#define EN_B 6  // Enable pin for Motor 2 (PWM)

int speedA = 150;  // Speed for Motor 1 (0-255)
int speedB = 150;  // Speed for Motor 2 (0-255)
volatile int encoderValue1 = 0;
volatile int encoderValue2 = 0;
int lastStateCLK1;
int lastStateCLK2;

void setup() {
  Serial.begin(115200);    // Serial monitor on PC
  espSerial.begin(38400);  // Communication from ESP32

  // Encoder 1 setup
  pinMode(CLK1, INPUT);
  pinMode(DT1, INPUT);
  lastStateCLK1 = digitalRead(CLK1);
  attachInterrupt(digitalPinToInterrupt(CLK1), readEncoder1, CHANGE);

  // Encoder 2 setup
  pinMode(CLK2, INPUT);
  pinMode(DT2, INPUT);
  lastStateCLK2 = digitalRead(CLK2);
  attachInterrupt(digitalPinToInterrupt(CLK2), readEncoder2, CHANGE);

  // Motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Enable pins
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);

  // Set initial speed using PWM
  analogWrite(EN_A, speedA);
  analogWrite(EN_B, speedB);
}

void loop() {
  if (espSerial.available()) {
    String line = espSerial.readStringUntil('\n');
    //Serial.println(line);

      LFO = (int) line[0] - '0' == 1 ? true : false;  // Convert char to int
      RFI = (int) line[1] - '0' == 1 ? true : false;
      RFO = (int) line[2] - '0' == 1 ? true : false;
      LBO = (int) line[3] - '0' == 1 ? true : false;
      LFI = (int) line[4] - '0' == 1 ? true : false;
      RBO = (int) line[5] - '0' == 1 ? true : false;
      LBI = (int) line[6] - '0' == 1 ? true : false;
      RBI = (int) line[7] - '0' == 1 ? true : false;

      // Print sensor values
      Serial.print("LFO: ");
      Serial.print(LFO);
      Serial.print(" RFI: ");
      Serial.print(RFI);
      Serial.print(" RFO: ");
      Serial.print(RFO);
      Serial.print(" LBO: ");
      Serial.print(LBO);
      Serial.print(" LFI: ");
      Serial.print(LFI);
      Serial.print(" RBO: ");
      Serial.print(RBO);
      Serial.print(" LBI: ");
      Serial.print(LBI);
      Serial.print(" RBI: ");
      Serial.println(RBI);
  }
}

void printEncoders() {
  Serial.print("Encoder 1: ");
  Serial.print(encoderValue1);
  Serial.print(" | Encoder 2: ");
  Serial.println(encoderValue2);
}

void moveForward() {
  // Motor 1 Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor 2 Forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveReverse() {
  // Motor 1 Reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  Serial.println("Motor 1: REVERSE");

  // Motor 2 Reverse
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Motor 2: REVERSE");
}

void stop() {
  // Stop Motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Interrupt Service Routine (ISR) for Encoder 1
void readEncoder1() {
  int stateCLK1 = digitalRead(CLK1);
  if (digitalRead(DT1) != stateCLK1) {
    encoderValue1++;
  } else {
    encoderValue1--;
  }
}

// Interrupt Service Routine (ISR) for Encoder 2
void readEncoder2() {
  int stateCLK2 = digitalRead(CLK2);
  if (digitalRead(DT2) != stateCLK2) {
    encoderValue2++;
  } else {
    encoderValue2--;
  }
}
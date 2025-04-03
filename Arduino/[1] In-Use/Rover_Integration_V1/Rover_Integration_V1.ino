#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial espSerial(A0, 255);  // RX = A0, TX unused

// MPU6050 variables
const int MPU = 0x68;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;
float GyroErrorX, GyroErrorY, GyroErrorZ;
int c = 0;

// Sensors
bool LFO, RFI, RFO, LBO, LFI, RBO, LBI, RBI;

// Motor and encoder pins
#define CLK1 2
#define DT1 A2
#define CLK2 3
#define DT2 A3
#define IN1 4
#define IN2 A1
#define IN3 8
#define IN4 7
#define EN_A 5
#define EN_B 6

int baseSpeedA = 200;
int baseSpeedB = 240;
volatile int encoderValue1 = 0;
volatile int encoderValue2 = 0;
int lastStateCLK1, lastStateCLK2;

// PID control variables
float yawTarget = 0.0;
float Kp = 3.0;

void setup() {
  Serial.begin(115200);
  espSerial.begin(38400);

  setupMPU();
  calculate_IMU_error();

  pinMode(CLK1, INPUT); pinMode(DT1, INPUT);
  pinMode(CLK2, INPUT); pinMode(DT2, INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK1), readEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CLK2), readEncoder2, CHANGE);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(EN_A, OUTPUT); pinMode(EN_B, OUTPUT);
}

void loop() {
  updateIMU();

  if (espSerial.available()) {
    String line = espSerial.readStringUntil('\n');

    LFI = line[0] - '0';
    RFI = line[1] - '0';
    RFO = line[2] - '0';
    LBO = line[3] - '0';
    LFO = line[4] - '0';
    RBO = line[5] - '0';
    LBI = line[6] - '0';
    RBI = line[7] - '0';

    Serial.println(String(LFI) + String(RFI) + String(RFO) + String(LBO) + String(LFO) + String(RBO) + String(LBI) + String(RBI));

    // 🚧 Obstacle ahead: reverse and turn
    if (LFO == 0 && RFO == 0) {
      Serial.println("OBSTACLE AHEAD - BACKING UP");

      stop();
      delay(100);

      moveReverse();
      delay(1000);
      stop();
      delay(100);

      Serial.println("TURNING RIGHT IN PLACE");
      rotateRight();
      delay(1500);
      stop();
      delay(100);

      yawTarget = yaw;  // update yaw baseline
      Serial.println("RESUMING FORWARD");
      moveForwardWithYawControl();
    }

    // 🚗 Edge following (left side)
    else if (LFI == 1 && LBI == 1 && LFO == 0 && LBO == 0) {
      yawTarget = yaw;
      moveForwardWithYawControl();
      Serial.println("STRAIGHT");
    }

    // ↩️ Steer Left
    else if ((LFO == 1 && LFI == 1 && LBO == 1 && LBI == 1) || (LFI == 1 && (LFO == 0 || LFO == 1) && LBI == 0 && LBO == 0)) {
      steerLeft();
      Serial.println("STEER LEFT");
    }

    // ↪️ Steer Right
    else if (LFO == 0 || LBO == 1) {
      steerRight();
      Serial.println("STEER RIGHT");
    }

    // ➡️ Default Forward
    else {
      yawTarget = yaw;
      moveForwardWithYawControl();
      Serial.println("STRAIGHT (DEFAULT)");
    }
  }

  printEncoders();
}

// ========================== IMU FUNCTIONS ==========================

void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(true);
  Wire.beginTransmission(MPU); Wire.write(0x1B); Wire.write(0x10); Wire.endTransmission(true);
}

void updateIMU() {
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  Wire.beginTransmission(MPU); Wire.write(0x43); Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
}

void calculate_IMU_error() {
  while (c < 200) {
    Wire.beginTransmission(MPU); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    GyroErrorX += GyroX / 131.0;
    GyroErrorY += GyroY / 131.0;
    GyroErrorZ += GyroZ / 131.0;
    c++;
  }
  GyroErrorX /= 200; GyroErrorY /= 200; GyroErrorZ /= 200;
}

// ========================== MOTOR CONTROL ==========================

void moveForwardWithYawControl() {
  float error = yawTarget - yaw;
  int correction = (int)(Kp * error);

  int speedLeft = constrain(baseSpeedA + correction, 0, 255);
  int speedRight = constrain(baseSpeedB - correction, 0, 255);

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(EN_A, speedLeft);

  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN_B, speedRight);
}

void moveReverse() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(EN_A, baseSpeedA);
  analogWrite(EN_B, baseSpeedB);
}

void stop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(EN_A, 0);
  analogWrite(EN_B, 0);
}

void steerLeft() {
  int speedLeft = baseSpeedA;
  int speedRight = baseSpeedB * 0.6;

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(EN_A, speedLeft);

  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN_B, speedRight);
}

void steerRight() {
  int speedLeft = baseSpeedA * 0.6;
  int speedRight = baseSpeedB;

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(EN_A, speedLeft);

  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN_B, speedRight);
}

void rotateRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(EN_A, baseSpeedA);

  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(EN_B, baseSpeedB);
}

// ========================== ENCODER ISR ==========================

void readEncoder1() {
  int stateCLK1 = digitalRead(CLK1);
  if (digitalRead(DT1) != stateCLK1) encoderValue1++;
  else encoderValue1--;
}

void readEncoder2() {
  int stateCLK2 = digitalRead(CLK2);
  if (digitalRead(DT2) != stateCLK2) encoderValue2++;
  else encoderValue2--;
}

void printEncoders() {
  // Serial.print(" | Enc1: "); Serial.print(encoderValue1);
  // Serial.print(" | Enc2: "); Serial.println(encoderValue2);
}

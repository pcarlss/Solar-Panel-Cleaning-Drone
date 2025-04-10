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

#define CleaningEN 0

int baseSpeedA = 200;
int baseSpeedB = 240;
volatile int encoderValue1 = 0;
volatile int encoderValue2 = 0;
int lastStateCLK1, lastStateCLK2;

// PID control variables
float yawTarget = 0.0;
float Kp = 3.0;

// Perimeter tracking variables
enum PerimeterState {
  FOLLOW_EDGE,
  TURN_AT_CORNER,
  COMPLETE
};

PerimeterState currentState = FOLLOW_EDGE;
int cornerCount = 0;
const int MAX_CORNERS = 4;  // Assuming rectangular surface
float initialYaw = 0.0;
bool firstCornerFound = false;
unsigned long turnStartTime = 0;
const unsigned long TURN_DURATION = 8000;  // ms for 90-degree turn

void setup() {
  //Serial.begin(115200);
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
  pinMode(CleaningEN, OUTPUT);  // Initialize cleaning motor enable pin
  
  // Store initial yaw for perimeter completion detection
  delay(1000);  // Wait for IMU to stabilize
  updateIMU();
  initialYaw = yaw;
  
 
  Serial.println("Cleaning brushes activated");
  Serial.println("Perimeter tracking initialized");
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

    //Serial.println(String(LFI) + String(RFI) + String(RFO) + String(LBO) + String(LFO) + String(RBO) + String(LBI) + String(RBI));

    // State machine for perimeter tracking
    switch (currentState) {
      case FOLLOW_EDGE:
        followEdge();
         // Turn on cleaning brushes at the end of setup
        digitalWrite(CleaningEN, HIGH);
        break;
        
      case TURN_AT_CORNER:
        turnAtCorner();
         // Turn on cleaning brushes at the end of setup
        digitalWrite(CleaningEN, LOW);
        break;
        
      case COMPLETE:
        // Perimeter complete, stop motors and turn off cleaning brushes
        stop();
        digitalWrite(CleaningEN, LOW);
        Serial.println("PERIMETER COMPLETE! Cleaning brushes deactivated.");
        break;
    }
  }

  printEncoders();
}

void followEdge() {
  // Corner detection - when both left front switches go off
  if (RFO == 0 && RFI == 0) {
    Serial.println("CORNER DETECTED!");
    stop();
    delay(100);
    moveReverse();
    delay(500);
    
    // Check if we've completed a full circuit
    if (cornerCount >= MAX_CORNERS) {
      // Check if we're close to our starting orientation
      float yawDiff = abs(yaw - initialYaw);
      if (yawDiff < 0.5 || yawDiff > (2 * PI - 0.5)) {
        currentState = COMPLETE;
        return;
      }
    }
    
    // Start turning at corner
    currentState = TURN_AT_CORNER;
    turnStartTime = millis();
    cornerCount++;
    // Serial.print("Corner count: ");
    // Serial.println(cornerCount);
  }
  // 🚗 Edge following (left side)
  else if (LFI == 1 && LBI == 1 && LFO == 0 && LBO == 0) {
    yawTarget = yaw;
    moveForwardWithYawControl();
    //Serial.println("STRAIGHT");
  }
  // ↩️ Steer Left
  else if ((LFO == 1 && LFI == 1 && LBO == 1 && LBI == 1) || (LFI == 1 && (LFO == 0 || LFO == 1) && LBI == 0 && LBO == 0)) {
    steerLeft();
    //Serial.println("STEER LEFT");
  }
  // ↪️ Steer Right
  else if (LFO == 0 || LBO == 1) {
    steerRight();
    //Serial.println("STEER RIGHT");
  }
  // ➡️ Default Forward
  else {
    yawTarget = yaw;
    moveForwardWithYawControl();
    //Serial.println("STRAIGHT (DEFAULT)");
  }
}

float angleDifference(float target, float current) {
  float diff = target - current;
  if (diff > PI) diff -= 2 * PI;
  if (diff < -PI) diff += 2 * PI;
  return diff;
}


void turnAtCorner() {
  static bool turnInitialized = false;
  static float targetYaw = 0;

  if (!turnInitialized) {
    targetYaw = yaw + PI / 2.0;
    if (targetYaw > 2 * PI) targetYaw -= 2 * PI;
    turnInitialized = true;

    Serial.print("Target Yaw set to: ");
    Serial.println(targetYaw * 180.0 / PI, 1);
  }

  float yawDiff = abs(angleDifference(targetYaw, yaw));

  Serial.print("Current Yaw: ");
  Serial.print(yaw * 180.0 / PI, 1);
  Serial.print("° | Target: ");
  Serial.print(targetYaw * 180.0 / PI, 1);
  Serial.print("° | Diff: ");
  Serial.print(yawDiff * 180.0 / PI, 1);
  Serial.println("°");

  if (yawDiff > radians(4)) {
    rotateRightBurst();  // perform one burst cycle
  } else {
    stop();
    yawTarget = yaw;
    currentState = FOLLOW_EDGE;
    turnInitialized = false;
    Serial.println("TURN COMPLETE");
  }
}



// ========================== IMU FUNCTIONS ==========================

void setupMPU() {
  Wire.begin();
  Wire.beginTransmission(MPU); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(true);
  Wire.beginTransmission(MPU); Wire.write(0x1B); Wire.write(0x10); Wire.endTransmission(true);
}

void updateIMU() {
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;  // in seconds

  Wire.beginTransmission(MPU); Wire.write(0x43); Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;

  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  if (abs(GyroZ) < 0.5) GyroZ = 0.0;  // ignore small noise


  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += (GyroZ * elapsedTime) / 5.0;  // Slows down yaw accumulation 3x


  // Normalize yaw
  if (yaw > 2 * PI) yaw -= 2 * PI;
  if (yaw < 0) yaw += 2 * PI;

  previousTime = currentTime;  // 🟢 Move this to the end
}


void calculate_IMU_error() {
  long gyroX_sum = 0;
  long gyroY_sum = 0;
  long gyroZ_sum = 0;

  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

    gyroX_sum += rawGyroX;
    gyroY_sum += rawGyroY;
    gyroZ_sum += rawGyroZ;

    delay(5);  // small delay between samples
  }

  GyroErrorX = (gyroX_sum / 200.0) / 131.0;
  GyroErrorY = (gyroY_sum / 200.0) / 131.0;
  GyroErrorZ = (gyroZ_sum / 200.0) / 131.0;

  Serial.print("Gyro bias (Z): ");
  Serial.println(GyroErrorZ, 6);
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
  int speedLeft = baseSpeedA * 0.4;
  int speedRight = 255;

  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  analogWrite(EN_A, speedLeft);

  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(EN_B, speedRight);
}

// void rotateRight() {
//   digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
//   analogWrite(EN_A, baseSpeedA*0.5) ;

//   digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
//   analogWrite(EN_B, baseSpeedB);
// }

void rotateRightBurst() {
  static unsigned long lastBurstTime = 0;
  static int phase = 0;
  const int burstTime = 150;

  if (millis() - lastBurstTime < burstTime) return;

  if (phase == 0) {
    // Left motor backward
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    analogWrite(EN_A, baseSpeedA);

    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(EN_B, 0);

    phase = 1;
  } else {
    // Right motor forward
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    analogWrite(EN_A, 0);

    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(EN_B, baseSpeedB * 0.8);

    phase = 0;
  }

  lastBurstTime = millis();
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
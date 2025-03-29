#define IN1 4    // Motor 1 Direction 1
#define IN2 A1    // Motor 1 Direction 2
#define IN3 8    // Motor 2 Direction 1
#define IN4 7    // Motor 2 Direction 2

#define EN_A 5   // Enable pin for Motor 1 (PWM)
#define EN_B 6   // Enable pin for Motor 2 (PWM)

int speedA = 150; // Speed for Motor 1 (0-255)
int speedB = 150; // Speed for Motor 2 (0-255)

void setup() {
    Serial.begin(115200); // Initialize Serial Monitor
    Serial.println("Initializing motor control pins...");

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
    Serial.println("Motors enabled with initial speeds.");
}

void loop() {
    Serial.println("Rotating motors FORWARD...");

    // Motor 1 Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    Serial.print("Motor 1: FORWARD at speed ");
    Serial.println(speedA);

    // Motor 2 Forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.print("Motor 2: FORWARD at speed ");
    Serial.println(speedB);

    delay(5000);

    Serial.println("Reversing motor direction...");

    // Motor 1 Reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    Serial.println("Motor 1: REVERSE");

    // Motor 2 Reverse
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Motor 2: REVERSE");

    delay(5000);

    Serial.println("Stopping motors...");

    // Stop Motors
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(EN_A, 0);
    analogWrite(EN_B, 0);
    Serial.println("Motors: STOPPED");

    delay(5000);
}

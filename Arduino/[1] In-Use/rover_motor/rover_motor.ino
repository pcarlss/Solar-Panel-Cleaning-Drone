#define IN1 4  // Motor 1 Direction 1
#define IN2 5  // Motor 1 Direction 2
#define IN3 6  // Motor 2 Direction 1
#define IN4 7  // Motor 2 Direction 2

void setup() {
    Serial.begin(115200); // Initialize Serial Monitor
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop() {
    // Motor 1 Forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    Serial.println("Motor 1: FORWARD");

    // Motor 2 Forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Motor 2: FORWARD");

    delay(5000);

    // Motor 1 Reverse
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    Serial.println("Motor 1: REVERSE");

    // Motor 2 Reverse
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Motor 2: REVERSE");

    delay(5000);

    // Stop Motors
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Motors: STOPPED");

    delay(5000);
}

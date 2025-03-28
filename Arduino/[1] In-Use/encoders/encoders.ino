#define CLK1 2   // Encoder 1 Clock pin (Interrupt)
#define DT1 A2   // Encoder 1 Data pin

#define CLK2 3   // Encoder 2 Clock pin (Interrupt)
#define DT2 A3   // Encoder 2 Data pin

volatile int encoderValue1 = 0;
volatile int encoderValue2 = 0;

int lastStateCLK1;
int lastStateCLK2;

void setup() {
    Serial.begin(115200);

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
}

void loop() {
    // Continuously print encoder values
    Serial.print("Encoder 1: ");
    Serial.print(encoderValue1);
    Serial.print(" | Encoder 2: ");
    Serial.println(encoderValue2);
  // Small delay for serial output readability
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

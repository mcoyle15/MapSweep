// Motor A (Right) pins
const int ENA = 5;  // Enable pin for motor A (PWM)
const int IN1 = 8;  // Control pin 1 for motor A
const int IN2 = 9;  // Control pin 2 for motor A

// Motor B (Left) pins
const int ENB = 6;  // Enable pin for motor B (PWM)
const int IN3 = 10; // Control pin 1 for motor B
const int IN4 = 11; // Control pin 2 for motor B

// Hall sensor pins
const int hallSensor1A = 2; // Hall sensor A for motor 1
const int hallSensor1B = 3; // Hall sensor B for motor 1
const int hallSensor2A = 4; // Hall sensor A for motor 2
const int hallSensor2B = 5; // Hall sensor B for motor 2

// Encoder counters
volatile long hallCount1A = 0;
volatile long hallCount1B = 0;
volatile long hallCount2A = 0;
volatile long hallCount2B = 0;

// Interrupt service routines
void hallSensor1A_ISR() {
  hallCount1A++;
}

void hallSensor1B_ISR() {
  hallCount1B++;
}

void hallSensor2A_ISR() {
  hallCount2A++;
}

void hallSensor2B_ISR() {
  hallCount2B++;
}

void setup() {
  // Set all the motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize motors to off
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  // Set Hall sensor pins to inputs
  pinMode(hallSensor1A, INPUT);
  pinMode(hallSensor1B, INPUT);
  pinMode(hallSensor2A, INPUT);
  pinMode(hallSensor2B, INPUT);

  // Attach interrupts to Hall sensor pins
  attachInterrupt(digitalPinToInterrupt(hallSensor1A), hallSensor1A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(hallSensor1B), hallSensor1B_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(hallSensor2A), hallSensor2A_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(hallSensor2B), hallSensor2B_ISR, RISING);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Test Motor A (Right)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255); // Full speed

  delay(2000); // Run for 2 seconds

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 255); // Full speed in reverse

  delay(2000); // Run for 2 seconds

  digitalWrite(ENA, LOW); // Stop Motor A

  // Test Motor B (Left)
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 255); // Full speed

  delay(2000); // Run for 2 seconds

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 255); // Full speed in reverse

  delay(2000); // Run for 2 seconds

  digitalWrite(ENB, LOW); // Stop Motor B

  // Print Hall sensor counts
  Serial.print("Motor 1 Hall Sensor A Count: ");
  Serial.println(hallCount1A);
  Serial.print("Motor 1 Hall Sensor B Count: ");
  Serial.println(hallCount1B);
  Serial.print("Motor 2 Hall Sensor A Count: ");
  Serial.println(hallCount2A);
  Serial.print("Motor 2 Hall Sensor B Count: ");
  Serial.println(hallCount2B);

  delay(2000); // Wait for 2 seconds before repeating
}
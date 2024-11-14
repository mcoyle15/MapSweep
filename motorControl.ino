#include <Arduino.h>

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

// Function to set motor speed and direction
void setMotorSpeed(int enablePin, int in1Pin, int in2Pin, float speed) {
  // Constrain speed to valid range
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
  analogWrite(enablePin, abs(speed));
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
  if (Serial.available() > 0) {
    // Read the command from the serial port
    String command = Serial.readStringUntil('\n');
    float linear = command.substring(0, command.indexOf(',')).toFloat();
    float angular = command.substring(command.indexOf(',') + 1).toFloat();

    // Calculate motor speeds
    float rightSpeed = linear + angular;
    float leftSpeed = linear - angular;

    // Set motor speeds
    setMotorSpeed(ENA, IN1, IN2, rightSpeed);
    setMotorSpeed(ENB, IN3, IN4, leftSpeed);
  }

  // Print Hall sensor counts for debugging
  Serial.print("Motor 1 Hall Sensor A Count: ");
  Serial.println(hallCount1A);
  Serial.print("Motor 1 Hall Sensor B Count: ");
  Serial.println(hallCount1B);
  Serial.print("Motor 2 Hall Sensor A Count: ");
  Serial.println(hallCount2A);
  Serial.print("Motor 2 Hall Sensor B Count: ");
  Serial.println(hallCount2B);

  delay(1000); // Adjust the delay as needed
}
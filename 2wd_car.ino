/*
2wd_car
Autonomous car, 2 wheel drive with ultrasound ranger on servo.
Copyright Ali Raheem 2024 -
MIT Licensed
File version: 2024-11-10 20:40 GMT
*/

#include <Servo.h>

const bool USE_SERIAL = true;

// H-bridge pins for motors
const int R_BAK = 4; // Right motor backward control pin
const int R_FWD = 5; // Right motor forward control pin
const int L_BAK = 6; // Left motor backward control pin
const int L_FWD = 7; // Left motor forward control pin

// PWM outputs for motor speeds
const int LEFT_FWD = 180;  // Speed for left motor forward
const int RIGHT_FWD = 200; // Speed for right motor forward
const int LEFT_BAK = 200;  // Speed for left motor backward
const int RIGHT_BAK = 200; // Speed for right motor backward

// Sensor and servo pins
const int SERVO = 3;      // Servo control pin
const int US_TRIG = 11;   // Ultrasonic sensor trigger pin
const int US_ECHO = 10;   // Ultrasonic sensor echo pin

// Servo object to control the ultrasonic sensor's movement
Servo scanner;

// Constants
const int NUM_RANGES = 16;
const int SERIAL_BAUD = 9600;
const int MOTOR_START_DELAY = 10; // Delay for high-power pulse to overcome static friction

void setup() {
  // Initialize motor control pins as outputs
  pinMode(R_FWD, OUTPUT);
  pinMode(R_BAK, OUTPUT);
  pinMode(L_FWD, OUTPUT);
  pinMode(L_BAK, OUTPUT);

  // Begin serial communication for debugging
  if (USE_SERIAL) {
    Serial.begin(SERIAL_BAUD);
    Serial.println("Starting up car");
  }

  // Attach the servo to its control pin
  scanner.attach(SERVO);

  // Initialize ultrasonic sensor pins
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
}

void loop() {
  static uint16_t ranges[NUM_RANGES] = {0}; // Array to store distance measurements
  static byte i = 0; // Index for scanning positions (0 to 15)

  // Get the current distance measurement
  uint16_t range_cm = getRangeTicks();

  if(USE_SERIAL)
    Serial.println(range_cm);
    
  if (20 < range_cm)
    goFWD();
  else{
    stopAll();
    for(i = 0; i < NUM_RANGES; i++) {
      scanner.write(20 + i * 10);
      delay(200);
      ranges[i] = getRangeTicks();
      if(USE_SERIAL)
        Serial.println(ranges[i]);
    }
    scanner.write(80);
    byte dir = findBestPath(ranges, NUM_RANGES);
    if (dir < NUM_RANGES/2)
      revLeft();
    else revRight();
    delay(1000);
  }

  // Determine the best direction to move based on sensor data
  byte dir = findBestPath(ranges, NUM_RANGES);

}

// Function to find the best path based on a sliding window and weighted scoring
byte findBestPath(uint16_t *ranges, int numReadings) {
  const int windowSize = 3; // Size of each sliding window
  int maxScore = 0;
  byte bestIndex = 0;

  for (int i = 0; i <= numReadings - windowSize; i++) {
    int windowSum = 0;

    // Calculate weighted score for this window
    for (int j = 0; j < windowSize; j++) {
      int angle = 10 + (i + j) * 10;
      int weight = 180 - abs(90 - angle);
      windowSum += ranges[i + j] * weight;
    }

    if (windowSum > maxScore) {
      maxScore = windowSum;
      bestIndex = i + windowSize / 2;
    }
  }
  return bestIndex;
}

constexpr uint16_t cmToTicks(uint16_t cm) {
  return cm * 58;
}

// Function to get a distance measurement from the ultrasonic sensor
unsigned long getRangeTicks() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  unsigned long duration = pulseIn(US_ECHO, HIGH, 25000); // Timeout after 25ms
  uint16_t ticks = static_cast<uint16_t>(duration >> 4); // drop 4 bits to fit in 16bits
  return duration * 0.0343;
}

// Function to move the car forward with high-speed pulse
void goFWD() {
  // Ensure motors are stopped in the opposite direction before moving forward
  analogWrite(R_BAK, 0);
  analogWrite(L_BAK, 0);

  // High-speed pulse to overcome static friction
  analogWrite(R_FWD, 255);
  analogWrite(L_FWD, 255);
  delay(MOTOR_START_DELAY);

  // Set motor to desired speed
  analogWrite(R_FWD, RIGHT_FWD);
  analogWrite(L_FWD, LEFT_FWD);
}

// Function to move the car backward with high-speed pulse
void goBAK() {
  // Ensure motors are stopped in the opposite direction before moving backward
  analogWrite(R_FWD, 0);
  analogWrite(L_FWD, 0);

  // High-speed pulse to overcome static friction
  analogWrite(R_BAK, 255);
  analogWrite(L_BAK, 255);
  delay(MOTOR_START_DELAY);

  // Set motor to desired speed
  analogWrite(R_BAK, RIGHT_BAK);
  analogWrite(L_BAK, LEFT_BAK);
}

// Function to turn the car left with high-speed pulse
void goLeft() {
  // Ensure motors are stopped in the opposite direction
  analogWrite(R_BAK, 0);
  analogWrite(L_FWD, 0);

  // High-speed pulse to overcome static friction
  analogWrite(R_FWD, 255);
  analogWrite(L_BAK, 255);
  delay(MOTOR_START_DELAY);

  // Set motor to desired speed
  analogWrite(R_FWD, RIGHT_FWD);
  analogWrite(L_BAK, LEFT_BAK);
}

// Function to turn the car right with high-speed pulse
void goRight() {
  // Ensure motors are stopped in the opposite direction
  analogWrite(R_FWD, 0);
  analogWrite(L_BAK, 0);

  // High-speed pulse to overcome static friction
  analogWrite(R_BAK, 255);
  analogWrite(L_FWD, 255);
  delay(MOTOR_START_DELAY);

  // Set motor to desired speed
  analogWrite(R_BAK, RIGHT_BAK);
  analogWrite(L_FWD, LEFT_FWD);
}
// Function to turn the car left with high-speed pulse
void revLeft() {
  // Ensure motors are stopped in the opposite direction
  analogWrite(R_BAK, 0);
  analogWrite(L_FWD, 0);

  // High-speed pulse to overcome static friction
  analogWrite(L_BAK, 255);
  delay(MOTOR_START_DELAY);

  // Set motor to desired speed
  analogWrite(L_BAK, LEFT_BAK);
}

// Function to turn the car right with high-speed pulse
void revRight() {
  // Ensure motors are stopped in the opposite direction
  analogWrite(R_FWD, 0);
  analogWrite(L_BAK, 0);

  // High-speed pulse to overcome static friction
  analogWrite(R_BAK, 255);
  delay(MOTOR_START_DELAY);

  // Set motor to desired speed
  analogWrite(R_BAK, RIGHT_BAK);
}

// Function to stop all motor movements
void stopAll() {
  analogWrite(R_FWD, 0);
  analogWrite(L_FWD, 0);
  analogWrite(R_BAK, 0);
  analogWrite(L_BAK, 0);
}

#include <ESP32Servo.h>
#include <Wire.h>
#include "MCP23017.h"

// Light sensor pins
#define LIGHT_SENSOR_1 36 // VP - front
#define LIGHT_SENSOR_2 39 // VN - right
#define LIGHT_SENSOR_3 34 // D34 - left 
#define LIGHT_SENSOR_4 15 // D15 - back

// // Thresholds based on your test
// #define THRESHOLD1 750
// #define THRESHOLD2 700
// #define THRESHOLD3 1300
// #define THRESHOLD4 700

// Navigation configuration
#define NUM_SENSORS 4
#define WINDOW_SIZE 20
#define LOWER_THRESHOLD 1.2
#define UPPER_THRESHOLD 2.0

float baseline[NUM_SENSORS] = {0};
float baselineStdDev[NUM_SENSORS] = {0};
int recentReadings[NUM_SENSORS][WINDOW_SIZE] = {0};
int sampleCount = 0;
bool calibrated = false;

// Servo pins (only using 2 directly connected for now)
#define SERVO_2 32 // D32 - working
#define SERVO_4 13 // D13 - working

// Servo microsecond range
#define USMIN 600
#define USMAX 2400

// Motor A (left motor)
#define ENA 17  // TX2
#define IN1 18  // D18
#define IN2 19  // D19

// Motor B (right motor)
#define ENB 21  // D21
#define IN3 4   // D4
#define IN4 23  // D23

// Create servo objects
Servo servo2;
Servo servo4;

// navigation con

void setup() {
  Serial.begin(115200);
  Serial.println("Robotic Flower - Light Response Test");

  // Attach servos with calibrated pulse widths
  servo2.attach(SERVO_2, USMIN, USMAX);
  servo4.attach(SERVO_4, USMIN, USMAX);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Start with petals closed
  closeAllPetals();
  delay(300);
}

void loop() {
  // Read sensor values
  int light1 = analogRead(LIGHT_SENSOR_1);
  int light2 = analogRead(LIGHT_SENSOR_2);
  int light3 = analogRead(LIGHT_SENSOR_3);
  int light4 = analogRead(LIGHT_SENSOR_4);

  // Print sensor data
  Serial.print("Sensor values: ");
  Serial.print(light1); Serial.print(", ");
  Serial.print(light2); Serial.print(", ");
  Serial.print(light3); Serial.print(", ");
  Serial.println(light4);

  // Check if any sensor detects *bright* light (value below threshold)
  if (light1 < THRESHOLD1 || light2 < THRESHOLD2 ||
      light3 < THRESHOLD3 || light4 < THRESHOLD4) {
    Serial.println("Bright light detected - Opening petals!");
    
    
    openAllPetals();
    moveForward();
  } else {
    Serial.println("No bright light - Closing petals.");
    closeAllPetals();
    stopMotors();
  }

  Serial.println("-----------------------------");
  delay(500);
}

void openAllPetals() {
  servo2.write(90); // Move servo counterclockwise
  servo4.write(90);
}

void closeAllPetals() {
  servo2.write(0);  // Move servo clockwise back to original
  servo4.write(0);
}

void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200); // Speed 0-255

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200); // Speed 0-255
}

void moveBackward() {
  Serial.println("Moving backward");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

void stopMotors() {
  Serial.println("Stopping motors");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

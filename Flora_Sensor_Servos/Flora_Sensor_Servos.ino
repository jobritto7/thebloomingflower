#include <ESP32Servo.h>
#include <Wire.h>
#include "MCP23017.h"
#include <math.h>

// Sensor direction mapping
#define LIGHT_SENSOR_1 36 // VP - front
#define LIGHT_SENSOR_2 39 // VN - right
#define LIGHT_SENSOR_3 34 // D34 - left 
#define LIGHT_SENSOR_4 15 // D15 - back

Servo servo1;
Servo servo2;

// Servo pins
#define SERVO_1 32
#define SERVO_2 13

// Motor A (left)
#define ENA 17
#define IN1 18
#define IN2 19

// Motor B (right)
#define ENB 21
#define IN3 4
#define IN4 23

#define NUM_SENSORS 4
#define WINDOW_SIZE 20
#define BRIGHT_THRESHOLD 1.2
#define STOP_THRESHOLD 2.0

// Servo range
#define USMIN 600
#define USMAX 2400


// Sensor processing
int sensorPins[NUM_SENSORS] = {
  LIGHT_SENSOR_1,
  LIGHT_SENSOR_2,
  LIGHT_SENSOR_3,
  LIGHT_SENSOR_4
};

float baseline[NUM_SENSORS] = {0};
float baselineStdDev[NUM_SENSORS] = {0};
int recentReadings[NUM_SENSORS][WINDOW_SIZE] = {0};
int sampleCount = 0;
int bufferIndex = 0;
bool calibrated = false;

// Bloom state
bool lightDetected = false;
bool petalsOpen = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Robotic Flower - Light Navigation Setup");

  servo1.attach(SERVO_1, USMIN, USMAX);
  servo2.attach(SERVO_2, USMIN, USMAX);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  closeAllPetals();
  Serial.println("Idle start: petals closed for 1 minute.");
  // delay(60000); // Idle for 60 seconds
  delay(500); // 500 milliseconds = 0.5 seconds

  Serial.println("Getting started!");
  spin360();
}

void loop() {
  int readings[NUM_SENSORS];
  float currentAvg[NUM_SENSORS] = {0};

  // Read and store sensor values
  for (int i = 0; i < NUM_SENSORS; i++) {
    readings[i] = analogRead(sensorPins[i]);
    recentReadings[i][bufferIndex] = readings[i];
  }

  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;
  sampleCount++;

  if (sampleCount < WINDOW_SIZE) {
    Serial.println("Calibrating...");
    delay(500);
    return;
  }

  if (!calibrated) {
    // Compute baseline and std deviation
    for (int i = 0; i < NUM_SENSORS; i++) {
      float sum = 0;
      for (int j = 0; j < WINDOW_SIZE; j++) {
        sum += recentReadings[i][j];
      }
      baseline[i] = sum / WINDOW_SIZE;

      float variance = 0;
      for (int j = 0; j < WINDOW_SIZE; j++) {
        variance += pow(recentReadings[i][j] - baseline[i], 2);
      }
      baselineStdDev[i] = sqrt(variance / WINDOW_SIZE);
    }
    calibrated = true;
    Serial.println("Calibration complete.");
  }

  float currentStdDev[NUM_SENSORS];
  float stdDevRatio[NUM_SENSORS];

  int brightestDirection = -1;
  float maxRatio = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    float sum = 0;
    for (int j = 0; j < WINDOW_SIZE; j++) {
      sum += recentReadings[i][j];
    }
    currentAvg[i] = sum / WINDOW_SIZE;

    float variance = 0;
    for (int j = 0; j < WINDOW_SIZE; j++) {
      variance += pow(recentReadings[i][j] - currentAvg[i], 2);
    }
    currentStdDev[i] = sqrt(variance / WINDOW_SIZE);

    stdDevRatio[i] = baselineStdDev[i] > 0 ? currentStdDev[i] / baselineStdDev[i] : 0;

    if (stdDevRatio[i] > maxRatio && currentAvg[i] < baseline[i]) {
      maxRatio = stdDevRatio[i];
      brightestDirection = i;
    }

    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": Avg=");
    Serial.print(currentAvg[i]);
    Serial.print(" Ratio=");
    Serial.println(stdDevRatio[i]);
  }

  Serial.print("Max ratio = ");
  Serial.println(maxRatio);

  // Light-based behavior
  if (maxRatio > BRIGHT_THRESHOLD) {
    lightDetected = true;
    Serial.print("Significant light detected. Direction: ");
    Serial.println(brightestDirection);

    if (brightestDirection == 0 && maxRatio > STOP_THRESHOLD) {
      Serial.println("Bright light ahead. Stopping.");
      flutterPetals();
      stopMotors();
    } else {
      rotateToward(brightestDirection);
    }
  } else {
    lightDetected = false;
    Serial.println("No significant light detected.");
    stopMotors();
  }

  // Petal control based on light presence
  if (lightDetected && !petalsOpen) {
    openAllPetals();
    petalsOpen = true;
  } else if (!lightDetected && petalsOpen) {
    closeAllPetals();
    petalsOpen = false;
  }

  Serial.println("--------");
  delay(500);
}

// Movement and servo control

void openAllPetals() {
  servo1.write(90);
  servo2.write(90);
}

void closeAllPetals() {
  servo1.write(0);
  servo2.write(0);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);
}

void rotateToward(int direction) {
  switch (direction) {
    case 1: // Right
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 150);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 150);
      break;
    case 2: // Left
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 150);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 150);
      break;
    case 3: // Back (180° turn)
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 150);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 150);
      delay(800);
      break;
    default: // Forward
      moveForward();
      break;
  }

  delay(400);
  stopMotors();
}

spin360() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150);

  delay(1600); // Calibrate this to complete one full rotation

  stopMotors();
}


void flutterPetals(int times = 3) {
  for (int i = 0; i < times; i++) {
    servo2.write(60);
    servo4.write(60);
    delay(200);
    servo2.write(120);
    servo4.write(120);
    delay(200);
  }

  // Return to open position after dancing
  openAllPetals();
}
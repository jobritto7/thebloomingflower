#include <ESP32Servo.h>
#include <Wire.h>
#include "MCP23017.h"
#include <math.h>
// sensor 2, 13 and 30


// Sensor direction mapping
#define LIGHT_SENSOR_1 36 // VP - front
#define LIGHT_SENSOR_2 39 // VN - right
#define LIGHT_SENSOR_3 34 // D34 - left
#define LIGHT_SENSOR_4 15 // D15 - right

Servo servo1;
Servo servo2;

// Servo pins
#define SERVO_1 32
#define SERVO_2 13

// Motor A (left) - front
#define ENA 17
#define IN1 18
#define IN2 19

// Motor B (right) - front
#define ENB 21
#define IN3 4
#define IN4 23

// Motor C - left back
#define ENC 12
#define IN5 14
#define IN6 27

// Motor D - right back
#define END 25
#define IN7 26
#define IN8 22


#define NUM_SENSORS 4
#define WINDOW_SIZE 20
#define REBASELINE_SIZE 20

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

int sensorThreshold1s[NUM_SENSORS] = {
  10,
  15,
  15,
  15
};

int sensorThreshold2s[NUM_SENSORS] = {
  100,
  25,
  25,
  25
};
int thresh1Counter[NUM_SENSORS] = {0};
int thresh2Counter[NUM_SENSORS] = {0};
float baseline[NUM_SENSORS] = {0};
float baselineStdDev[NUM_SENSORS] = {0};
int recentReadings[NUM_SENSORS][WINDOW_SIZE] = {0};
int recentNoDetectionReadings[NUM_SENSORS][WINDOW_SIZE] = {0};
void calibrate(int recent[NUM_SENSORS][WINDOW_SIZE]);
int readings[NUM_SENSORS];
float currentAvg[NUM_SENSORS] = {0};
int sampleCount = 0;
int bufferIndex = 0;
bool calibrated = false;
int recalibrateIndex = 0;
int skipped = 0;

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
  // delay(500); // 500 milliseconds = 0.5 seconds

  Serial.println("Getting started!");
  // spin360();
}

void spin360() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 150);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 150);

  delay(500); // Calibrate this to complete one full rotation

  stopMotors();
}


void flutterPetals(int times = 1) {
  Serial.println("fluttering!");
  for (int i = 0; i < times; i++) {
    servo1.write(60);
    servo2.write(60);
    delay(200);
    servo1.write(120);
    servo2.write(120);
    delay(200);
  }

  // Return to open position after dancing
  openAllPetals();
}

void loop() {
  if (skipped++ < 30) {
    Serial.println("Skipping...");
    delay(200);
    return;
  }
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
    Serial.print("Not calibrated calibrating");
    calibrate(recentReadings);
  }
  else if (recalibrateIndex % WINDOW_SIZE == 0) {
    Serial.print("Needs recal calibrating");
    calibrate(recentNoDetectionReadings);
    recalibrateIndex = (recalibrateIndex + 1) % WINDOW_SIZE;
  }

  float currentStdDev[NUM_SENSORS];
  float stdDevRatio[NUM_SENSORS];

  int brightestDirection = -1;
  int maxRatio = 0;
  bool thresh1Exceeded = false;
  bool thresh2Exceeded = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    float sum = 0;
    for (int j = 0; j < WINDOW_SIZE; j++) {
      sum += recentReadings[i][j];
    }
    currentAvg[i] = sum / WINDOW_SIZE;

    float variance = pow(readings[i] - baseline[i], 2);  // or use recentReadings again
    currentStdDev[i] = sqrt(variance);

    stdDevRatio[i] = baselineStdDev[i] > 0 ? currentStdDev[i] / baselineStdDev[i] : 0;

    // Brightest direction logic
    if ((stdDevRatio[i] > maxRatio) && (readings[i] < baseline[i])) {
      maxRatio = stdDevRatio[i];
      brightestDirection = i;
    }

    // Check threshold 1
    if ((stdDevRatio[i] > sensorThreshold1s[i]) && (readings[i] < baseline[i])) {
      thresh1Counter[i]++;
      if (thresh1Counter[i] >= 3) {
        thresh1Exceeded = true;
      }
    } else {
      thresh1Counter[i] = 0;  // Reset if not exceeded
    }

    // Check threshold 2
    if ((stdDevRatio[i] > sensorThreshold2s[i]) && (readings[i] < baseline[i])) {
      thresh2Counter[i]++;
      if (thresh2Counter[i] >= 3) {
        thresh2Exceeded = true;
      }
    } else {
      thresh2Counter[i] = 0;  // Reset if not exceeded
    }

    // Debug prints
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": Baseline=");
    Serial.print(baseline[i]);
    Serial.print(" - Avg=");
    Serial.print(currentAvg[i]);
    Serial.print(" - Reading=");
    Serial.print(readings[i]);
    Serial.print(" - Variance=");
    Serial.print(variance);
    Serial.print(" Ratio=");
    Serial.print(stdDevRatio[i]);
    Serial.print(" - Baseline Std=");
    Serial.print(baselineStdDev[i]);
    Serial.print(" Current Std=");
    Serial.print(currentStdDev[i]);
    Serial.print(" - Thresh1 Count=");
    Serial.print(thresh1Counter[i]);
    Serial.print(" - Thresh2 Count=");
    Serial.println(thresh2Counter[i]);
  }


  Serial.print("Max ratio = ");
  Serial.print(maxRatio);
  Serial.print("- Recal index = ");
  Serial.println(recalibrateIndex);



  // Light-based behavior
  if (thresh1Exceeded) {
    lightDetected = true;
    Serial.print("Significant light detected. Direction: ");
    Serial.println(brightestDirection);

    if (brightestDirection == 0 && thresh2Exceeded) {
      Serial.println("Bright light ahead. Stopping.");
      // flutterPetals();
      stopMotors();
    } else {
      rotateToward(brightestDirection);
    }
  } else {
    for (int i = 0; i < NUM_SENSORS; i++) {
      recentNoDetectionReadings[i][recalibrateIndex] = readings[i];
    }
    recalibrateIndex = (recalibrateIndex + 1) % WINDOW_SIZE;
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

void calibrate(int recent[NUM_SENSORS][WINDOW_SIZE]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int sorted[WINDOW_SIZE];

    // Copy to temp array
    for (int j = 0; j < WINDOW_SIZE; j++) {
      sorted[j] = recent[i][j];
    }

    // Simple sort (can use better sorting if needed)
    for (int j = 0; j < WINDOW_SIZE - 1; j++) {
      for (int k = j + 1; k < WINDOW_SIZE; k++) {
        if (sorted[j] > sorted[k]) {
          int temp = sorted[j];
          sorted[j] = sorted[k];
          sorted[k] = temp;
        }
      }
    }

    // Use middle 10 values (ignore bottom 5 and top 5)
    float sum = 0;
    for (int j = 5; j < 15; j++) {
      sum += sorted[j];
    }
    baseline[i] = sum / 10.0;

    float variance = 0;
    for (int j = 5; j < 15; j++) {
      variance += pow(sorted[j] - baseline[i], 2);
    }
    baselineStdDev[i] = sqrt(variance / 10.0);
  }

  calibrated = true;
  Serial.println("Calibration (with outlier filtering) complete.");
}

// Movement and servo control

void openAllPetals() {
  Serial.println("openAllPetals");
  servo1.write(90);
  servo2.write(90);
}

void closeAllPetals() {
  Serial.println("closeAllPetals");
  servo1.write(0);
  servo2.write(0);
}

void stopMotors() {
  Serial.println("stopMotors");
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);
}

void moveForward() {
  Serial.println("moveForward");
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 220);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 220);

  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  analogWrite(ENC, 220);

  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
  analogWrite(END, 220);
}

void rotateToward(int direction) {
  switch (direction) {
    case 1: // Right
      Serial.println("Turning right");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 220);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 220);
      break;
    case 2: // Left
      Serial.println("Turning left");
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, 220);

      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 220);

      digitalWrite(IN5, HIGH);
      digitalWrite(IN6, LOW);
      analogWrite(ENC, 220);

      digitalWrite(IN7, LOW);
      digitalWrite(IN8, HIGH);
      analogWrite(END, 220);
      
      break;
    case 3: // Back (180° turn)
      Serial.println("180 turn");
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 220);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, 220);
      digitalWrite(IN5, HIGH);
      digitalWrite(IN6, LOW);
      analogWrite(ENC, 220);
      digitalWrite(IN7, LOW);
      digitalWrite(IN8, HIGH);
      analogWrite(END, 220);
      delay(800);
      break;
    default: // Forward
      Serial.println("Move forward rotate toward");
      moveForward();
      break;
  }

  delay(400);
  stopMotors();
}
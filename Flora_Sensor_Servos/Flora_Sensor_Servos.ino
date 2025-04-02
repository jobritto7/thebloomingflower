// THE ROBOTIC FLOWER
// Code put together by Inga Woods-Waight
// TikTok - @ingawoods.waight
// Instagram - @ingawoods.waight

// libraries communicate with Arduino board
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

// constants won't change
const int DISTANCE_THRESHOLD = 150;  // centimeters

// Light sensor pins
const int LIGHT_SENSOR_1 = 34;  // VP
const int LIGHT_SENSOR_2 = 39;  // VN
const int LIGHT_SENSOR_3 = 34;  // D34
const int LIGHT_SENSOR_4 = 15;  // D15

// Servo pins
const int SERVO_1 = 35;  // D35
const int SERVO_2 = 32;  // D32
const int SERVO_3 = 33;  // D33
const int SERVO_4 = 13;  // D13

// Light threshold for opening petals
const int LIGHT_THRESHOLD = 2000; 

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 150   // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 600   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN 600      // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX 2400     // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50  // Analog servos run at ~50 Hz updates

// variables will change:
float duration_us, distance_cm;

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  Serial.begin(9600);
  Serial.println("4 channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  // Attach servos to their pins
  servo1.attach(SERVO_1);
  servo2.attach(SERVO_2);
  servo3.attach(SERVO_3);
  servo4.attach(SERVO_4);
  
  // Set initial position (closed)
  closeAllPetals();
}

void loop() {
  // Read light sensor values
  int light1 = analogRead(LIGHT_SENSOR_1);
  int light2 = analogRead(LIGHT_SENSOR_2);
  int light3 = analogRead(LIGHT_SENSOR_3);
  int light4 = analogRead(LIGHT_SENSOR_4);
  
  // Print sensor values for debugging
  Serial.print("Light Sensors: ");
  Serial.print(light1);
  Serial.print(", ");
  Serial.print(light2);
  Serial.print(", ");
  Serial.print(light3);
  Serial.print(", ");
  Serial.println(light4);
  
  // Check if any light sensor exceeds threshold
  if (light1 > LIGHT_THRESHOLD || light2 > LIGHT_THRESHOLD || 
      light3 > LIGHT_THRESHOLD || light4 > LIGHT_THRESHOLD) {
    openAllPetals();
  } else {
    closeAllPetals();
  }
  
  delay(100);  // Small delay to prevent too frequent updates
}

void openAllPetals() {
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
}

void closeAllPetals() {
  servo1.write(0);
  servo2.write(0);
  servo3.write(0);
  servo4.write(0);
}
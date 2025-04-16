#include <ESP32Servo.h>

#define LIGHT_SENSOR_PIN_1 36 // light sensor 1 VP
#define LIGHT_SENSOR_PIN_2 39 // light sensor 2 VN
#define LIGHT_SENSOR_PIN_3 34 // light sensor 3 D34
#define LIGHT_SENSOR_PIN_4 15 // light sensor 4 D15

#define THRESHOLD1 750  // Adjust based on actual readings
#define THRESHOLD2 700
#define THRESHOLD3 1300
#define THRESHOLD4 700

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4; 

void setup() {
    Serial.begin(115200);

}

void loop() {
    int sensorValue1 = analogRead(LIGHT_SENSOR_PIN_1);
    int sensorValue2 = analogRead(LIGHT_SENSOR_PIN_2);
    int sensorValue3 = analogRead(LIGHT_SENSOR_PIN_3);
    int sensorValue4 = analogRead(LIGHT_SENSOR_PIN_4);

    Serial.print("Light Sensor 1 Intensity: ");
    //int maxLight = max(sensorValue);
    Serial.println(sensorValue1);

    Serial.print("Light Sensor 2 Intensity: ");
    //int maxLight = max(sensorValue);
    Serial.println(sensorValue2);

    Serial.print("Light Sensor 3 Intensity: ");
    //int maxLight = max(sensorValue);
    Serial.println(sensorValue3);

    Serial.print("Light Sensor 4 Intensity: ");
    //int maxLight = max(sensorValue);
    Serial.println(sensorValue4);


    if (sensorValue1 < THRESHOLD1) {
        Serial.println("Light Sensor 1: Bright light detected! (Flashlight ON)");

    } else {
        Serial.println("Light Sensor 1: No bright light detected. (Ambient light)");
    }

    
    if (sensorValue2 < THRESHOLD2) {
        Serial.println("Light Sensor 2: Bright light detected! (Flashlight ON)");
    } else {
        Serial.println("Light Sensor 2: No bright light detected. (Ambient light)");
    }

    if (sensorValue3 < THRESHOLD3) {
        Serial.println("Light Sensor 3: Bright light detected! (Flashlight ON)");
    } else {
        Serial.println("Light Sensor 3: No bright light detected. (Ambient light)");
    }

    if (sensorValue4 < THRESHOLD4) {
        Serial.println("Light Sensor 4: Bright light detected! (Flashlight ON)");
    } else {
        Serial.println("Light Sensor 4: No bright light detected. (Ambient light)");
    }

    Serial.println("-----------------------------");
    delay(500); // cycles
}


// #define LIGHT_SENSOR_PIN 15  light sensor 4 D15
// #define LIGHT_SENSOR_PIN 34 // light sensor 3 D34
// #define LIGHT_SENSOR_PIN 36 // light sensor 1 VP
#define LIGHT_SENSOR_PIN 39 // light sensor 2 VN
#define THRESHOLD 2000  // Adjust based on actual readings

void setup() {
    Serial.begin(115200);
}

void loop() {
    int sensorValue = analogRead(LIGHT_SENSOR_PIN);
    Serial.print("Light Intensity: ");
    //int maxLight = max(sensorValue);
    Serial.println(sensorValue);

    if (sensorValue < THRESHOLD) {
        Serial.println("Bright light detected! (Flashlight ON)");
    } else {
        Serial.println("No bright light detected. (Ambient light)");
    }

    delay(500);
}


#include "IRSensor.h"

IRSensor::IRSensor() {
    umbrellaDetected = false;
    lastReadTime = 0;
}

void IRSensor::init() {
    pinMode(IR_SENSOR_PIN, INPUT);
    umbrellaDetected = false;
}

void IRSensor::update() {
    unsigned long currentTime = millis();
    if (currentTime - lastReadTime >= READ_INTERVAL) {
        // IR sensor typically outputs LOW when object is detected
        // Adjust this logic based on your specific IR sensor module
        umbrellaDetected = (digitalRead(IR_SENSOR_PIN) == LOW);
        lastReadTime = currentTime;
    }
}

bool IRSensor::isUmbrellaDetected() {
    return umbrellaDetected;
}

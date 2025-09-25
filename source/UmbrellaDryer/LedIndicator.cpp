#include "LedIndicator.h"

LedIndicator::LedIndicator(int pin1, int pin2, int pin3)
    : ledPin1(pin1), ledPin2(pin2), ledPin3(pin3) {}

void LedIndicator::init() {
    Serial.print("Initializing LED!");
    pinMode(ledPin1, OUTPUT);
    pinMode(ledPin2, OUTPUT);
    pinMode(ledPin3, OUTPUT);
    digitalWrite(ledPin1, LOW);
    digitalWrite(ledPin2, LOW);
    digitalWrite(ledPin3, LOW);
    delay(500);
    Serial.println("Successful!");
}

void LedIndicator::set(int ledPin, bool on) {
    digitalWrite(ledPin, on ? HIGH : LOW);
}

#include "TactileButton.h"

TactileButton::TactileButton() : first_time(true) {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        inputState[i] = LOW;
        lastInputState[i] = LOW;
        inputFlags[i] = LOW;
        lastDebounceTime[i] = 0;
    }
}

void TactileButton::init() {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        pinMode(inputPins[i], INPUT);
        digitalWrite(inputPins[i], HIGH);
    }
    delay(1000);
    Serial.println("Push Buttons: Initialized!");
}

void TactileButton::inputAction(int BUTTON_PIN) {
    Serial.println(BUTTON_PIN);
}

void TactileButton::setInputFlags() {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        int reading = digitalRead(inputPins[i]);
        if (reading != lastInputState[i]) {
            lastDebounceTime[i] = millis();
        }
        if (millis() - lastDebounceTime[i] > DEBOUNCE_DELAY) {
            if (reading != inputState[i]) {
                inputState[i] = reading;
                if (inputState[i] == HIGH) {
                    inputFlags[i] = HIGH;
                }
            }
        }
        lastInputState[i] = reading;
    }
}

void TactileButton::resolveInputFlags() {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        if (inputFlags[i] == HIGH) {
            if (i == 3 && first_time) {
                first_time = false;
            } else {
                inputAction(i);
            }
            inputFlags[i] = LOW;
        }
    }
}

#include "TactileButton.h"

// Forward declaration for SYSTEM_MODE
#ifndef SYSTEM_MODE
#define MODE_DEVELOPMENT 1
#define MODE_PRODUCTION 2
#define SYSTEM_MODE MODE_DEVELOPMENT
#endif

TactileButton::TactileButton() : first_time(true) {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        inputState[i] = LOW;
        lastInputState[i] = LOW;
        inputFlags[i] = LOW;
        buttonPressed[i] = false;
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
    #if SYSTEM_MODE == MODE_DEVELOPMENT
    Serial.println(BUTTON_PIN);
    #else
    // Production mode button actions will be handled in main loop
    // Set flags for main program to process
    #endif
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
                #if SYSTEM_MODE == MODE_PRODUCTION
                buttonPressed[i] = true; // Set flag for main program
                #endif
            }
            inputFlags[i] = LOW;
        }
    }
}

bool TactileButton::getButtonPressed(int buttonIndex) {
    if (buttonIndex >= 0 && buttonIndex < BUTTON_COUNT) {
        bool pressed = buttonPressed[buttonIndex];
        buttonPressed[buttonIndex] = false; // Clear flag
        return pressed;
    }
    return false;
}

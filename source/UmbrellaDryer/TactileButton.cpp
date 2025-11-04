#include "TactileButton.h"

// Forward declaration for SYSTEM_MODE
#ifndef SYSTEM_MODE
#define MODE_DEVELOPMENT 1
#define MODE_PRODUCTION 2
#define SYSTEM_MODE MODE_DEVELOPMENT
#endif

TactileButton::TactileButton() : first_time(true) {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        inputState[i] = HIGH;      // Default state is HIGH (not pressed)
        lastInputState[i] = HIGH;  // Default state is HIGH (not pressed)
        inputFlags[i] = LOW;
        buttonPressed[i] = false;
        lastDebounceTime[i] = 0;
    }
}

void TactileButton::init() {
    for (int i = 0; i < BUTTON_COUNT; i++) {
        pinMode(inputPins[i], INPUT_PULLUP);  // Use INPUT_PULLUP for proper pull-up
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
                if (inputState[i] == LOW) {  // Button pressed = LOW with pull-up
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
                buttonPressed[i] = true; // Always set flag for main program to check
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

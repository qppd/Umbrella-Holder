#include "LimitSwitch.h"

LimitSwitch::LimitSwitch() {
    doorClosed = false;
    lastSwitchState = HIGH;
    currentSwitchState = HIGH;
    lastDebounceTime = 0;
}

void LimitSwitch::init() {
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
    doorClosed = false;
}

void LimitSwitch::update() {
    int reading = digitalRead(LIMIT_SWITCH_PIN);
    
    // If the switch state has changed, reset the debounce timer
    if (reading != lastSwitchState) {
        lastDebounceTime = millis();
    }
    
    // Only update the state if enough time has passed (debouncing)
    if ((millis() - lastDebounceTime) > SWITCH_DEBOUNCE_DELAY) {
        // If the reading has changed
        if (reading != currentSwitchState) {
            currentSwitchState = reading;
            // Limit switch is pressed (LOW) when door is closed
            doorClosed = (currentSwitchState == LOW);
        }
    }
    
    lastSwitchState = reading;
}

bool LimitSwitch::isDoorClosed() {
    return doorClosed;
}

bool LimitSwitch::isDoorOpen() {
    return !doorClosed;
}

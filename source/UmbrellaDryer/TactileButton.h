#ifndef TACTILE_BUTTON_H
#define TACTILE_BUTTON_H

#include <Arduino.h>


#include "Pins.h"

class TactileButton {
public:
    TactileButton();
    void init();
    void setInputFlags();
    void resolveInputFlags();
    void inputAction(int BUTTON_PIN);
    const int inputPins[BUTTON_COUNT] = { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 };

private:
    int inputState[BUTTON_COUNT];
    int lastInputState[BUTTON_COUNT];
    bool inputFlags[BUTTON_COUNT];
    long lastDebounceTime[BUTTON_COUNT];
    bool first_time;
};

#endif // TACTILE_BUTTON_H

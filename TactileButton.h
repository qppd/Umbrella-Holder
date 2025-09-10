#ifndef TACTILE_BUTTON_H
#define TACTILE_BUTTON_H

#include <Arduino.h>

#define BUTTON_COUNT 4
#define DEBOUNCE_DELAY 50

#define BUTTON_1 4
#define BUTTON_2 5
#define BUTTON_3 6
#define BUTTON_4 7

class TactileButton {
public:
    TactileButton();
    void init();
    void setInputFlags();
    void resolveInputFlags();
    void inputAction(int BUTTON_PIN);

private:
    int inputState[BUTTON_COUNT];
    int lastInputState[BUTTON_COUNT];
    bool inputFlags[BUTTON_COUNT];
    long lastDebounceTime[BUTTON_COUNT];
    const int inputPins[BUTTON_COUNT] = { BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4 };
    bool first_time;
};

#endif // TACTILE_BUTTON_H

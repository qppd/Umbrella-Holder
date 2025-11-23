#ifndef LIMITSWITCH_H
#define LIMITSWITCH_H

#include <Arduino.h>
#include "Pins.h"

class LimitSwitch {
public:
    LimitSwitch();
    void init();
    bool isDoorClosed(); // Returns true if door is closed (switch pressed)
    bool isDoorOpen(); // Returns true if door is open (switch not pressed)
    void update(); // Call regularly to read switch state with debouncing

private:
    bool doorClosed;
    int lastSwitchState;
    int currentSwitchState;
    unsigned long lastDebounceTime;
    const unsigned long SWITCH_DEBOUNCE_DELAY = 50; // 50ms debounce
};

#endif // LIMITSWITCH_H

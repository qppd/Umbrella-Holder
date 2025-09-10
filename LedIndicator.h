#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <Arduino.h>
// ...existing code...
#include "Pins.h"

class LedIndicator {
public:
    LedIndicator(int pin1 = LED_1, int pin2 = LED_2, int pin3 = LED_3);
    void init();
    void set(int ledPin, bool on);
private:
    int ledPin1, ledPin2, ledPin3;
};

#endif // LED_INDICATOR_H

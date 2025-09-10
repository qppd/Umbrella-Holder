#ifndef LED_INDICATOR_H
#define LED_INDICATOR_H

#include <Arduino.h>

class LedIndicator {
public:
    LedIndicator(int pin1 = 2, int pin2 = 3, int pin3 = 11);
    void init();
    void set(int ledPin, bool on);
private:
    int ledPin1, ledPin2, ledPin3;
};

#endif // LED_INDICATOR_H

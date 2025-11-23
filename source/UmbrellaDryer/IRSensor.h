#ifndef IRSENSOR_H
#define IRSENSOR_H

#include <Arduino.h>
#include "Pins.h"

class IRSensor {
public:
    IRSensor();
    void init();
    bool isUmbrellaDetected(); // Returns true if umbrella is detected
    void update(); // Call regularly to read sensor state

private:
    bool umbrellaDetected;
    unsigned long lastReadTime;
    const unsigned long READ_INTERVAL = 100; // Read every 100ms
};

#endif // IRSENSOR_H

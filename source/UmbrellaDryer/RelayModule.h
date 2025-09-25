#ifndef RELAY_MODULE_H
#define RELAY_MODULE_H

#include <Arduino.h>

#include "Pins.h"

class RelayModule {
public:
    RelayModule(uint16_t relay1 = RELAY_HEATER, uint16_t relay2 = RELAY_BLOWER);
    void init();
    void set(uint16_t relay, bool opened);
private:
    uint16_t relay1, relay2;
};

#endif // RELAY_MODULE_H

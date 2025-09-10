#ifndef RELAY_MODULE_H
#define RELAY_MODULE_H

#include <Arduino.h>

class RelayModule {
public:
    RelayModule(uint16_t relay1 = 8, uint16_t relay2 = 9);
    void init();
    void set(uint16_t relay, bool opened);
private:
    uint16_t relay1, relay2;
};

#endif // RELAY_MODULE_H

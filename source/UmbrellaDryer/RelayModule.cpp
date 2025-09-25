#include "RelayModule.h"

RelayModule::RelayModule(uint16_t relay1, uint16_t relay2)
    : relay1(relay1), relay2(relay2) {}

void RelayModule::init() {
    pinMode(relay1, OUTPUT);
    pinMode(relay2, OUTPUT);
}

void RelayModule::set(uint16_t relay, bool opened) {
    digitalWrite(relay, opened ? HIGH : LOW);
}

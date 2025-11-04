#ifndef PINS_H
#define PINS_H

// Tactile Buttons
#define BUTTON_1 4  // Start cycle
#define BUTTON_2 5  // Stop cycle
#define BUTTON_3 6  // Increase temperature (+5°C)
#define BUTTON_4 7  // Decrease temperature (-5°C)
#define BUTTON_COUNT 4
#define DEBOUNCE_DELAY 50

// DHT Sensor
#define DHTPIN 10
#define DHTTYPE DHT22

// Relays
#define RELAY_HEATER 8
#define RELAY_BLOWER 9

#endif // PINS_H

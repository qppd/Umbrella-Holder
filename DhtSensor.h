#ifndef DHT_SENSOR_H
#define DHT_SENSOR_H

#include <Arduino.h>
#include "DHT.h"


#include "Pins.h"

class DhtSensor {
public:
    DhtSensor();
    void begin();
    float getTemperature(bool isFahrenheit = false);
    float getHumidity();
private:
    DHT dht;
};

#endif // DHT_SENSOR_H

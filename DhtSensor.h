#ifndef DHT_SENSOR_H
#define DHT_SENSOR_H

#include <Arduino.h>
#include "DHT.h"

#define DHTPIN 10
#define DHTTYPE DHT22

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

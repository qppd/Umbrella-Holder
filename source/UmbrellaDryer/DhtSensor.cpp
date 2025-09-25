
#include "DhtSensor.h"
#include "Pins.h"

DhtSensor::DhtSensor() : dht(DHTPIN, DHTTYPE) {}

void DhtSensor::begin() {
    dht.begin();
}

float DhtSensor::getTemperature(bool isFahrenheit) {
    float temperature;
    if (isFahrenheit) {
        temperature = dht.readTemperature(true);
    } else {
        temperature = dht.readTemperature();
    }
    if (isnan(temperature)) return -1;
    else return temperature;
}

float DhtSensor::getHumidity() {
    float humidity = dht.readHumidity();
    if (isnan(humidity)) return -1;
    else return humidity;
}

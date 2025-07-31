#include "DHT.h"
#define DHTPIN 10  // Digital pin connected to the DHT sensor

//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22  // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);

void initDHT() {
  dht.begin();
}



/*
  Function/Method for getting DHT temperature
  
*/
float getDHTTemperature(boolean isFarenheit) {
  float temperature;
  if (isFarenheit) {
    temperature = dht.readTemperature(true);
  } else {
    temperature = dht.readTemperature();
  }
  if (isnan(temperature)) return -1;
  else return temperature;
}


/*
  Function/Method for getting DHT humidity
*/
float getDHTHumidity() {
  float humidity = dht.readHumidity();
  if (isnan(humidity)) return -1;
  else return humidity;
}
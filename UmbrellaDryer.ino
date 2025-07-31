#include "MY_BUTTON.h"
#include "MY_DHT.h"
#include "MY_LCD.h"
#include "MY_LED.h"
#include "MY_PID.h"
#include "MY_RELAY.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Umbrella Holder/Dryer!");

  initBUTTONS();
  initDHT();
  initLCD();
  initLED();
  initRELAY();
  initPID();

  setLCDText("Initialized!!!", 0, 0);
  delay(3000);
  clearLCD();
}

unsigned long loop_time_for_interval;
int loop_interval = 1000;
bool isOpen = false;

void loop() {

  setInputFlags();
  resolveInputFlags();

  if (millis() > loop_time_for_interval + loop_interval) {
    loop_time_for_interval = millis();

    CURRENT_TEMPERATURE = getDHTTemperature(true);
    getDHTHumidity();
    pidCOMPUTE();

    if (isOpen) {
      operateLED(LED_PIN_1, true);
      operateLED(LED_PIN_2, true);
      operateLED(LED_PIN_3, true);
      operateSSR(RELAY_1, true);
      operateSSR(RELAY_2, true);
      isOpen = false;
    } else {
      operateLED(LED_PIN_1, false);
      operateLED(LED_PIN_2, false);
      operateLED(LED_PIN_3, false);
      operateSSR(RELAY_1, false);
      operateSSR(RELAY_2, false);
      isOpen = true;
    }
  }
}

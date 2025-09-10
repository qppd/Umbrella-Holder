
#include "TactileButton.h"
#include "DhtSensor.h"
#include "I2cDisplay.h"
#include "LedIndicator.h"
#include "PidController.h"
#include "RelayModule.h"

#define MODE_DEVELOPMENT 1
#define MODE_PRODUCTION 2
#define SYSTEM_MODE MODE_DEVELOPMENT // Change to MODE_PRODUCTION for full code

TactileButton buttons;
DhtSensor dht;
I2cDisplay lcd;
LedIndicator leds;
PidController pid;
RelayModule relays;

String serialInput = "";
bool isOpen = true;
unsigned long loop_time_for_interval = 0;
const unsigned long loop_interval = 5000; // 5 seconds, adjust as needed

void setup() {
  Serial.begin(115200);
  Serial.println("Umbrella Holder/Dryer!");

  buttons.init();
  dht.begin();
  lcd.init();
  leds.init();
  relays.init();
  pid.init();

  lcd.setText("Initialized!!!", 0, 0);
  delay(3000);
  lcd.clear();
}

void loop() {
#if SYSTEM_MODE == MODE_DEVELOPMENT
  // Serial command-based testing
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialInput.trim();
      if (serialInput == "test_button") {
        Serial.println("Testing Buttons...");
        buttons.setInputFlags();
        buttons.resolveInputFlags();
      } else if (serialInput == "test_dht") {
        Serial.println("Testing DHT...");
        float t = dht.getTemperature();
        float h = dht.getHumidity();
        Serial.print("Temp: "); Serial.println(t);
        Serial.print("Humidity: "); Serial.println(h);
      } else if (serialInput == "test_lcd") {
        Serial.println("Testing LCD...");
        lcd.setText("LCD OK", 0, 0);
        delay(1000);
        lcd.clear();
      } else if (serialInput == "test_led") {
        Serial.println("Testing LEDs...");
        leds.set(2, true); delay(300);
        leds.set(3, true); delay(300);
        leds.set(11, true); delay(300);
        leds.set(2, false); leds.set(3, false); leds.set(11, false);
      } else if (serialInput == "test_pid") {
        Serial.println("Testing PID...");
        pid.setCurrentTemperature(dht.getTemperature());
        pid.compute();
        Serial.print("PID Output: "); Serial.println(pid.getOutput());
      } else if (serialInput == "test_relay") {
        Serial.println("Testing Relays...");
        relays.set(8, true); delay(500);
        relays.set(9, true); delay(500);
        relays.set(8, false); relays.set(9, false);
      } else {
        Serial.println("Unknown command. Try: test_button, test_dht, test_lcd, test_led, test_pid, test_relay");
      }
      serialInput = "";
    } else {
      serialInput += c;
    }
  }
#else
  // PRODUCTION: Full system logic
  buttons.setInputFlags();
  buttons.resolveInputFlags();

  if (millis() > loop_time_for_interval + loop_interval) {
    loop_time_for_interval = millis();
    float temp = dht.getTemperature(true);
    float hum = dht.getHumidity();
    pid.setCurrentTemperature(temp);
    pid.compute();
    // Example: control relays/leds based on PID output or other logic
    if (isOpen) {
      leds.set(2, true);
      leds.set(3, true);
      leds.set(11, true);
      relays.set(8, true);
      relays.set(9, true);
      isOpen = false;
    } else {
      leds.set(2, false);
      leds.set(3, false);
      leds.set(11, false);
      relays.set(8, false);
      relays.set(9, false);
      isOpen = true;
    }
  }
#endif
}

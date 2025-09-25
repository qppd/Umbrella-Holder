

#include "TactileButton.h"
#include "DhtSensor.h"
#include "I2cDisplay.h"
#include "LedIndicator.h"
#include "PidController.h"
#include "RelayModule.h"
#include "Pins.h"

#define MODE_DEVELOPMENT 1
#define MODE_PRODUCTION 2
#define SYSTEM_MODE MODE_PRODUCTION // Change to MODE_PRODUCTION for full code

TactileButton buttons;
DhtSensor dht;
I2cDisplay lcd;
LedIndicator leds;
PidController pid;
RelayModule relays;

String serialInput = "";

// Production mode variables
const unsigned long DRYING_DURATION = 8UL * 60UL * 1000UL; // 8 minutes in ms
// Pins are now defined in Pins.h
const double HEATER_SETPOINT = 60.0;

bool dryingActive = false;
unsigned long dryingStartTime = 0;

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
  leds.set(LED_1, true); delay(300);
  leds.set(LED_2, true); delay(300);
  leds.set(LED_3, true); delay(300);
  leds.set(LED_1, false); leds.set(LED_2, false); leds.set(LED_3, false);
      } else if (serialInput == "test_pid") {
        Serial.println("Testing PID...");
        pid.setCurrentTemperature(dht.getTemperature());
        pid.compute();
        Serial.print("PID Output: "); Serial.println(pid.getOutput());
      } else if (serialInput == "test_relay") {
        Serial.println("Testing Relays...");
      relays.set(RELAY_HEATER, true); delay(500);
      relays.set(RELAY_BLOWER, true); delay(500);
      relays.set(RELAY_HEATER, false); relays.set(RELAY_BLOWER, false);
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
  // Start drying cycle on boot if not already started
  if (!dryingActive) {
    dryingActive = true;
    dryingStartTime = millis();
    // Turn on blower and heater
    relays.set(RELAY_BLOWER, true); // Blower always ON during drying
    relays.set(RELAY_HEATER, true); // Heater ON, will be PID controlled
  leds.set(LED_1, true); // Example: indicate drying
  leds.set(LED_2, true);
  leds.set(LED_3, true);
    pid.setSetpoint(HEATER_SETPOINT);
    lcd.setText("Drying...", 0, 0);
  }

  // During drying
  if (dryingActive) {
    float temp = dht.getTemperature();
    pid.setCurrentTemperature(temp);
    pid.compute();
    double pidOutput = pid.getOutput();

    // Heater relay PID control (simple ON/OFF for SSR)
    if (temp < HEATER_SETPOINT - 1) {
      relays.set(RELAY_HEATER, true); // Turn heater ON
    } else if (temp > HEATER_SETPOINT + 1) {
      relays.set(RELAY_HEATER, false); // Turn heater OFF
    }
    // Blower always ON during drying
    relays.set(RELAY_BLOWER, true);

    // Check if drying time is over
    if (millis() - dryingStartTime >= DRYING_DURATION) {
      // Stop everything
      relays.set(RELAY_HEATER, false);
      relays.set(RELAY_BLOWER, false);
  leds.set(LED_1, false);
  leds.set(LED_2, false);
  leds.set(LED_3, false);
      lcd.setText("Done!", 0, 0);
      dryingActive = false;
    }
  }
#endif
}

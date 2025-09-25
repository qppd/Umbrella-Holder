

#include "TactileButton.h"
#include "DhtSensor.h"
#include "I2cDisplay.h"
#include "LedIndicator.h"
#include "PidController.h"
#include "RelayModule.h"
#include "Pins.h"

#define MODE_DEVELOPMENT 1
#define MODE_PRODUCTION 2
#define SYSTEM_MODE MODE_DEVELOPMENT // Set to DEVELOPMENT for serial testing

TactileButton buttons;
DhtSensor dht;
I2cDisplay lcd;
LedIndicator leds;
PidController pid;
RelayModule relays;

char serialInput[32];
uint8_t inputIndex = 0;
bool serialComplete = false;

// Production mode variables
const unsigned long DRYING_DURATION = 8UL * 60UL * 1000UL; // 8 minutes in ms
const double HEATER_SETPOINT = 60.0;

bool dryingActive = false;
unsigned long dryingStartTime = 0;

// Testing variables
bool continuousMonitoring = false;
bool interactiveMode = false;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 2000; // 2 seconds

// System status variables
struct SystemStatus {
  bool dhtOk;
  bool lcdOk;
  bool ledsOk;
  bool relaysOk;
  bool buttonsOk;
  bool pidOk;
} systemStatus = {false, false, false, false, false, false};

void setup() {
  Serial.begin(115200);
  Serial.println(F("UMBRELLA DRYER - DEV MODE"));
  
  Serial.print(F("Init... "));
  
  buttons.init();
  systemStatus.buttonsOk = true;
  
  dht.begin();
  delay(500);
  float testTemp = dht.getTemperature();
  systemStatus.dhtOk = (testTemp != -1);
  
  lcd.init();
  systemStatus.lcdOk = true;
  
  leds.init();
  systemStatus.ledsOk = true;
  
  relays.init();
  systemStatus.relaysOk = true;
  
  pid.init();
  systemStatus.pidOk = true;

  Serial.println(F("OK"));
  
  lcd.setText("DEV MODE", 0, 0);
  lcd.setText("Type help", 0, 1);
  
  Serial.println(F("Type 'help' for commands"));
  
  // Clear input buffer
  memset(serialInput, 0, sizeof(serialInput));
  inputIndex = 0;
}

void loop() {
#if SYSTEM_MODE == MODE_DEVELOPMENT
  // Handle serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serialInput[inputIndex] = '\0';
      serialComplete = true;
    } else if (inputIndex < sizeof(serialInput) - 1) {
      serialInput[inputIndex++] = tolower(c);
    }
  }
  
  // Process complete commands
  if (serialComplete) {
    processSerialCommand(serialInput);
    memset(serialInput, 0, sizeof(serialInput));
    inputIndex = 0;
    serialComplete = false;
  }
  
  // Handle continuous monitoring
  if (continuousMonitoring && (millis() - lastSensorRead >= SENSOR_READ_INTERVAL)) {
    printSensorReadings();
    lastSensorRead = millis();
  }
  
  // Handle interactive button testing
  if (interactiveMode) {
    buttons.setInputFlags();
    buttons.resolveInputFlags();
    delay(50);
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

// ========================================
//           TESTING FUNCTIONS
// ========================================

void processSerialCommand(const char* command) {
  if (strcmp_P(command, PSTR("help")) == 0 || strcmp_P(command, PSTR("?")) == 0) {
    printHelp();
  } else if (strcmp_P(command, PSTR("status")) == 0) {
    printSystemStatus();
  } else if (strcmp_P(command, PSTR("sensors")) == 0) {
    printSensorReadings();
  } else if (strcmp_P(command, PSTR("test_dht")) == 0) {
    testDHTSensor();
  } else if (strcmp_P(command, PSTR("test_lcd")) == 0) {
    testLCDDisplay();
  } else if (strcmp_P(command, PSTR("test_led")) == 0) {
    testLEDIndicators();
  } else if (strcmp_P(command, PSTR("test_relay")) == 0) {
    testRelayModule();
  } else if (strcmp_P(command, PSTR("test_button")) == 0) {
    testTactileButtons();
  } else if (strcmp_P(command, PSTR("test_pid")) == 0) {
    testPIDController();
  } else if (strcmp_P(command, PSTR("monitor")) == 0) {
    continuousMonitoring = !continuousMonitoring;
    Serial.println(continuousMonitoring ? F("Monitor ON") : F("Monitor OFF"));
  } else if (strcmp_P(command, PSTR("h_on")) == 0) {
    relays.set(RELAY_HEATER, true);
    Serial.println(F("Heater ON"));
  } else if (strcmp_P(command, PSTR("h_off")) == 0) {
    relays.set(RELAY_HEATER, false);
    Serial.println(F("Heater OFF"));
  } else if (strcmp_P(command, PSTR("b_on")) == 0) {
    relays.set(RELAY_BLOWER, true);
    Serial.println(F("Blower ON"));
  } else if (strcmp_P(command, PSTR("b_off")) == 0) {
    relays.set(RELAY_BLOWER, false);
    Serial.println(F("Blower OFF"));
  } else if (strcmp_P(command, PSTR("led_on")) == 0) {
    leds.set(LED_1, true);
    leds.set(LED_2, true);
    leds.set(LED_3, true);
    Serial.println(F("LEDs ON"));
  } else if (strcmp_P(command, PSTR("led_off")) == 0) {
    leds.set(LED_1, false);
    leds.set(LED_2, false);
    leds.set(LED_3, false);
    Serial.println(F("LEDs OFF"));
  } else if (strcmp_P(command, PSTR("clear")) == 0) {
    lcd.clear();
    Serial.println(F("LCD cleared"));
  } else if (strcmp_P(command, PSTR("reset")) == 0) {
    Serial.println(F("Resetting..."));
    delay(500);
    asm volatile ("  jmp 0");
  } else {
    Serial.println(F("Unknown. Type 'help'"));
  }
}

void printHelp() {
  Serial.println(F("\n--- COMMANDS ---"));
  Serial.println(F("status - System status"));
  Serial.println(F("sensors - Sensor data"));
  Serial.println(F("test_dht - Test DHT22"));
  Serial.println(F("test_lcd - Test LCD"));
  Serial.println(F("test_led - Test LEDs"));
  Serial.println(F("test_relay - Test relays"));
  Serial.println(F("test_button - Test buttons"));
  Serial.println(F("test_pid - Test PID"));
  Serial.println(F("monitor - Toggle monitoring"));
  Serial.println(F("h_on/h_off - Heater control"));
  Serial.println(F("b_on/b_off - Blower control"));
  Serial.println(F("led_on/led_off - LED control"));
  Serial.println(F("clear - Clear LCD"));
  Serial.println(F("reset - Reset system"));
}

void printSystemStatus() {
  Serial.println(F("\n--- STATUS ---"));
  Serial.print(F("DHT: ")); Serial.println(systemStatus.dhtOk ? F("OK") : F("ERR"));
  Serial.print(F("LCD: ")); Serial.println(systemStatus.lcdOk ? F("OK") : F("ERR"));
  Serial.print(F("LED: ")); Serial.println(systemStatus.ledsOk ? F("OK") : F("ERR"));
  Serial.print(F("RELAY: ")); Serial.println(systemStatus.relaysOk ? F("OK") : F("ERR"));
  Serial.print(F("BTN: ")); Serial.println(systemStatus.buttonsOk ? F("OK") : F("ERR"));
  Serial.print(F("PID: ")); Serial.println(systemStatus.pidOk ? F("OK") : F("ERR"));
  Serial.print(F("Monitor: ")); Serial.println(continuousMonitoring ? F("ON") : F("OFF"));
}



void printSensorReadings() {
  float temp = dht.getTemperature();
  float hum = dht.getHumidity();
  
  Serial.print(F("T:"));
  Serial.print(temp);
  Serial.print(F("C H:"));
  Serial.print(hum);
  Serial.println(F("%"));
  
  pid.setCurrentTemperature(temp);
  pid.compute();
  Serial.print(F("PID:"));
  Serial.println(pid.getOutput());
  
  // Update LCD
  lcd.clear();
  lcd.setText("T:", 0, 0);
  lcd.setText(temp, 2, 0);
  lcd.setText("H:", 8, 0);
  lcd.setText(hum, 10, 0);
}



void testDHTSensor() {
  float temp = dht.getTemperature();
  float hum = dht.getHumidity();
  
  Serial.print(F("DHT - T:"));
  Serial.print(temp);
  Serial.print(F(" H:"));
  Serial.print(hum);
  Serial.println(temp != -1 && hum != -1 ? F(" OK") : F(" FAIL"));
  
  systemStatus.dhtOk = (temp != -1 && hum != -1);
}

void testLCDDisplay() {
  lcd.clear();
  lcd.setText("LCD TEST", 0, 0);
  delay(1000);
  lcd.clear();
  Serial.println(F("LCD OK"));
  systemStatus.lcdOk = true;
}

void testLEDIndicators() {
  leds.set(LED_1, true); delay(200);
  leds.set(LED_2, true); delay(200);
  leds.set(LED_3, true); delay(200);
  leds.set(LED_1, false);
  leds.set(LED_2, false);
  leds.set(LED_3, false);
  Serial.println(F("LED OK"));
  systemStatus.ledsOk = true;
}

void testRelayModule() {
  relays.set(RELAY_HEATER, true);
  delay(500);
  relays.set(RELAY_BLOWER, true);
  delay(500);
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
  Serial.println(F("Relay OK"));
  systemStatus.relaysOk = true;
}

void testTactileButtons() {
  Serial.println(F("Press buttons for 5s..."));
  unsigned long start = millis();
  
  while (millis() - start < 5000) {
    buttons.setInputFlags();
    for (int i = 0; i < 4; i++) {
      if (digitalRead(buttons.inputPins[i]) == HIGH) {
        Serial.print(F("BTN")); Serial.println(i + 1);
        delay(200);
      }
    }
  }
  systemStatus.buttonsOk = true;
}

void testPIDController() {
  float temp = dht.getTemperature();
  if (temp == -1) {
    Serial.println(F("PID - DHT error"));
    return;
  }
  
  pid.setCurrentTemperature(temp);
  pid.compute();
  
  Serial.print(F("PID - T:"));
  Serial.print(temp);
  Serial.print(F(" Out:"));
  Serial.println(pid.getOutput());
  
  systemStatus.pidOk = true;
}



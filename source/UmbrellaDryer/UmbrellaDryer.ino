

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
const double MIN_SAFE_TEMP = -10.0; // Minimum valid temperature
const double MAX_SAFE_TEMP = 80.0;  // Maximum safe temperature
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000; // 1 second
const unsigned long SENSOR_TIMEOUT = 5000; // 5 seconds for sensor timeout

// System states
enum SystemState {
  STATE_STANDBY,
  STATE_STARTING,
  STATE_DRYING,
  STATE_COMPLETED,
  STATE_ERROR,
  STATE_EMERGENCY_STOP
};

SystemState currentState = STATE_STANDBY;
bool dryingActive = false;
unsigned long dryingStartTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSensorRead = 0;
bool displayMode = 0; // 0: Status, 1: Detailed
double targetTemperature = HEATER_SETPOINT;
bool emergencyStop = false;
int sensorErrorCount = 0;

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
  
#if SYSTEM_MODE == MODE_DEVELOPMENT
  Serial.println(F("UMBRELLA DRYER - DEV MODE"));
  Serial.print(F("Init... "));
#else
  Serial.println(F("UMBRELLA DRYER - PRODUCTION MODE"));
#endif
  
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

#if SYSTEM_MODE == MODE_DEVELOPMENT
  Serial.println(F("OK"));
  lcd.setText("DEV MODE", 0, 0);
  lcd.setText("Type help", 0, 1);
  Serial.println(F("Type 'help' for commands"));
#else
  lcd.setText("UMBRELLA DRYER", 0, 0);
  lcd.setText("Initializing...", 0, 1);
  delay(2000);
  lcd.clear();
  lcd.setText("READY", 0, 0);
  lcd.setText("Press BTN1 to start", 0, 1);
#endif
  
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
  // PRODUCTION: Full system logic with state machine
  
  // Always check buttons for user input
  buttons.setInputFlags();
  buttons.resolveInputFlags();
  
  // Handle button presses
  handleButtonPresses();
  
  // Update sensors and safety checks
  updateSensorsAndSafety();
  
  // State machine logic
  switch (currentState) {
    case STATE_STANDBY:
      handleStandbyState();
      break;
    case STATE_STARTING:
      handleStartingState();
      break;
    case STATE_DRYING:
      handleDryingState();
      break;
    case STATE_COMPLETED:
      handleCompletedState();
      break;
    case STATE_ERROR:
      handleErrorState();
      break;
    case STATE_EMERGENCY_STOP:
      handleEmergencyStopState();
      break;
  }
  
  // Update display
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  
#endif
}

// ========================================
//        PRODUCTION MODE FUNCTIONS
// ========================================

void handleButtonPresses() {
  // Button 1: Start/Stop cycle
  if (buttons.getButtonPressed(0)) {
    if (currentState == STATE_STANDBY) {
      startDryingCycle();
    } else if (currentState == STATE_DRYING) {
      stopDryingCycle();
    }
  }
  
  // Button 2: Toggle display mode
  if (buttons.getButtonPressed(1)) {
    displayMode = !displayMode;
  }
  
  // Button 3: Temperature adjustment (±5°C)
  if (buttons.getButtonPressed(2)) {
    if (currentState == STATE_STANDBY) {
      targetTemperature += 5.0;
      if (targetTemperature > 70.0) targetTemperature = 50.0; // Cycle 50-70°C
    }
  }
  
  // Button 4: Emergency stop
  if (buttons.getButtonPressed(3)) {
    emergencyStop = true;
    currentState = STATE_EMERGENCY_STOP;
  }
}

void updateSensorsAndSafety() {
  float temp = dht.getTemperature();
  float hum = dht.getHumidity();
  
  // Check sensor validity
  if (temp == -1 || hum == -1) {
    sensorErrorCount++;
    if (sensorErrorCount > 3) {
      currentState = STATE_ERROR;
      return;
    }
  } else {
    sensorErrorCount = 0;
  }
  
  // Safety temperature checks
  if (temp < MIN_SAFE_TEMP || temp > MAX_SAFE_TEMP) {
    currentState = STATE_ERROR;
    return;
  }
  
  // Update PID controller
  if (currentState == STATE_DRYING) {
    pid.setCurrentTemperature(temp);
    pid.compute();
    
    // Use PID output for heater control (simplified)
    double pidOutput = pid.getOutput();
    bool heaterOn = (pidOutput > 128); // Threshold-based for SSR
    relays.set(RELAY_HEATER, heaterOn);
  }
}

void startDryingCycle() {
  currentState = STATE_STARTING;
  dryingActive = true;
  dryingStartTime = millis();
  pid.setSetpoint(targetTemperature);
  
  // Turn on systems
  relays.set(RELAY_BLOWER, true);
  leds.set(LED_1, true);
  leds.set(LED_2, true);
  leds.set(LED_3, true);
}

void stopDryingCycle() {
  currentState = STATE_COMPLETED;
  dryingActive = false;
  
  // Turn off everything
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
  leds.set(LED_1, false);
  leds.set(LED_2, false);
  leds.set(LED_3, false);
}

void handleStandbyState() {
  // System ready, waiting for user input
  leds.set(LED_1, false);
  leds.set(LED_2, false);
  leds.set(LED_3, false);
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
}

void handleStartingState() {
  // Brief startup phase
  currentState = STATE_DRYING;
}

void handleDryingState() {
  // Check if drying time is over
  if (millis() - dryingStartTime >= DRYING_DURATION) {
    stopDryingCycle();
  }
  
  // Keep blower running
  relays.set(RELAY_BLOWER, true);
}

void handleCompletedState() {
  // Cycle complete, stay here until reset or new cycle
  static unsigned long completedTime = millis();
  
  // Flash LED to indicate completion
  if ((millis() - completedTime) % 1000 < 500) {
    leds.set(LED_1, true);
  } else {
    leds.set(LED_1, false);
  }
}

void handleErrorState() {
  // Error state - flash all LEDs
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
  
  static unsigned long errorTime = millis();
  bool flashState = (millis() - errorTime) % 500 < 250;
  leds.set(LED_1, flashState);
  leds.set(LED_2, flashState);
  leds.set(LED_3, flashState);
  
  // Allow restart after 10 seconds
  if (millis() - errorTime > 10000) {
    currentState = STATE_STANDBY;
    sensorErrorCount = 0;
  }
}

void handleEmergencyStopState() {
  // Emergency stop - everything off
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
  leds.set(LED_1, false);
  leds.set(LED_2, false);
  leds.set(LED_3, false);
  
  // Require manual reset
  if (buttons.getButtonPressed(0) && buttons.getButtonPressed(1)) {
    emergencyStop = false;
    currentState = STATE_STANDBY;
  }
}

void updateDisplay() {
  lcd.clear();
  
  float temp = dht.getTemperature();
  float hum = dht.getHumidity();
  
  if (displayMode == 0) {
    // Status display
    switch (currentState) {
      case STATE_STANDBY:
        lcd.setText("READY - Press BTN1", 0, 0);
        lcd.setText("Target: ", 0, 1);
        lcd.setText(targetTemperature, 8, 1);
        lcd.setText("C", 13, 1);
        break;
      case STATE_DRYING:
        lcd.setText("DRYING...", 0, 0);
        unsigned long remaining = (DRYING_DURATION - (millis() - dryingStartTime)) / 1000;
        lcd.setText("Time: ", 0, 1);
        lcd.setText((int)(remaining / 60), 6, 1);
        lcd.setText(":", 8, 1);
        lcd.setText((int)(remaining % 60), 9, 1);
        break;
      case STATE_COMPLETED:
        lcd.setText("CYCLE COMPLETE!", 0, 0);
        lcd.setText("Press BTN1 for new", 0, 1);
        break;
      case STATE_ERROR:
        lcd.setText("ERROR - CHECK SYS", 0, 0);
        lcd.setText("Wait 10s to reset", 0, 1);
        break;
      case STATE_EMERGENCY_STOP:
        lcd.setText("EMERGENCY STOP", 0, 0);
        lcd.setText("BTN1+BTN2 to reset", 0, 1);
        break;
    }
  } else {
    // Detailed display
    lcd.setText("T:", 0, 0);
    lcd.setText(temp, 2, 0);
    lcd.setText("C H:", 7, 0);
    lcd.setText(hum, 10, 0);
    lcd.setText("%", 15, 0);
    
    if (currentState == STATE_DRYING) {
      lcd.setText("PID:", 0, 1);
      lcd.setText(pid.getOutput(), 4, 1);
      lcd.setText(" H:", 9, 1);
      lcd.setText(digitalRead(RELAY_HEATER) ? "ON" : "OFF", 12, 1);
    } else {
      lcd.setText("BTN2: Toggle view", 0, 1);
    }
  }
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



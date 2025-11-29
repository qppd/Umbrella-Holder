

#include "TactileButton.h"
#include "DhtSensor.h"
#include "I2cDisplay.h"
#include "PidController.h"
#include "RelayModule.h"
#include "IRSensor.h"
#include "LimitSwitch.h"
#include "Pins.h"

#define MODE_DEVELOPMENT 1
#define MODE_PRODUCTION 2
#define SYSTEM_MODE MODE_PRODUCTION // Set to DEVELOPMENT for serial testing

TactileButton buttons;
DhtSensor dht;
I2cDisplay lcd;
PidController pid;
RelayModule relays;
IRSensor irSensor;
LimitSwitch limitSwitch;

char serialInput[32];
uint8_t inputIndex = 0;
bool serialComplete = false;

// Production mode variables
const unsigned long HEATER_HEADSTART_DURATION = 2UL * 60UL * 1000UL; // 2 minutes heater-only head start
const unsigned long DRYING_DURATION = 10UL * 60UL * 1000UL; // 10 minutes total drying time (2 min heater only + 8 min heater+blower)
const double HEATER_SETPOINT = 60.0;
const double MIN_SAFE_TEMP = -10.0; // Minimum valid temperature
const double MAX_SAFE_TEMP = 80.0;  // Maximum safe temperature
const unsigned long DISPLAY_UPDATE_INTERVAL = 500; // 500ms for smooth updates
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
SystemState lastDisplayedState = STATE_EMERGENCY_STOP; // Force initial full update
bool dryingActive = false;
unsigned long dryingStartTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastSensorRead = 0;
bool displayMode = 0; // 0: Status, 1: Detailed
double targetTemperature = HEATER_SETPOINT;
bool emergencyStop = false;
int sensorErrorCount = 0;

// Auto-start variables
bool lastUmbrellaDetected = false;
bool lastDoorClosed = false;

// LCD update tracking to prevent flicker
String lastLine0 = "";
String lastLine1 = "";
String lastLine2 = "";
String lastLine3 = "";
float lastDisplayedTemp = -999;
float lastDisplayedHum = -999;
int lastDisplayedMinutes = -1;
int lastDisplayedSeconds = -1;

// Setpoint adjustment display
bool showingSetpoint = false;
unsigned long setpointAdjustTime = 0;
const unsigned long SETPOINT_DISPLAY_DURATION = 5000; // 5 seconds

// Testing variables
bool continuousMonitoring = false;
bool interactiveMode = false;
const unsigned long SENSOR_READ_INTERVAL = 2000; // 2 seconds

// System status variables
struct SystemStatus {
  bool dhtOk;
  bool lcdOk;
  bool relaysOk;
  bool buttonsOk;
  bool pidOk;
  bool limitOk;
  bool irOk;
} systemStatus = {false, false, false, false, false, false, false};

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
  
  relays.init();
  systemStatus.relaysOk = true;
  
  pid.init();
  systemStatus.pidOk = true;
  
  irSensor.init();
  limitSwitch.init();

#if SYSTEM_MODE == MODE_DEVELOPMENT
  Serial.println(F("OK"));
  lcd.setText("DEV MODE", 0, 0);
  lcd.setText("Type help", 0, 1);
  Serial.println(F("Type 'help' for commands"));
#else
  lcd.setText("  UMBRELLA DRYER  ", 0, 0);
  lcd.setText("  Initializing... ", 0, 1);
  delay(2000);
  lcd.clear();
  lcd.setText("  UMBRELLA DRYER  ", 0, 0);
  lcd.setText("Temp: --.-", 0, 1);
  lcd.setText((char)223, 10, 1); // Degree symbol
  lcd.setText("C", 11, 1);
  lcd.setText("Humi: --.-%", 0, 2);
  lcd.setText("Press BTN1: START", 0, 3);
  // Initialize last values to force first update
  lastDisplayedTemp = -999;
  lastDisplayedHum = -999;
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
  
  // Update IR sensor and limit switch
  irSensor.update();
  limitSwitch.update();
  
  // Check for auto-start condition
  checkAutoStart();
  
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

void checkAutoStart() {
  bool umbrellaDetected = irSensor.isUmbrellaDetected();
  bool doorClosed = limitSwitch.isDoorClosed();
  
  // Auto-start only when:
  // 1. Umbrella is detected by IR sensor
  // 2. Door is closed (limit switch pressed)
  // 3. System is in standby or completed state
  // 4. This is a new detection (edge trigger)
  if (umbrellaDetected && doorClosed && 
      (currentState == STATE_STANDBY || currentState == STATE_COMPLETED)) {
    // Only trigger on state change (edge detection)
    if (!lastUmbrellaDetected || !lastDoorClosed) {
      startDryingCycle();
      showingSetpoint = false; // Clear setpoint display when auto-starting
    }
  }
  
  // Prevent starting if door is closed but no umbrella detected
  if (doorClosed && !umbrellaDetected && 
      (currentState == STATE_STANDBY || currentState == STATE_COMPLETED)) {
    // Could display a message here if needed
  }
  
  // Update last states
  lastUmbrellaDetected = umbrellaDetected;
  lastDoorClosed = doorClosed;
}

void handleButtonPresses() {
  // Button 1: Start drying cycle
  if (buttons.getButtonPressed(0)) {
    if (currentState == STATE_STANDBY || currentState == STATE_COMPLETED) {
      startDryingCycle();
      showingSetpoint = false; // Clear setpoint display when starting
    }
  }
  
  // Button 2: Stop drying cycle
  if (buttons.getButtonPressed(1)) {
    if (currentState == STATE_DRYING) {
      stopDryingCycle();
    }
  }
  
  // Button 3: Increase temperature (+5°C)
  if (buttons.getButtonPressed(2)) {
    targetTemperature += 5.0;
    if (targetTemperature > 70.0) targetTemperature = 70.0; // Max 70°C
    showingSetpoint = true;
    setpointAdjustTime = millis();
    // Force display update
    lastDisplayedState = STATE_EMERGENCY_STOP;
  }
  
  // Button 4: Decrease temperature (-5°C)
  if (buttons.getButtonPressed(3)) {
    targetTemperature -= 5.0;
    if (targetTemperature < 50.0) targetTemperature = 50.0; // Min 50°C
    showingSetpoint = true;
    setpointAdjustTime = millis();
    // Force display update
    lastDisplayedState = STATE_EMERGENCY_STOP;
  }
  
  // Check if setpoint display timeout has elapsed
  if (showingSetpoint && (millis() - setpointAdjustTime >= SETPOINT_DISPLAY_DURATION)) {
    showingSetpoint = false;
    // Force display update to return to normal screen
    lastDisplayedState = STATE_EMERGENCY_STOP;
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
  
  // Update PID controller during drying
  if (currentState == STATE_DRYING) {
    pid.setCurrentTemperature(temp);
    pid.compute();
    
    // Use PID output for heater control (threshold-based like proven heater controller)
    bool heaterOn = pid.isHeatingRequired(); // true if PID output > 0
    relays.set(RELAY_HEATER, heaterOn);
  }
}

void startDryingCycle() {
  currentState = STATE_STARTING;
  dryingActive = true;
  dryingStartTime = millis();
  pid.setSetpoint(targetTemperature);
  
  // Start with heater only (blower controlled in drying state)
  relays.set(RELAY_HEATER, false); // PID will control this
  relays.set(RELAY_BLOWER, false); // Will start after head-start period
}

void stopDryingCycle() {
  currentState = STATE_COMPLETED;
  dryingActive = false;
  
  // Turn off everything
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
}

void handleStandbyState() {
  // System ready, waiting for user input
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
}

void handleStartingState() {
  // Brief startup phase
  currentState = STATE_DRYING;
}

void handleDryingState() {
  unsigned long elapsed = millis() - dryingStartTime;
  
  // Check if drying time is over
  if (elapsed >= DRYING_DURATION) {
    stopDryingCycle();
    return;
  }
  
  // Phase 1 (0-2 minutes): Heater only (head start)
  // Phase 2 (2-6 minutes): Heater + Blower
  
  if (elapsed < HEATER_HEADSTART_DURATION) {
    // Phase 1: Heater-only head start
    relays.set(RELAY_BLOWER, false);
  } else {
    // Phase 2: Heater + Blower
    relays.set(RELAY_BLOWER, true);
  }
  
  // PID controls heater in both phases (will be updated in updateSensorsAndSafety)
}

void handleCompletedState() {
  // Cycle complete, stay here until reset or new cycle
}

void handleErrorState() {
  // Error state
  relays.set(RELAY_HEATER, false);
  relays.set(RELAY_BLOWER, false);
  
  static unsigned long errorTime = millis();
  
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
  
  // Require manual reset
  if (buttons.getButtonPressed(0) && buttons.getButtonPressed(1)) {
    emergencyStop = false;
    currentState = STATE_STANDBY;
  }
}

void updateDisplay() {
  float temp = dht.getTemperature();
  float hum = dht.getHumidity();
  
  // Handle invalid sensor readings
  if (temp == -1) temp = 0.0;
  if (hum == -1) hum = 0.0;
  
  // Check if showing setpoint adjustment screen
  if (showingSetpoint) {
    displaySetpointAdjustment();
    return;
  }
  
  // Only clear and redraw if state changed
  bool stateChanged = (currentState != lastDisplayedState);
  if (stateChanged) {
    lcd.clear();
    lastDisplayedState = currentState;
    lastDisplayedTemp = -999;
    lastDisplayedHum = -999;
    lastDisplayedMinutes = -1;
    lastDisplayedSeconds = -1;
  }
  
  switch (currentState) {
    case STATE_STANDBY:
      if (stateChanged) {
        lcd.setText("  UMBRELLA DRYER  ", 0, 0);
        lcd.setText("Temp:       ", 0, 1);
        lcd.setText((char)223, 11, 1); // Degree symbol
        lcd.setText("C", 12, 1);
        lcd.setText("Humi:       %", 0, 2);
        lcd.setText("Set:    C BTN1:GO", 0, 3);
        lcd.setText((char)223, 7, 3); // Degree symbol in Set temp
      }
      
      // Update current temperature if changed (>0.2°C difference to prevent jitter)
      if (abs(temp - lastDisplayedTemp) > 0.2) {
        lcd.setTextPadded(temp, 6, 1, 5, 1);
        lastDisplayedTemp = temp;
      }
      
      // Update humidity if changed (>0.5% difference)
      if (abs(hum - lastDisplayedHum) > 0.5) {
        lcd.setTextPadded(hum, 6, 2, 5, 1);
        lastDisplayedHum = hum;
      }
      
      // Always update target temperature display
      lcd.setTextPadded((int)targetTemperature, 4, 3, 3);
      break;
      
    case STATE_STARTING:
      if (stateChanged) {
        lcd.setText("  UMBRELLA DRYER  ", 0, 0);
        lcd.setText("  *** STARTING ***", 0, 1);
        lcd.setText("                    ", 0, 2);
        lcd.setText("  Please wait...  ", 0, 3);
      }
      break;
      
    case STATE_DRYING:
      {
        unsigned long elapsed = millis() - dryingStartTime;
        unsigned long remaining = (DRYING_DURATION > elapsed) ? (DRYING_DURATION - elapsed) : 0;
        int minutes = remaining / 60000;
        int seconds = (remaining % 60000) / 1000;
        
        if (stateChanged) {
          lcd.setText("  *** DRYING ***  ", 0, 0);
          lcd.setText("Time:   :  ", 0, 1);
          lcd.setText("Temp:       ", 0, 2);
          lcd.setText((char)223, 11, 2); // Degree symbol
          lcd.setText("C", 12, 2);
          lcd.setText("Humi:       %", 0, 3);
        }
        
        // Update timer if changed
        if (minutes != lastDisplayedMinutes || seconds != lastDisplayedSeconds) {
          // Build timer string
          String timeStr = "";
          if (minutes < 10) timeStr += "0";
          timeStr += String(minutes);
          timeStr += ":";
          if (seconds < 10) timeStr += "0";
          timeStr += String(seconds);
          lcd.setText(timeStr, 6, 1);
          lastDisplayedMinutes = minutes;
          lastDisplayedSeconds = seconds;
        }
        
        // Update temperature if changed
        if (abs(temp - lastDisplayedTemp) > 0.2) {
          lcd.setTextPadded(temp, 6, 2, 5, 1);
          lastDisplayedTemp = temp;
        }
        
        // Update humidity if changed
        if (abs(hum - lastDisplayedHum) > 0.5) {
          lcd.setTextPadded(hum, 6, 3, 5, 1);
          lastDisplayedHum = hum;
        }
      }
      break;
      
    case STATE_COMPLETED:
      if (stateChanged) {
        lcd.setText("  UMBRELLA DRYER  ", 0, 0);
        lcd.setText(" CYCLE COMPLETE!  ", 0, 1);
        lcd.setText("                    ", 0, 2);
        lcd.setText("Press BTN1 for new", 0, 3);
      }
      break;
      
    case STATE_ERROR:
      if (stateChanged) {
        lcd.setText("  UMBRELLA DRYER  ", 0, 0);
        lcd.setText("  ** ERROR **     ", 0, 1);
        lcd.setText(" Check sensors!   ", 0, 2);
        lcd.setText("Auto-reset in 10s", 0, 3);
      }
      break;
      
    case STATE_EMERGENCY_STOP:
      if (stateChanged) {
        lcd.setText("  UMBRELLA DRYER  ", 0, 0);
        lcd.setText(" EMERGENCY STOP!  ", 0, 1);
        lcd.setText("                    ", 0, 2);
        lcd.setText("BTN1+BTN2 to reset", 0, 3);
      }
      break;
  }
}

void displaySetpointAdjustment() {
  // Show setpoint adjustment screen
  static bool screenInitialized = false;
  
  if (!screenInitialized) {
    lcd.clear();
    lcd.setText("  UMBRELLA DRYER  ", 0, 0);
    lcd.setText("                    ", 0, 1);
    lcd.setText("  TARGET TEMP:    ", 0, 2);
    lcd.setText("                    ", 0, 3);
    screenInitialized = true;
  }
  
  // Display the setpoint value (centered)
  String tempStr = "     " + String((int)targetTemperature) + " ";
  tempStr += (char)223; // Degree symbol
  tempStr += "C     ";
  lcd.setText(tempStr, 0, 2);
  
  // Show countdown or instruction
  unsigned long elapsed = millis() - setpointAdjustTime;
  unsigned long remaining = (SETPOINT_DISPLAY_DURATION - elapsed) / 1000;
  String instruction = "BTN3:+ BTN4:- (" + String((int)remaining + 1) + "s)";
  
  // Center the instruction
  int padding = (20 - instruction.length()) / 2;
  String paddedInstruction = "";
  for (int i = 0; i < padding; i++) paddedInstruction += " ";
  paddedInstruction += instruction;
  while (paddedInstruction.length() < 20) paddedInstruction += " ";
  
  lcd.setText(paddedInstruction, 0, 3);
  
  // Reset screen initialized flag when leaving setpoint mode
  if (!showingSetpoint) {
    screenInitialized = false;
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
  } else if (strcmp_P(command, PSTR("test_relay")) == 0) {
    testRelayModule();
  } else if (strcmp_P(command, PSTR("test_button")) == 0) {
    testTactileButtons();
  } else if (strcmp_P(command, PSTR("test_pid")) == 0) {
    testPIDController();
  } else if (strcmp_P(command, PSTR("test_limit")) == 0) {
    testLimitSwitch();
  } else if (strcmp_P(command, PSTR("test_ir")) == 0) {
    testIRSensor();
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
  Serial.println(F("test_relay - Test relays"));
  Serial.println(F("test_button - Test buttons"));
  Serial.println(F("test_pid - Test PID"));
  Serial.println(F("test_limit - Test limit switch"));
  Serial.println(F("test_ir - Test IR sensor"));
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
  Serial.print(F("RELAY: ")); Serial.println(systemStatus.relaysOk ? F("OK") : F("ERR"));
  Serial.print(F("BTN: ")); Serial.println(systemStatus.buttonsOk ? F("OK") : F("ERR"));
  Serial.print(F("PID: ")); Serial.println(systemStatus.pidOk ? F("OK") : F("ERR"));
  Serial.print(F("LIMIT: ")); Serial.println(systemStatus.limitOk ? F("OK") : F("ERR"));
  Serial.print(F("IR: ")); Serial.println(systemStatus.irOk ? F("OK") : F("ERR"));
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
      if (digitalRead(buttons.inputPins[i]) == LOW) {  // LOW = pressed with pull-up
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

void testLimitSwitch() {
  limitSwitch.update();
  bool closed = limitSwitch.isDoorClosed();
  bool open = limitSwitch.isDoorOpen();
  
  Serial.print(F("Limit - Closed:"));
  Serial.print(closed ? F("YES") : F("NO"));
  Serial.print(F(" Open:"));
  Serial.println(open ? F("YES") : F("NO"));
  
  systemStatus.limitOk = true; // Assuming it's working if we can read it
}

void testIRSensor() {
  irSensor.update();
  bool detected = irSensor.isUmbrellaDetected();
  
  Serial.print(F("IR - Umbrella:"));
  Serial.println(detected ? F("DETECTED") : F("NOT DETECTED"));
  
  systemStatus.irOk = true; // Assuming it's working if we can read it
}



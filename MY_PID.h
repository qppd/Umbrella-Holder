#include <PID_v1.h>

double THRESHOLD_1 = 4;
double THRESHOLD_2 = 0;
double THRESHOLD_3 = 22;

double PID_OUTPUT = 0;
double CURRENT_TEMPERATURE = 0;
double HEATING_TEMPERATURE_SETPOINT = 60;

PID pidHeating(&CURRENT_TEMPERATURE, &PID_OUTPUT, &HEATING_TEMPERATURE_SETPOINT, THRESHOLD_1, THRESHOLD_2, THRESHOLD_3, DIRECT);

void initPID() {
  pidHeating.SetMode(AUTOMATIC);
  pidHeating.SetOutputLimits(0, 5000);
}

void pidCOMPUTE() {

  pidHeating.Compute();

  if (PID_OUTPUT > 0) {
    Serial.println("Opening Heater");
  } else {
    Serial.println("Closing Heater");
  }
}

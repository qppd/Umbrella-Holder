#include "PidController.h"

// Forward declaration for SYSTEM_MODE
#ifndef SYSTEM_MODE
#define MODE_DEVELOPMENT 1
#define MODE_PRODUCTION 2
#define SYSTEM_MODE MODE_DEVELOPMENT
#endif

PidController::PidController(double kp, double ki, double kd, double setpoint)
    : kp(kp), ki(ki), kd(kd), output(0), currentTemperature(0), setpoint(setpoint),
      pid(&currentTemperature, &output, &this->setpoint, kp, ki, kd, DIRECT) {}

void PidController::init() {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 255); // 0-255 for PWM-style control
}

void PidController::compute() {
    pid.Compute();
    // Remove debug output in production mode
    #if SYSTEM_MODE == MODE_DEVELOPMENT
    if (output > 0) {  // For 0-1 normalized output range
        Serial.println("Heater Relay ON");
    } else {
        Serial.println("Heater Relay OFF");
    }
    #endif
}

void PidController::setCurrentTemperature(double temp) {
    currentTemperature = temp;
}

double PidController::getOutput() const {
    return output;
}

void PidController::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

#include "PidController.h"

PidController::PidController(double kp, double ki, double kd, double setpoint)
    : kp(kp), ki(ki), kd(kd), output(0), currentTemperature(0), setpoint(setpoint),
      pid(&currentTemperature, &output, &this->setpoint, kp, ki, kd, DIRECT) {}

void PidController::init() {
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(0, 5000);
}

void PidController::compute() {
    pid.Compute();
    if (output > 0) {
        Serial.println("Opening Heater");
    } else {
        Serial.println("Closing Heater");
    }
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

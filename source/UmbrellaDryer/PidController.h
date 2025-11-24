#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>
#include <PID_v1.h>

class PidController {
public:
    PidController(double kp = 4, double ki = 0, double kd = 22, double setpoint = 60);
    void init();
    void compute();
    void setCurrentTemperature(double temp);
    double getOutput() const;
    void setSetpoint(double setpoint);
    bool isHeatingRequired() const; // Returns true if PID output > 0
private:
    double kp, ki, kd;
    double output;
    double currentTemperature;
    double setpoint;
    PID pid;
};

#endif // PID_CONTROLLER_H

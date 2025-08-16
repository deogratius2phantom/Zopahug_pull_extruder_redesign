#ifndef EXTRUDER_CONTROL_H
#define EXTRUDER_CONTROL_H

// Include necessary libraries
#include <Arduino.h>
#include <pins.h>
#include <thermistor.h>
#include <pid_tunner.h>

// Define constants
class Extruder {
public:
    Extruder(int extruderHotEndPin, int tempSensorPin, int extruderIndex, double kp, double ki, double kd);
    void initialize();
    void setTemperature(double temperature);
    float readTemperature();
    void tunePID(double tuner_set_point);
    void extrude(bool state);
    bool readFilamentSensor();
    void setFilamentSensorPin(int pin);
    void disableHeater();
    void checkStability( double currentTemperature, unsigned long& stableStartTime, bool& isStable);
    PID myPID;
    // Function prototypes
private:
    int extruderHotEndPin;
    int tempSensorPin;
    int filamentSensorPin;
    double currentTemperature;
    double currentMotorTemperature;
    double currentMotorControllerTemperature;
    int extruderIndex;
    volatile double input;
    volatile double  output;
    volatile double setpoint;
    double kp;
    double ki;
    double kd;

    int motorStepPin;
    int motorDirectionPin;
    int motorEnablePin;
    
};

#endif // EXTRUDER_CONTROL_H
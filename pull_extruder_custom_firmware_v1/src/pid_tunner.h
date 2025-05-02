#ifndef PID_TUNNER_H
#define PID_TUNNER_H
#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
bool setHotEndTemperature(uint8_t target_temperature);
void tunePID(double setpoint) ;
#endif // PID_TUNNER_H
#include <extruder_control.h>
byte ATuneModeRemember=2;
double tuner_input=80, tuner_output=50, tuner_setpoint=180;
//double kp=38.27,ki=5.38,kd=40.70;
double kpmodel=1.5, taup=100, theta[50];
double outputStart=5;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
boolean tuning = false;
unsigned long  modelTime, serialTime;
PID_ATune aTune(&tuner_input, &tuner_output);
Extruder::Extruder(int extruderHotEndPin, int tempSensorPin, int extruderIndex, double kp, double ki, double kd) 
    :myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT) {
    this->extruderHotEndPin = extruderHotEndPin;
    this->tempSensorPin = tempSensorPin;
    this->extruderIndex = extruderIndex;
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}
void Extruder::setFilamentSensorPin(int pin) {
    filamentSensorPin = pin;
}
void Extruder::initialize() {
    pinMode(extruderHotEndPin, OUTPUT);
    pinMode(filamentSensorPin, INPUT_PULLUP);
    Serial.println("Extruder_"+String(extruderIndex)+" initialized");
    myPID.SetMode(AUTOMATIC);
}
float Extruder::readTemperature() {
    return getTemperature(analogRead(tempSensorPin));
}
void Extruder::setTemperature(double temperature) {
    setpoint = temperature;
    input = getTemperature(analogRead(tempSensorPin));
    myPID.Compute();
    analogWrite(extruderHotEndPin, output);
}
void Extruder::tunePID(double tuner_set_point) {
    input = getTemperature(analogRead(tempSensorPin));
    double bestKp = kp, bestKi = ki, bestKd = kd;
    double bestError = 1e10; // Initialize with a large number
    for (int cycle = 0; cycle < 10; cycle++) {
        double totalError = 0;
        unsigned long startTime = millis();
        unsigned long stableTime = 0;
        const unsigned long stableDuration = 10000; // 10 seconds
        setpoint = tuner_set_point;
        myPID.SetMode(AUTOMATIC);
        while (true) {
            input = getTemperature(analogRead(tempSensorPin));
            myPID.Compute();
            analogWrite(extruderHotEndPin, output);
            Serial.print("temperature: ");
            Serial.println(input);

            totalError += abs(setpoint - input);
            if (abs(setpoint - input) <= 1.0) {
                if (stableTime == 0) {
                    stableTime = millis();
                } else if (millis() - stableTime >= stableDuration) {
                    break;
                }
            } else {
                stableTime = 0;
            }
            if (millis() - startTime > 60000) { // Timeout after 60 seconds
                break;
            }
            delay(100); // Small delay to prevent overwhelming the loop
        }
        if (totalError < bestError) {
            bestError = totalError;
            bestKp = kp;
            bestKi = ki;
            bestKd = kd;
        }
    }
    kp = bestKp;
    ki = bestKi;
    kd = bestKd;
    myPID.SetTunings(kp, ki, kd);
    Serial.print("Best PID parameters: Kp=");
    Serial.print(bestKp);
    Serial.print(", Ki=");
    Serial.print(bestKi);
    Serial.print(", Kd=");
    Serial.println(bestKd);
}
bool Extruder::readFilamentSensor() {
    return digitalRead(filamentSensorPin);
}
void Extruder::disableHeater() {
    analogWrite(extruderHotEndPin, 0);
}

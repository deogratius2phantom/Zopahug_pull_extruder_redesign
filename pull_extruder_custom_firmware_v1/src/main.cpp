#include <Arduino.h>
#include <extruder_control.h>
#include <ArduinoJson.h>
#include <ota.h>
const int TEMPERATURE_ERROR_THRESHOLD = 15;
const unsigned long STABILITY_TIME = 10000; // 5 seconds

volatile unsigned long stepDelay = 2000;  // Initial delay in µs (adjustable)
volatile bool extruder_1_motor_stepState = false;
volatile bool extruder_2_motor_stepState = false;
volatile bool extruder_3_motor_stepState = false;
volatile long stepCount = 0;
volatile float currentSpeed = 1000; // Set to max speed for continuous motion
volatile float maxSpeed = 300; // Max steps per second
volatile float acceleration = 1000; // Steps per second²
int steps_per_millimeter = 800; // Steps per millimeter
int upload_counter=0;
Extruder EXTRUDER_1(EXTRUDER_1_HEATER, EXTRUDER_1_THERMISTOR, 1, 38.50, 4.08, 40.70);
Extruder EXTRUDER_2(EXTRUDER_2_HEATER, EXTRUDER_2_THERMISTOR, 2, 38.50, 4.08, 40.70);
Extruder EXTRUDER_3(EXTRUDER_3_HEATER, EXTRUDER_3_THERMISTOR, 3, 38.50, 4.08, 40.70);
double setpoint_temperature = 225.00;
int revs = 2;
// SpeedyStepper EXTRUDER_1_MOTOR;
// SpeedyStepper EXTRUDER_2_MOTOR;
// SpeedyStepper EXTRUDER_3_MOTOR;

// Add these variables to keep track of stability
unsigned long stableStartTime1 = 0;
unsigned long stableStartTime2 = 0;
unsigned long stableStartTime3 = 0;
bool extruder_1_isStable = false;
bool extruder_2_isStable = false;
bool extruder_3_isStable = false;
volatile bool extruder_1_enable = false;
volatile bool extruder_2_enable = false;
volatile bool extruder_3_enable = false;
bool extruder_1_filament_sensor_state = false;
bool extruder_2_filament_sensor_state = false;
bool extruder_3_filament_sensor_state = false;
int dataToPublish[256] = {};
// Define the states for the state machine
volatile bool extruder_1_heater_enable=false;
volatile bool extruder_2_heater_enable=false;
volatile bool extruder_3_heater_enable=false;
volatile long extruder_1_motor_steps = 0;
volatile long extruder_2_motor_steps = 0;
volatile long extruder_3_motor_steps = 0;
enum State {
    WAITING_FOR_STABILITY,
    ALL_STABLE,
    MOVING_MOTOR
};

State state1 = WAITING_FOR_STABILITY;
State state2 = WAITING_FOR_STABILITY;
State state3 = WAITING_FOR_STABILITY;
String jsonOutput;
void setMotorSpeed(float speed);
void checkStability(Extruder& extruder, double setpoint, unsigned long& stableStartTime, bool& isStable);
void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    EXTRUDER_1.initialize();
    EXTRUDER_2.initialize();
    EXTRUDER_3.initialize();
    pinMode(EXTRUDER_1_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_2_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_3_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_1_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_1_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_2_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_2_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_3_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_3_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_1_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_2_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_3_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_1_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(EXTRUDER_2_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(EXTRUDER_3_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    // disable motor drivers on startup
    digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, HIGH);
    digitalWrite(EXTRUDER_2_MOTOR_DRIVER_ENABLE, HIGH);
    digitalWrite(EXTRUDER_3_MOTOR_DRIVER_ENABLE, HIGH);
    Serial.print("Setpoint temperature: ");
    Serial.println(setpoint_temperature);
    cli();
    //timer 1 setup to control extruder1 
    TCCR1A = 0;
    TCCR1B = (1 << WGM12); // CTC mode
    TCCR1B |= (1 << CS11); // Prescaler 8
    OCR1A = 1999; // Adjusted for 10ms interval (16 MHz / 8 / 100 Hz - 1)
    TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt
    // Timer 2 setup for 50ms interval
    TCCR2A = 0;  // Normal operation
    TCCR2B = (1 << WGM21);  // CTC mode (Clear Timer on Compare Match)
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  // Prescaler 1024
    OCR2A = 31249;  // Adjusted for 2 seconds interval (16 MHz / 1024 / 0.5 Hz - 1)
    TIMSK2 |= (1 << OCIE2A);  // Enable Timer2 compare interrupt
    //timer 3 setup to control extruder2
    TCCR3A = 0;  // Normal operation
    TCCR3B = (1 << WGM32);  // CTC mode (Clear Timer on Compare Match)
    TCCR3B |= (1 << CS31);  // Prescaler 8
    OCR3A = 2999;  // Adjusted for 15ms interval (16 MHz / 8 / 66.67 Hz - 1)
    TIMSK3 |= (1 << OCIE3A);  // Enable Timer3 compare interrupt
    // Timer 4 setup to control stepper motors
    TCCR4A = 0;  // Normal operation
    TCCR4B = (1 << WGM42);  // CTC mode (Clear Timer on Compare Match)
    TCCR4B |= (1 << CS41);  // Prescaler 8
    OCR4A = 3999;  // Adjusted for desired interval
    TIMSK4 |= (1 << OCIE4A);  // Enable Timer4 compare interrupt   
    //timer 5 setup to control extruder3
    TCCR5A = 0;
    TCCR5B = (1 << WGM52); // CTC mode
    TCCR5B |= (1 << CS51); // Prescaler 8
    OCR5A = 3999; // Adjusted for 20ms interval
    TIMSK5 |= (1 << OCIE5A); // Enable Timer5 compare interrupt


    sei();

    setMotorSpeed(currentSpeed);
    //startGsm(-1);
    //checkForUpdate("2868510");

}
ISR(TIMER1_COMPA_vect) {
    extruder_1_enable = digitalRead(EXTRUDER_1_ENABLE_SWITCH);
    extruder_1_filament_sensor_state=!digitalRead(EXTRUDER_1_FILAMENT_SENSOR);
    if (extruder_1_enable==true ) {
        EXTRUDER_1.setTemperature(setpoint_temperature);
        extruder_1_heater_enable=true;
    }
    else
    {
        extruder_1_heater_enable=false;
        EXTRUDER_1.disableHeater();
        digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, HIGH); // Disable extruder_1 motor driver
    }

}
ISR(TIMER2_COMPA_vect)
{

    
   //
    
}
ISR(TIMER3_COMPA_vect) {
    extruder_2_enable = digitalRead(EXTRUDER_2_ENABLE_SWITCH);  
    extruder_2_filament_sensor_state=!digitalRead(EXTRUDER_2_FILAMENT_SENSOR);
    if (extruder_2_enable==true) {
        EXTRUDER_2.setTemperature(setpoint_temperature);
        extruder_2_heater_enable=true;
    }
    else
    {
        extruder_2_heater_enable=false;
        EXTRUDER_2.disableHeater();
        digitalWrite(EXTRUDER_2_MOTOR_DRIVER_ENABLE, HIGH); // Disable extruder_2 motor driver
    }
}
ISR(TIMER4_COMPA_vect) {
    if (extruder_1_enable && extruder_1_isStable && extruder_1_filament_sensor_state) {
        digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, LOW); // Enable extruder_1 motor driver
        digitalWrite(EXTRUDER_1_MOTOR_DRIVER_STEP, extruder_1_motor_stepState); // Set step pin to stepState
        extruder_1_motor_stepState = !extruder_1_motor_stepState; // Toggle step state
        if (extruder_1_motor_stepState) {
            //extruder_1_motor_steps++; // Increment step count
        }
    }

    if (extruder_2_enable && extruder_2_isStable && extruder_2_filament_sensor_state) {
        digitalWrite(EXTRUDER_2_MOTOR_DRIVER_DIR, HIGH); // Set direction for extruder_2
        digitalWrite(EXTRUDER_2_MOTOR_DRIVER_ENABLE, LOW); // Enable extruder_2 motor driver
        digitalWrite(EXTRUDER_2_MOTOR_DRIVER_STEP, extruder_2_motor_stepState); // Set step pin to stepState
        extruder_2_motor_stepState = !extruder_2_motor_stepState; // Toggle step state
        if (extruder_2_motor_stepState) {
            //extruder_2_motor_steps++; // Increment step count
        }
    }

    if (extruder_3_enable && extruder_3_isStable && extruder_3_filament_sensor_state) {
        digitalWrite(EXTRUDER_3_MOTOR_DRIVER_DIR, HIGH); // Set direction for extruder_3
        digitalWrite(EXTRUDER_3_MOTOR_DRIVER_ENABLE, LOW); // Enable extruder_3 motor driver
        digitalWrite(EXTRUDER_3_MOTOR_DRIVER_STEP, extruder_3_motor_stepState); // Set step pin to stepState
        extruder_3_motor_stepState = !extruder_3_motor_stepState; // Toggle step state
        if (extruder_3_motor_stepState) {
            //extruder_3_motor_steps++; // Increment step count
        }
    }
}
ISR(TIMER5_COMPA_vect) {
    extruder_3_enable = digitalRead(EXTRUDER_3_ENABLE_SWITCH);
    extruder_3_filament_sensor_state=!digitalRead(EXTRUDER_3_FILAMENT_SENSOR);
    if (extruder_3_enable==true) {
        EXTRUDER_3.setTemperature(setpoint_temperature);
        extruder_3_heater_enable=true;
    }
    else
    {
        extruder_3_heater_enable=false;
        EXTRUDER_3.disableHeater();
        digitalWrite(EXTRUDER_3_MOTOR_DRIVER_ENABLE, HIGH); // Disable extruder_3 motor driver
    }
    // upload_counter++;

}

void setMotorSpeed(float speed) {
    currentSpeed = speed;
    stepDelay = 1000000.0 / currentSpeed; // Convert speed to delay (µs)
    OCR4A = stepDelay; // Adjust timer
  }
void loop() {
    // Read temperatures
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 2000) {
        double temp1 = EXTRUDER_1.readTemperature();
        double temp2 = EXTRUDER_2.readTemperature();
        double temp3 = EXTRUDER_3.readTemperature();

        // Create JSON formatted string using ArduinoJson
        DynamicJsonDocument doc(1024);
       // StaticJsonDocument<1024> doc;
        JsonObject extruder1 = doc.createNestedObject("Extruder1");
        extruder1["Temperature"] = temp1;
        extruder1["isEnabled"] = extruder_1_enable;
        extruder1["heater enable"] = extruder_1_heater_enable;
        extruder1["filament presence"] = extruder_1_filament_sensor_state;
        extruder1["IsStable"] = extruder_1_isStable;
        extruder1["Steps"] = extruder_1_motor_steps;
        extruder1["extruded filament length"] = extruder_1_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;

        JsonObject extruder2 = doc.createNestedObject("Extruder2");
        extruder2["Temperature"] = temp2;
        extruder2["isEnabled"] = extruder_2_enable;
        extruder2["heater enable"] = extruder_2_heater_enable;
        extruder2["filament presence"] = extruder_2_filament_sensor_state;
        extruder2["IsStable"] = extruder_2_isStable;
        extruder2["Steps"] = extruder_2_motor_steps;
        extruder2["extruded filament length"] = extruder_2_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;

        JsonObject extruder3 = doc.createNestedObject("Extruder3");
        extruder3["Temperature"] = temp3;
        extruder3["isEnabled"] = extruder_3_enable;
        extruder3["heater enable"] = extruder_3_heater_enable;
        extruder3["filament presence"] = extruder_3_filament_sensor_state;
        extruder3["IsStable"] = extruder_3_isStable;
        extruder3["Steps"] = extruder_3_motor_steps;
        extruder3["extruded filament length"] = extruder_3_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;

        
        serializeJsonPretty(doc, jsonOutput);
        dataToPublish[0] =  extruder1["isEnabled"];
        dataToPublish[1] =  extruder2["isEnabled"];
        dataToPublish[2] =  extruder3["isEnabled"];
        dataToPublish[3] =  extruder1["Temperature"];
        dataToPublish[4] =  extruder2["Temperature"];
        dataToPublish[5] =  extruder3["Temperature"];
        dataToPublish[6] =  extruder1["filament presence"];
        dataToPublish[7] =  extruder2["filament presence"];
        dataToPublish[8] =  extruder3["filament presence"];
        dataToPublish[9] =  extruder1["IsStable"];
        dataToPublish[10] = extruder2["IsStable"];
        dataToPublish[11] = extruder3["IsStable"];
        dataToPublish[12] = extruder1["Steps"];
        dataToPublish[13] = extruder2["Steps"];
        dataToPublish[14] = extruder3["Steps"];
        dataToPublish[15] = extruder1["extruded filament length"];
        dataToPublish[16] = extruder2["extruded filament length"];
        dataToPublish[17] = extruder3["extruded filament length"];
        // Print JSON formatted string
        Serial.println(jsonOutput);
        doc.clear();
        //upload_counter++;
        lastPrintTime = currentTime;

    }
    if(upload_counter>=30)
    {
        //checkForUpdate("2868510");
        //mqttUpdate("2868510",dataToPublish);
        //upload_counter=0;
    }

    // Check stability for each extruder
    if (!extruder_1_enable) {
        extruder_1_isStable = false;
    } else {
        checkStability(EXTRUDER_1, setpoint_temperature, stableStartTime1, extruder_1_isStable);
    }

    if (!extruder_2_enable) {
        extruder_2_isStable = false;
    } else {
        checkStability(EXTRUDER_2, setpoint_temperature, stableStartTime2, extruder_2_isStable);
    }

    if (!extruder_3_enable) {
        extruder_3_isStable = false;
    } else {
        checkStability(EXTRUDER_3, setpoint_temperature, stableStartTime3, extruder_3_isStable);
    }
    
     
   
}

void checkStability(Extruder& extruder, double setpoint, unsigned long& stableStartTime, bool& isStable) {
    double currentTemp = extruder.readTemperature();
    if (abs(currentTemp - setpoint) <= TEMPERATURE_ERROR_THRESHOLD) {
        if (!isStable) {
            stableStartTime = millis();
            isStable = true;
        } else if (millis() - stableStartTime >= STABILITY_TIME) {
            isStable = true;
        }
    } else {
        isStable = false;
    }
}

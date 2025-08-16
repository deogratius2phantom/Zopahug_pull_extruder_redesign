#include <Arduino.h>
#include <extruder_control.h>
#include <ArduinoJson.h>
#include <ota.h>
const int TEMPERATURE_ERROR_THRESHOLD = 10;
const unsigned long STABILITY_TIME = 10000; // 10 seconds

volatile unsigned long stepDelay = 2000;  // Initial delay in µs (adjustable)
volatile bool extruder_1_motor_stepState = false;
volatile bool extruder_2_motor_stepState = false;
volatile bool extruder_3_motor_stepState = false;
volatile bool extruder_4_motor_stepState = false;
volatile bool extruder_5_motor_stepState = false;
volatile long stepCount = 0;
volatile float currentSpeed = 1000; // Set to max speed for continuous motion
volatile float maxSpeed = 300; // Max steps per second
volatile float acceleration = 1000; // Steps per second²
int steps_per_millimeter = 800; // Steps per millimeter
int upload_counter=0;
Extruder EXTRUDER_1(EXTRUDER_1_HEATER, EXTRUDER_1_THERMISTOR, 1, 35.50, 4.08, 40.70);
Extruder EXTRUDER_2(EXTRUDER_2_HEATER, EXTRUDER_2_THERMISTOR, 2, 35.50, 4.08, 40.70);
Extruder EXTRUDER_3(EXTRUDER_3_HEATER, EXTRUDER_3_THERMISTOR, 3, 35.50, 4.08, 40.70);
Extruder EXTRUDER_4(EXTRUDER_4_HEATER, EXTRUDER_4_THERMISTOR, 4, 35.50, 4.08, 40.70);
Extruder EXTRUDER_5(EXTRUDER_5_HEATER, EXTRUDER_5_THERMISTOR, 5, 35.50, 4.08, 40.70);
double setpoint_temperature = 225.00;
int revs = 2;

// Add these variables to keep track of stability
unsigned long stableStartTime1 = 0;
unsigned long stableStartTime2 = 0;
unsigned long stableStartTime3 = 0;
unsigned long stableStartTime4 = 0;
unsigned long stableStartTime5 = 0;

static bool extruder_1_isStable = false;
static bool extruder_2_isStable = false;
static bool extruder_3_isStable = false;
static bool extruder_4_isStable = false;
static bool extruder_5_isStable = false;

volatile bool extruder_1_enable = false;
volatile bool extruder_2_enable = false;
volatile bool extruder_3_enable = false;
volatile bool extruder_4_enable = false;
volatile bool extruder_5_enable = false;
bool extruder_1_filament_sensor_state = false;
bool extruder_2_filament_sensor_state = false;
bool extruder_3_filament_sensor_state = false;
bool extruder_4_filament_sensor_state = false;
bool extruder_5_filament_sensor_state = false;
int dataToPublish[256] = {};
// Define the states for the state machine
volatile bool extruder_1_heater_enable=false;
volatile bool extruder_2_heater_enable=false;
volatile bool extruder_3_heater_enable=false;
volatile bool extruder_4_heater_enable=false;
volatile bool extruder_5_heater_enable=false;
volatile long extruder_1_motor_steps = 0;
volatile long extruder_2_motor_steps = 0;
volatile long extruder_3_motor_steps = 0;
volatile long extruder_4_motor_steps = 0;
volatile long extruder_5_motor_steps = 0;
enum State {
    WAITING_FOR_STABILITY,
    ALL_STABLE,
    MOVING_MOTOR
};

State state1 = WAITING_FOR_STABILITY;
State state2 = WAITING_FOR_STABILITY;
State state3 = WAITING_FOR_STABILITY;
State state4 = WAITING_FOR_STABILITY;
State state5 = WAITING_FOR_STABILITY;
String jsonOutput;
void setMotorSpeed(float speed);
void checkStability(Extruder& extruder, double setpoint, unsigned long& stableStartTime, bool& isStable);
void setupTimer1() {
    noInterrupts(); // Disable interrupts during setup

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // Normal mode
    // Prescaler = 64 (1 tick = 4 µs at 16 MHz)
    TCCR1B |= (1 << CS11) | (1 << CS10);

    // Initial compare match values
    OCR1A = TCNT1 + 2500;   // 10 ms
    OCR1B = TCNT1 + 5000;   // 20 ms
    OCR1C = TCNT1 + 7500;   // 30 ms

    // Enable compare match interrupts
    TIMSK1 |= (1 << OCIE1A);
    TIMSK1 |= (1 << OCIE1B);
    TIMSK1 |= (1 << OCIE1C);

    interrupts(); // Re-enable interrupts
}

void setupTimer3() {
    noInterrupts(); // Disable interrupts during setup

    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3  = 0;

    // Normal mode
    // Prescaler = 64 (1 tick = 4 µs at 16 MHz)
    TCCR3B |= (1 << CS31) | (1 << CS30);

    // Initial compare match values
    OCR3A = TCNT3 + 2500;   // 10 ms
    OCR3B = TCNT3 + 5000;   // 20 ms
    OCR3C = TCNT3 + 7500;   // 30 ms

    // Enable compare match interrupts
    TIMSK3 |= (1 << OCIE3A);
    TIMSK3 |= (1 << OCIE3B);
    TIMSK3 |= (1 << OCIE3C);

    interrupts(); // Re-enable interrupts
}

void setupTimer4() {
    noInterrupts(); // Disable interrupts during setup
    TCCR4A = 0;
    TCCR4B = 0;
    TCCR4B |= (1 << WGM42); // CTC mode
    TCCR4B |= (1 << CS41) ; // Prescaler = 64 (1 tick = 4 µs at 16 MHz)
    OCR4A = 1000; // Set compare match value for 10 ms interval (2500 ticks)
    TIMSK4 |= (1 << OCIE4A); // Enable Timer4 compare match interrupt
    interrupts(); // Re-enable interrupts
}
void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    EXTRUDER_1.initialize();
    EXTRUDER_2.initialize();
    EXTRUDER_3.initialize();
    EXTRUDER_4.initialize();
    EXTRUDER_5.initialize();
    pinMode(EXTRUDER_1_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_2_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_3_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_4_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_5_MOTOR_DRIVER_ENABLE, OUTPUT);
    pinMode(EXTRUDER_1_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_1_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_2_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_2_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_3_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_3_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_4_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_4_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_5_MOTOR_DRIVER_STEP, OUTPUT);
    pinMode(EXTRUDER_5_MOTOR_DRIVER_DIR, OUTPUT);
    pinMode(EXTRUDER_1_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_2_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_3_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_4_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_5_FILAMENT_SENSOR, INPUT_PULLUP);
    pinMode(EXTRUDER_1_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(EXTRUDER_2_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(EXTRUDER_3_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(EXTRUDER_4_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(EXTRUDER_5_ENABLE_SWITCH, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    pinMode(EXTRUDER_1_ENABLE_LED, OUTPUT);
    pinMode(49, OUTPUT);
    pinMode(42, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(40, OUTPUT);
    pinMode(27, OUTPUT);
    pinMode(25, OUTPUT);
    // Set all these pins to logic LOW
    // disable motor drivers on startup
    digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, HIGH);
    digitalWrite(EXTRUDER_2_MOTOR_DRIVER_ENABLE, HIGH);
    digitalWrite(EXTRUDER_3_MOTOR_DRIVER_ENABLE, HIGH);
    digitalWrite(EXTRUDER_4_MOTOR_DRIVER_ENABLE, HIGH);
    digitalWrite(EXTRUDER_5_MOTOR_DRIVER_ENABLE, HIGH);

    digitalWrite(EXTRUDER_2_MOTOR_DRIVER_DIR, HIGH);
    digitalWrite(EXTRUDER_4_MOTOR_DRIVER_DIR, HIGH);
    digitalWrite(EXTRUDER_5_MOTOR_DRIVER_DIR, HIGH);

    Serial.print("Setpoint temperature: ");
    Serial.println(setpoint_temperature);
    cli();
    setupTimer1();
    setupTimer3();
    setupTimer4();
    // Timer 4 setup to control stepper motor

    sei();

    setMotorSpeed(currentSpeed);
    //startGsm(-1);
    //checkForUpdate("2868510");

}
// Timer1 register A interrupt service routine to read all control switches
ISR(TIMER1_COMPA_vect) {
    //read all control switches
    extruder_1_enable = digitalRead(EXTRUDER_1_ENABLE_SWITCH) == LOW;
    extruder_2_enable = digitalRead(EXTRUDER_2_ENABLE_SWITCH) == LOW;
    extruder_3_enable = digitalRead(EXTRUDER_3_ENABLE_SWITCH) == LOW;
    extruder_4_enable = digitalRead(EXTRUDER_4_ENABLE_SWITCH) == LOW;
    extruder_5_enable = digitalRead(EXTRUDER_5_ENABLE_SWITCH) == LOW;
    extruder_1_filament_sensor_state = digitalRead(EXTRUDER_1_FILAMENT_SENSOR) == LOW;
    extruder_2_filament_sensor_state = digitalRead(EXTRUDER_2_FILAMENT_SENSOR) == LOW;
    extruder_3_filament_sensor_state = digitalRead(EXTRUDER_3_FILAMENT_SENSOR) == LOW;
    extruder_4_filament_sensor_state = digitalRead(EXTRUDER_4_FILAMENT_SENSOR) == LOW;
    extruder_5_filament_sensor_state = digitalRead(EXTRUDER_5_FILAMENT_SENSOR) == LOW;
}
// Timer1 register B interrupt service routine to control extruder 1 and 2 heating
ISR(TIMER1_COMPB_vect) {
    if(extruder_1_enable){
        EXTRUDER_1.setTemperature(setpoint_temperature);
        extruder_1_heater_enable = true;
    }
    else{
        EXTRUDER_1.disableHeater();
        extruder_1_heater_enable = false;
    }
  
}
// Timer1 register C interrupt service routine to control extruder 2 heating
ISR(TIMER1_COMPC_vect) {
    if(extruder_2_enable){
    EXTRUDER_2.setTemperature(setpoint_temperature);
    extruder_2_heater_enable = true;
    }
    else{
        EXTRUDER_2.disableHeater();
        extruder_2_heater_enable = false;
    }
}
// Timer2 interrupt service routine to read temperatures and check stability
ISR(TIMER2_COMPA_vect)
{
  
}
// Timer3 register A interrupt service routine to control extruder 3 heating
ISR(TIMER3_COMPA_vect) {
    if(extruder_3_enable) {
        EXTRUDER_3.setTemperature(setpoint_temperature);
        extruder_3_heater_enable = true;
    } else {
        EXTRUDER_3.disableHeater();
        extruder_3_heater_enable = false;
    }
   
}
// Timer3 register B interrupt service routine to control extruder 4 heating
ISR(TIMER3_COMPB_vect){
    if(extruder_4_enable) {
        EXTRUDER_4.setTemperature(setpoint_temperature);
        extruder_4_heater_enable = true;
    } else {
        EXTRUDER_4.disableHeater();
        extruder_4_heater_enable = false;
    }

}
// Timer3 register C interrupt service routine to control extruder 5 heating
ISR(TIMER3_COMPC_vect){
    if(extruder_5_enable) {
        EXTRUDER_5.setTemperature(setpoint_temperature);
        extruder_5_heater_enable = true;
    } else {
        EXTRUDER_5.disableHeater();
        extruder_5_heater_enable = false;
    }

}
// Timer4 interrupt service routine to control stepper motors
ISR(TIMER4_COMPA_vect) {
    if(extruder_1_isStable&&extruder_1_filament_sensor_state==true){
        digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, LOW); // Enable motor driver for extruder 1
        extruder_1_motor_stepState = !extruder_1_motor_stepState;
        digitalWrite(EXTRUDER_1_MOTOR_DRIVER_STEP, extruder_1_motor_stepState);
    }
    else {
        digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, HIGH); // Disable motor driver if not stable or no filament
        extruder_1_motor_stepState = false; // Disable stepper motor if not stable or no filament
    }
    if(extruder_2_isStable&&extruder_2_filament_sensor_state==true){
        digitalWrite(EXTRUDER_2_MOTOR_DRIVER_ENABLE, LOW); // Enable motor driver for extruder 2
        extruder_2_motor_stepState = !extruder_2_motor_stepState;
        digitalWrite(EXTRUDER_2_MOTOR_DRIVER_STEP, extruder_2_motor_stepState);
    }
    else {
        digitalWrite(EXTRUDER_2_MOTOR_DRIVER_ENABLE, HIGH); // Disable motor driver if not stable or no filament
        extruder_2_motor_stepState = false; // Disable stepper motor if not stable or no filament
    }
    if(extruder_3_isStable&&extruder_3_filament_sensor_state==true){
        digitalWrite(EXTRUDER_3_MOTOR_DRIVER_ENABLE, LOW); // Enable motor driver for extruder 3
        extruder_3_motor_stepState = !extruder_3_motor_stepState;
        digitalWrite(EXTRUDER_3_MOTOR_DRIVER_STEP, extruder_3_motor_stepState);
    }
    else {
        digitalWrite(EXTRUDER_3_MOTOR_DRIVER_ENABLE, HIGH); // Disable motor driver if not stable or no filament
        extruder_3_motor_stepState = false; // Disable stepper motor if not stable or no filament
    }
    if(extruder_4_isStable&&extruder_4_filament_sensor_state==true){
        digitalWrite(EXTRUDER_4_MOTOR_DRIVER_ENABLE, LOW); // Enable motor driver for extruder 4
        extruder_4_motor_stepState = !extruder_4_motor_stepState;
        digitalWrite(EXTRUDER_4_MOTOR_DRIVER_STEP, extruder_4_motor_stepState);
    }
    else {
        digitalWrite(EXTRUDER_4_MOTOR_DRIVER_ENABLE, HIGH); // Disable motor driver if not stable or no filament
        extruder_4_motor_stepState = false; // Disable stepper motor if not stable or no filament
    }
    if(extruder_5_isStable&&extruder_5_filament_sensor_state==true){
        digitalWrite(EXTRUDER_5_MOTOR_DRIVER_ENABLE, LOW); // Enable motor driver for extruder 5
        extruder_5_motor_stepState = !extruder_5_motor_stepState;
        digitalWrite(EXTRUDER_5_MOTOR_DRIVER_STEP, extruder_5_motor_stepState);
    }
    else {
        digitalWrite(EXTRUDER_5_MOTOR_DRIVER_ENABLE, HIGH); // Disable motor driver if not stable or no filament
        extruder_5_motor_stepState = false; // Disable stepper motor if not stable or no filament
    }
    // digitalWrite(EXTRUDER_2_MOTOR_DRIVER_ENABLE, LOW);
    // digitalWrite(EXTRUDER_5_MOTOR_DRIVER_ENABLE, LOW);
    // digitalWrite(EXTRUDER_4_MOTOR_DRIVER_ENABLE, LOW); // Enable motor driver for extruder 1
    // extruder_2_motor_stepState = !extruder_2_motor_stepState;
    // extruder_4_motor_stepState = !extruder_4_motor_stepState;
    // extruder_5_motor_stepState = !extruder_5_motor_stepState;
    // digitalWrite(EXTRUDER_2_MOTOR_DRIVER_STEP, extruder_2_motor_stepState);
    // digitalWrite(EXTRUDER_4_MOTOR_DRIVER_STEP, extruder_4_motor_stepState);
    // digitalWrite(EXTRUDER_5_MOTOR_DRIVER_STEP, extruder_5_motor_stepState);
    // if(extruder_1_enable && extruder_1_filament_sensor_state && extruder_1_isStable) {
    //     extruder_1_motor_stepState = !extruder_1_motor_stepState;
    //     digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, LOW); // Enable motor driver
    //     digitalWrite(EXTRUDER_1_MOTOR_DRIVER_STEP, extruder_1_motor_stepState);
    //     if (extruder_1_motor_stepState) {
    //         digitalWrite(37,LOW);
    //         extruder_1_motor_steps++;
    //         stepCount++;
    //     }
    // }
    // else {
    //     digitalWrite(EXTRUDER_1_MOTOR_DRIVER_ENABLE, HIGH); // Disable motor driver
    // }
}
ISR(TIMER5_COMPA_vect) {
}
ISR(TIMER5_COMPB_vect){

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
        double temp4 = EXTRUDER_4.readTemperature();
        double temp5 = EXTRUDER_5.readTemperature();
        // Create JSON formatted string using ArduinoJson
        DynamicJsonDocument doc(1024);
       // StaticJsonDocument<1024> doc;
        if (extruder_1_enable) {
            JsonObject extruder1 = doc["Extruder1"].to<JsonObject>();
            extruder1["Temperature"] = temp1;
            extruder1["isEnabled"] = extruder_1_enable;
            extruder1["heater enable"] = extruder_1_heater_enable;
            extruder1["filament presence"] = extruder_1_filament_sensor_state;
            extruder1["IsStable"] = extruder_1_isStable;
            // extruder1["Steps"] = extruder_1_motor_steps;
            extruder1["extruded filament length"] = extruder_1_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;
        }if (extruder_2_enable) {
            JsonObject extruder2 = doc["Extruder2"].to<JsonObject>();
            extruder2["Temperature"] = temp2;
            extruder2["isEnabled"] = extruder_2_enable;
            extruder2["heater enable"] = extruder_2_heater_enable;
            extruder2["filament presence"] = extruder_2_filament_sensor_state;
            extruder2["IsStable"] = extruder_2_isStable;
            // extruder2["Steps"] = extruder_2_motor_steps;
            extruder2["extruded filament length"] = extruder_2_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;
        }if (extruder_3_enable) {
            JsonObject extruder3 = doc["Extruder3"].to<JsonObject>();
            extruder3["Temperature"] = temp3;
            extruder3["isEnabled"] = extruder_3_enable;
            extruder3["heater enable"] = extruder_3_heater_enable;
            extruder3["filament presence"] = extruder_3_filament_sensor_state;
            extruder3["IsStable"] = extruder_3_isStable;
            //extruder3["Steps"] = extruder_3_motor_steps;
            extruder3["extruded filament length"] = extruder_3_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;
        }if (extruder_4_enable) {
            JsonObject extruder4 = doc["Extruder4"].to<JsonObject>();
            extruder4["Temperature"] = temp4;
            extruder4["isEnabled"] = extruder_4_enable;
            extruder4["heater enable"] = extruder_4_heater_enable;
            extruder4["filament presence"] = extruder_4_filament_sensor_state;
            extruder4["IsStable"] = extruder_4_isStable;
            // extruder4["Steps"] = extruder_4_motor_steps;
            extruder4["extruded filament length"] = extruder_4_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;
        }if (extruder_5_enable) {
            JsonObject extruder5 = doc["Extruder5"].to<JsonObject>();
            extruder5["Temperature"] = temp5;
            extruder5["isEnabled"] = extruder_5_enable;
            extruder5["heater enable"] = extruder_5_heater_enable;
            extruder5["filament presence"] = extruder_5_filament_sensor_state;
            extruder5["IsStable"] = extruder_5_isStable;
            // extruder5["Steps"] = extruder_5_motor_steps;
            extruder5["extruded filament length"] = extruder_5_motor_steps * 0.04 * 3.14159 * 0.4 / steps_per_millimeter;
        }
        serializeJsonPretty(doc, jsonOutput);
        // dataToPublish[0] =  extruder1["isEnabled"];
        // dataToPublish[1] =  extruder2["isEnabled"];
        // dataToPublish[2] =  extruder3["isEnabled"];
        // dataToPublish[3] =  extruder4["isEnabled"];
        // dataToPublish[4] =  extruder5["isEnabled"];
        // dataToPublish[5] =  extruder1["Temperature"];
        // dataToPublish[6] =  extruder2["Temperature"];
        // dataToPublish[7] =  extruder3["Temperature"];
        // dataToPublish[8] =  extruder4["Temperature"];
        // dataToPublish[9] =  extruder5["Temperature"];
        // dataToPublish[10] =  extruder1["filament presence"];
        // dataToPublish[11] =  extruder2["filament presence"];
        // dataToPublish[12] =  extruder3["filament presence"];
        // dataToPublish[13] =  extruder4["filament presence"];
        // dataToPublish[14] =  extruder5["filament presence"];
        // dataToPublish[15] =  extruder1["IsStable"];
        // dataToPublish[16] = extruder2["IsStable"];
        // dataToPublish[17] = extruder3["IsStable"];
        // dataToPublish[18] = extruder4["IsStable"];
        // dataToPublish[19] = extruder5["IsStable"];
        // dataToPublish[20] = extruder1["Steps"];
        // dataToPublish[21] = extruder2["Steps"];
        // dataToPublish[22] = extruder3["Steps"];
        // dataToPublish[23] = extruder4["Steps"];
        // dataToPublish[24] = extruder5["Steps"];
        // dataToPublish[25] = extruder1["extruded filament length"];
        // dataToPublish[26] = extruder2["extruded filament length"];
        // dataToPublish[27] = extruder3["extruded filament length"];
        // dataToPublish[28] = extruder4["extruded filament length"];
        // dataToPublish[29] = extruder5["extruded filament length"];
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
    EXTRUDER_1.checkStability(EXTRUDER_1.readTemperature(), stableStartTime1, extruder_1_isStable);
    EXTRUDER_2.checkStability(EXTRUDER_2.readTemperature(), stableStartTime2, extruder_2_isStable);
    EXTRUDER_3.checkStability(EXTRUDER_3.readTemperature(), stableStartTime3, extruder_3_isStable);
    EXTRUDER_4.checkStability(EXTRUDER_4.readTemperature(), stableStartTime4, extruder_4_isStable);
    EXTRUDER_5.checkStability(EXTRUDER_5.readTemperature(), stableStartTime5, extruder_5_isStable);
}


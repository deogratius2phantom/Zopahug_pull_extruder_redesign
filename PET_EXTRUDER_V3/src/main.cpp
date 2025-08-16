#include <Arduino.h>
#include <pins.h>
#include <thermistor.h>
#include <PID_v1.h> // Include the PID library header
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>
#define EXRTUSION_TEMPERATURE 220 // Set the target temperature for the extruder for PET 220 to 235 degrees
const uint8_t NUMBER_OF_HEATERS = 5; // Number of heaters
// put function declarations here:
struct Heater {
  uint8_t pin;                   // Pin connected to the heater mosfet gate 
  uint8_t sensorPin;             // ADC Pin connected to the thermistor   
  double input, output, setpoint;// PID variables for corresponding heater
  PID* pid;                      // PID object for the heater
  float lastTemp;                // Last temperature reading
  uint32_t lastUpdate;          // Last update time for the heater
  bool enabled;                 // Flag to indicate if the heater is enabled
  bool stable;                  // Flag to indicate if the heater is stable
};

Heater heaters[NUMBER_OF_HEATERS];  //Array of 5 heaters
const uint8_t heaterPins[5] = {
  EXTRUDER_1_HEATER, 
  EXTRUDER_2_HEATER, 
  EXTRUDER_3_HEATER, 
  EXTRUDER_4_HEATER, 
  EXTRUDER_5_HEATER
};

const uint8_t thermistorPins[5] = {
  EXTRUDER_1_THERMISTOR, 
  EXTRUDER_2_THERMISTOR, 
  EXTRUDER_3_THERMISTOR, 
  EXTRUDER_4_THERMISTOR, 
  EXTRUDER_5_THERMISTOR
};

const uint8_t heaterEnableSwitchPins[5] = {
  EXTRUDER_1_ENABLE_SWITCH, 
  EXTRUDER_2_ENABLE_SWITCH, 
  EXTRUDER_3_ENABLE_SWITCH, 
  EXTRUDER_4_ENABLE_SWITCH, 
  EXTRUDER_5_ENABLE_SWITCH
};

bool thermalFault[5] = {false, false, false, false, false};  // Array to store thermal fault status for each heater
const uint32_t TIMEOUT_MS = 20000; // Timeout duration in milliseconds
const uint32_t AUTOTUNE_TIMEOUT_MS = 600000; // 10 minutes
const int AUTOTUNE_CYCLES = 5; // Marlin default is 8 cycles

void checkThermalRunaway(uint8_t i) {
  if (!heaters[i].enabled || thermalFault[i]) return;
  float currentTemp = heaters[i].input;
  if (currentTemp < heaters[i].lastTemp + 1 && (millis() - heaters[i].lastUpdate) > TIMEOUT_MS) {
    analogWrite(heaters[i].pin, 0);
    thermalFault[i] = true;
    Serial.print("Thermal runaway detected on heater ");
    Serial.println(i);
  }
  if (abs(currentTemp - heaters[i].setpoint) < 2) {
    heaters[i].stable = true;
  }
  heaters[i].lastTemp = currentTemp;
  heaters[i].lastUpdate = millis();
}

ISR(TIMER4_COMPA_vect) {
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    // Only run if the enable switch is ON (LOW, assuming INPUT_PULLUP)
    int adc = analogRead(heaters[i].sensorPin);
    heaters[i].input = getTemperature(adc);
    if (digitalRead(heaterEnableSwitchPins[i]) == LOW) {
      heaters[i].enabled = true;
      if (!thermalFault[i]) {
        heaters[i].pid->Compute();
        analogWrite(heaters[i].pin, heaters[i].output);
        //checkThermalRunaway(i);// consider movingt this to the main loop for better control
      }
    } else {
      heaters[i].enabled = false;
      analogWrite(heaters[i].pin, 0); // Turn off heater if not enabled
    }
  }
}

void setupTimer4() {
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4  = 0;
  TCCR4B |= (1 << WGM42) | (1 << CS42) | (1 << CS40); // CTC mode, prescaler 1024
  OCR4A = 15624; // 100 ms at 16 MHz
  TIMSK4 |= (1 << OCIE4A); // Enable interrupt
}

void setupHeaters() {
  for (int i = 0; i < 5; i++) {
    heaters[i] = {heaterPins[i], thermistorPins[i], 0, 0, EXRTUSION_TEMPERATURE, nullptr, 0, 0, true, false};
    pinMode(heaterPins[i], OUTPUT);
    pinMode(heaterEnableSwitchPins[i], INPUT_PULLUP);
    heaters[i].pid = new PID(&heaters[i].input, &heaters[i].output, &heaters[i].setpoint, 38.4, 4.8, 40.7, DIRECT);
    heaters[i].pid->SetOutputLimits(0, 200);
    heaters[i].pid->SetMode(AUTOMATIC);
  }
}

#define EEPROM_PID_ADDR_BASE 0
#define EEPROM_PID_BLOCK_SIZE 12 // 3 floats (Kp, Ki, Kd) * 4 bytes

void savePIDToEEPROM(uint8_t heaterIndex, double Kp, double Ki, double Kd) {
  int addr = EEPROM_PID_ADDR_BASE + heaterIndex * EEPROM_PID_BLOCK_SIZE;
  EEPROM.put(addr, Kp); addr += sizeof(float);
  EEPROM.put(addr, Ki); addr += sizeof(float);
  EEPROM.put(addr, Kd);
}

void loadPIDFromEEPROM(uint8_t heaterIndex, double &Kp, double &Ki, double &Kd) {
  int addr = EEPROM_PID_ADDR_BASE + heaterIndex * EEPROM_PID_BLOCK_SIZE;
  EEPROM.get(addr, Kp); addr += sizeof(float);
  EEPROM.get(addr, Ki); addr += sizeof(float);
  EEPROM.get(addr, Kd);
}

void loadAllPIDFromEEPROM() {
  for (uint8_t i = 0; i < NUMBER_OF_HEATERS; i++) {
    double Kp, Ki, Kd;
    loadPIDFromEEPROM(i, Kp, Ki, Kd);
    if (Kp > 0 && Ki > 0 && Kd > 0) {
      heaters[i].pid->SetTunings(Kp, Ki, Kd);
    }
  }
}

void autoTunePID(uint8_t heaterIndex, double tuneSetpoint, int power, int cycles) {
    Serial.print("Manual PID Autotune start: Heater "); Serial.print(heaterIndex);
    Serial.print(" | Setpoint: "); Serial.print(tuneSetpoint);
    Serial.print("C | Power: "); Serial.print(power);
    Serial.print(" | Cycles: "); Serial.println(cycles);

    heaters[heaterIndex].setpoint = tuneSetpoint;
    double input = getTemperature(analogRead(heaters[heaterIndex].sensorPin));
    double output = 0;
    bool heating = true;
    int cycleCount = 0;
    double maxTemp = input, minTemp = input;
    unsigned long t1 = 0, t2 = 0, period = 0;
    double Ku = 0, Pu = 0;
    double lastInput = input;
    unsigned long start = millis();
    unsigned long lastUpdate = millis();

    // Start with heater off
    analogWrite(heaters[heaterIndex].pin, 0);
    delay(500);

    while (cycleCount < cycles) {
        input = getTemperature(analogRead(heaters[heaterIndex].sensorPin));

        // Print progress
        Serial.print("Tuning Heater ");
        Serial.print(heaterIndex);
        Serial.print(" | Temp: ");
        Serial.print(input, 1);
        Serial.print(" C | Output: ");
        Serial.println(output);

        // --- Thermal runaway protection ---
        if (input < lastInput + 1 && (millis() - lastUpdate) > TIMEOUT_MS) {
            analogWrite(heaters[heaterIndex].pin, 0);
            Serial.println("Thermal runaway detected during auto-tune. Aborting.");
            return;
        }
        lastInput = input;
        lastUpdate = millis();

        if (millis() - start > AUTOTUNE_TIMEOUT_MS) {
            analogWrite(heaters[heaterIndex].pin, 0);
            Serial.println("Auto-tune timeout.");
            return;
        }

        // Heating phase
        if (heating) {
            output = power;
            analogWrite(heaters[heaterIndex].pin, output);
            if (input > tuneSetpoint + 5) { // Overshoot threshold
                heating = false;
                maxTemp = input;
                t1 = millis();
            }
        } else { // Cooling phase
            output = 0;
            analogWrite(heaters[heaterIndex].pin, output);
            if (input < tuneSetpoint - 5) { // Undershoot threshold
                heating = true;
                minTemp = input;
                t2 = millis();
                period = t2 - t1;
                t1 = t2;
                cycleCount++;
                Serial.print("Cycle "); Serial.print(cycleCount);
                Serial.print(": Max="); Serial.print(maxTemp, 2);
                Serial.print(" Min="); Serial.print(minTemp, 2);
                Serial.print(" Period="); Serial.print(period / 1000.0, 2); Serial.println("s");
                // Calculate Ku and Pu for Ziegler-Nichols
                Ku = (4.0 * power) / (3.14159 * (maxTemp - minTemp));
                Pu = period / 1000.0;
            }
        }
        delay(100);
    }
    analogWrite(heaters[heaterIndex].pin, 0); // Turn off heater after tuning

    // Ziegler-Nichols tuning rules
    double Kp = 0.6 * Ku;
    double Ki = 1.2 * Ku / Pu;
    double Kd = 0.075 * Ku * Pu;

    Serial.println("Manual PID Autotune finished!");
    Serial.print("Kp: "); Serial.print(Kp, 4);
    Serial.print(" Ki: "); Serial.print(Ki, 4);
    Serial.print(" Kd: "); Serial.println(Kd, 4);

    savePIDToEEPROM(heaterIndex, Kp, Ki, Kd);
    heaters[heaterIndex].pid->SetTunings(Kp, Ki, Kd);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupHeaters();
  loadAllPIDFromEEPROM();
  setupTimer4();
  sei(); // enable interrupts
}

void loop() {
  // put your main code here, to run repeatedly:
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 2000) {
    lastPrint = millis();
    for (int i = 0; i < 5; i++) {
      Serial.print("Heater ");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(heaters[i].input);
      Serial.print("C | Stable: ");
      Serial.print(heaters[i].stable ? "Yes" : "No");
      Serial.print(" | Enabled: ");
      Serial.println(heaters[i].enabled ? "Yes" : "No");
      checkThermalRunaway(i);
    }
    Serial.println();
  }

  // Serial command handler for PID auto-tune
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("TUNE")) {
      // Format: TUNE <heater> <setpoint> <power>
      int idx = -1, power = 200;
      double setpoint = EXRTUSION_TEMPERATURE;
      int numArgs = sscanf(cmd.c_str(), "TUNE %d %lf %d", &idx, &setpoint, &power);
      if (idx >= 0 && idx < NUMBER_OF_HEATERS) {
        if (numArgs == 2) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else if (numArgs == 3) autoTunePID(idx, setpoint, power, AUTOTUNE_CYCLES);
        else autoTunePID(idx, EXRTUSION_TEMPERATURE, 200, AUTOTUNE_CYCLES);
      } else {
        Serial.println("Invalid heater index. Use 0-4.");
      }
    }
  }
}

// put function definitions here:

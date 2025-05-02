// #include <pid_tunner.h>
// #include <Arduino.h>
// #include <pins.h>
// #include <thermistor.h>
// byte ATuneModeRemember=2;
// double input=80, output=50, setpoint=180;
// double kp=38.27,ki=5.38,kd=40.70;
// double kpmodel=1.5, taup=100, theta[50];
// double outputStart=5;
// double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
// unsigned int aTuneLookBack=20;

// boolean tuning = false;
// unsigned long  modelTime, serialTime;
// PID myPID(&input, &output, &setpoint,kp,ki,kd, DIRECT);
// PID_ATune aTune(&input, &output);

// //set to false to connect to the real world
// boolean useSimulation = true;
// bool setHotEndTemperature(uint8_t target_temperature)
// {
//     unsigned long startTime = millis();
//     unsigned long stableTime = 0;
//     const unsigned long stableDuration = 10000; // 10 seconds
//     const double tolerance = 1.0; // Temperature tolerance

//     setpoint = target_temperature;
//     myPID.SetMode(AUTOMATIC);

//     while (true) {
//         input = getTemperature(analogRead(PIN_THERMISTOR1));
//         myPID.Compute();
//         analogWrite(PIN_HEATER, output);
//         Serial.print("temperature: ");
//         Serial.println(input);
//         if (abs(setpoint - input) <= tolerance) {
//             if (stableTime == 0) {
//                 stableTime = millis();
//             } else if (millis() - stableTime >= stableDuration) {
//                 return true;
//             }
//         } else {
//             stableTime = 0;
//         }

//         if (millis() - startTime > 60000) { // Timeout after 60 seconds
//             return false;
//         }

//         delay(100); // Small delay to prevent overwhelming the loop
//     }
// }
// void tunePID(double tuner_set_point)
// {
//     input = getTemperature(analogRead(PIN_THERMISTOR1));
//     double bestKp = kp, bestKi = ki, bestKd = kd;
//     double bestError = 1e10; // Initialize with a large number

//     for (int cycle = 0; cycle < 10; cycle++) {
//         double totalError = 0;
//         unsigned long startTime = millis();
//         unsigned long stableTime = 0;
//         const unsigned long stableDuration = 10000; // 10 seconds
//         const double tolerance = 1.0; // Temperature tolerance

//         setpoint = tuner_set_point;
//         myPID.SetTunings(kp, ki, kd);
//         myPID.SetMode(AUTOMATIC);

//         while (true) {
//             input = getTemperature(analogRead(PIN_THERMISTOR1));
//             myPID.Compute();
//             analogWrite(PIN_HEATER, output);
//             Serial.print("Cycle: ");
//             Serial.print(cycle);
//             Serial.print(" temperature: ");
//             Serial.println(input);

//             double error = abs(setpoint - input);
//             totalError += error;

//             if (error <= tolerance) {
//                 if (stableTime == 0) {
//                     stableTime = millis();
//                 } else if (millis() - stableTime >= stableDuration) {
//                     break;
//                 }
//             } else {
//                 stableTime = 0;
//             }

//             if (millis() - startTime > 60000) { // Timeout after 60 seconds
//                 break;
//             }

//             delay(100); // Small delay to prevent overwhelming the loop
//         }

//         if (totalError < bestError) {
//             bestError = totalError;
//             bestKp = kp;
//             bestKi = ki;
//             bestKd = kd;
//         }

//         // Adjust PID parameters for the next cycle (simple tuning strategy)
//         kp += 1;
//         ki += 0.05;
//         kd += 0.1;
//     }

//     // Set the best PID parameters found
//     myPID.SetTunings(bestKp, bestKi, bestKd);
//     Serial.print("Best PID parameters: Kp=");
//     Serial.print(bestKp);
//     Serial.print(", Ki=");
//     Serial.print(bestKi);
//     Serial.print(", Kd=");
//     Serial.println(bestKd);

// }
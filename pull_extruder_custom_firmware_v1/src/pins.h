#ifndef PINS_H
#define PINS_H

// Define pin numbers for various components
#define EXTRUDER_1_MOTOR_DRIVER_STEP    36
#define EXTRUDER_1_MOTOR_DRIVER_DIR     34
#define EXTRUDER_1_MOTOR_DRIVER_ENABLE  30
#define EXTRUDER_1_HEATER               10
#define EXTRUDER_1_FILAMENT_SENSOR      25
#define EXTRUDER_1_THERMISTOR           A13

#define EXTRUDER_2_MOTOR_DRIVER_STEP    26
#define EXTRUDER_2_MOTOR_DRIVER_DIR     28
#define EXTRUDER_2_MOTOR_DRIVER_ENABLE  24
#define EXTRUDER_2_HEATER               9
#define EXTRUDER_2_FILAMENT_SENSOR      29
#define EXTRUDER_2_THERMISTOR           A14

#define EXTRUDER_3_MOTOR_DRIVER_STEP    A0
#define EXTRUDER_3_MOTOR_DRIVER_DIR     A1
#define EXTRUDER_3_MOTOR_DRIVER_ENABLE  38
#define EXTRUDER_3_HEATER               8
#define EXTRUDER_3_FILAMENT_SENSOR      14
#define EXTRUDER_3_THERMISTOR           A15

#define EXTRUDER_4_MOTOR_DRIVER_STEP    32
#define EXTRUDER_4_MOTOR_DRIVER_DIR     33
#define EXTRUDER_4_MOTOR_DRIVER_ENABLE  31
#define EXTRUDER_4_HEATER               7
#define EXTRUDER_4_FILAMENT_SENSOR      15
#define EXTRUDER_4_THERMISTOR           A5

#define EXTRUDER_5_MOTOR_DRIVER_STEP    22
#define EXTRUDER_5_MOTOR_DRIVER_DIR     23 
#define EXTRUDER_5_MOTOR_DRIVER_ENABLE  21
#define EXTRUDER_5_HEATER               6
#define EXTRUDER_5_FILAMENT_SENSOR      16
#define EXTRUDER_5_THERMISTOR           A10

//EXtruder operation enable switches

#define EXTRUDER_1_ENABLE_SWITCH        35
#define EXTRUDER_2_ENABLE_SWITCH        37
#define EXTRUDER_3_ENABLE_SWITCH        23
#define EXTRUDER_4_ENABLE_SWITCH        31
#define EXTRUDER_5_ENABLE_SWITCH        33

#define LED_PIN 13 // Built-in LED
//Communication control pin for sim800L


// Add more pin definitions as needed

#endif // PINS_H
#include "tmc/helpers/Types.h"

#define GLOBAL_OK

// Uncomment to turn debugging on
#define DEBUG
// #define DEBUG_HOMING
#define DEBUG_SCALING

#define MOTOR_COUNT 4

#define TMC5041_CLOCK_HZ 13200000

// Define GPIO pin numbers
// Be sure to sync any changes here to linuxcnc-rpi-image config.txt configuration.
//
// TORCH_FIRE / gpio 20 / pin 38 / output, drive low
#define PIN_TORCH_ON 20
// ARC_FREQ / gpio 1, pin 28 
// #define PIN_ARC_FREQ 1
// ARC_OK / gpio 26 / pin 37 / input, pull down
#define PIN_ARC_OK 26
// IHS_ENABLE / gpio 15 / pin 10 / output, drive low
#define PIN_OHMIC_ENABLE 15
// IHS_SENSE / gpio 14 / pin 8 / input, pull down
#define PIN_OHMIC_PROBE 14
// ESTOP / gpio 16 / pin 36 / input, pull down
#define PIN_ESTOP 16
// TORCH_FLOAT / gpio 12 / pin 32 / input, pull down
#define PIN_TORCH_FLOAT 12
// TORCH_LASER / gpio 21 / pin 40 / output, drive low
#define PIN_TORCH_LASER 21

// SPI
#define NUM_CS_PINS 4   // Total number of chip select pins
#define SPI0_CS0_GPIO 8
#define SPI0_CS1_GPIO 7
#define SPI0_CS2_GPIO 19
#define SPI0_CS3_GPIO 1

// ADC
#define ARC_VOLT_CS 3
#define ARC_VOLT_CHANNEL 0
#define ARC_VOLT_DIVISION 50
#define AIR_PRESSURE_CS 2
#define AIR_PRESSURE_CHANNEL 0

// Actual reference voltages
#define REF_5V 4.187

// Homing debounce
#define HOME_SEARCHING_DEBOUNCE 0
#define HOME_BACKING_DEBOUNCE 500
#define HOME_LATCHING_DEBOUNCE 0

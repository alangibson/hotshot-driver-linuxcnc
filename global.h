#include "tmc/helpers/Types.h"

#define GLOBAL_OK

// Uncomment to turn debugging on
// #define DEBUG
// #define DEBUG_HOMING
#define DEBUG_SCALING

#define MOTOR_COUNT 4

#define TMC5041_CLOCK_HZ 13200000

// Define GPIO pin numbers
//
// pin 31 / TORCH_FIRE
#define PIN_TORCH_ON 6
// pin 28 / ARC_FREQ
#define PIN_ARC_FREQ 1
// pin 37 / ARC_OK
#define PIN_ARC_OK 26
// pin 27 / IHS_ENABLE
#define PIN_OHMIC_ENABLE 0
// pin 29 / IHS_SENSE
#define PIN_OHMIC_PROBE 5
// pin 33 / ESTOP
#define PIN_ESTOP 13
// unused
// #define PIN_CLOCK 18
// unused
// #define PIN_TORCH_BREAKAWAY 1
// pin 35 / TORCH_FLOAT
#define PIN_TORCH_FLOAT 19
// pin 40 / TORCH_LASER
#define PIN_TORCH_LASER 21

// Homing debounce
#define HOME_SEARCHING_DEBOUNCE 0
#define HOME_BACKING_DEBOUNCE 500
#define HOME_LATCHING_DEBOUNCE 0

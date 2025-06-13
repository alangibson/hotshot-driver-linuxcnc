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
// pin 38 / TORCH_FIRE
#define PIN_TORCH_ON 20
// pin 28 / ARC_FREQ
#define PIN_ARC_FREQ 1
// pin 37 / ARC_OK
#define PIN_ARC_OK 26
// pin 10 / IHS_ENABLE
#define PIN_OHMIC_ENABLE 15
// pin 8 / IHS_SENSE
#define PIN_OHMIC_PROBE 14
// pin 36 / ESTOP
#define PIN_ESTOP 16
// pin 32 / TORCH_FLOAT
#define PIN_TORCH_FLOAT 12
// pin 40 / TORCH_LASER
#define PIN_TORCH_LASER 21

// Homing debounce
#define HOME_SEARCHING_DEBOUNCE 0
#define HOME_BACKING_DEBOUNCE 500
#define HOME_LATCHING_DEBOUNCE 0

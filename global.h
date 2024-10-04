#include "tmc/helpers/Types.h"

#define GLOBAL_OK

// Uncomment to turn debugging on
// #define DEBUG
// #define DEBUG_HOMING
#define DEBUG_SCALING

#define MOTOR_COUNT 4

#define TMC5041_CLOCK_HZ 13200000

// Define GPIO pin numbers
#define PIN_TORCH_ON 6
#define PIN_ARC_FREQ 22
#define PIN_ARC_OK 26
#define PIN_OHMIC_ENABLE 0
#define PIN_OHMIC_PROBE 5
#define PIN_ESTOP 13
#define PIN_CLOCK 18
#define PIN_TORCH_BREAKAWAY 1
#define PIN_TORCH_FLOAT 19
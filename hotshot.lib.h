#include "tmc/helpers/Types.h"

#define PULSES_TO_UNITS(pulses, units_per_pulse) ((pulses) * (units_per_pulse))
#define UNITS_TO_PULSES(units, units_per_pulse) ((units) / (units_per_pulse))

uint32_t units_per_rev(uint32_t pitch, uint32_t teeth);
uint32_t pulses_per_rev(uint32_t motor_fullsteps_per_rev, uint32_t microsteps);
float64_t units_per_pulse(float64_t units_per_rev, float64_t pulses_per_rev);
int32_t units_to_pulses(float64_t units, float64_t units_per_pulse);
float64_t pulses_to_units(int32_t pulses, float64_t units_per_pulse);

float thc_voltage(uint8_t chip, float V_ref);
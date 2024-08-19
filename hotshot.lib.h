#include "tmc/helpers/Types.h"

float64_t pulses_to_units(uint32_t pulses, float64_t unit_pulse_factor);
int32_t units_to_pulses(float64_t units, float64_t unit_pulse_factor);

float thc_voltage(uint8_t chip, float V_ref);
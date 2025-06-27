#include "stdio.h"
#include "stdint.h"
#include "sys/time.h"
#include "bcm2835.h"
#include "hotshot.h"

// Example:
//
// pitch = 2
// teeth = 20
// fs_per_rev = 200
// microsteps = 256 
// units = 100.0
//
// units_per_rev = pitch * teeth = 2 * 20 = 40
// pulses_per_rev = fs_per_rev * microsteps = 200 * 256 = 51200
//
// units_per_pulse = units_per_rev / pulses_per_rev = 40 / 51200 = 0.00078125
// pulses_per_unit = pulses_per_rev / units_per_rev = 51200 / 40 = 1280
//
// units_to_pulses = units / units_per_pulse = 100.0 / 0.00078125 = 128000
// pulses_to_units = pulses * units_per_pulse = 128000 * 0.00078125 = 100

uint32_t units_per_rev(uint32_t pitch, uint32_t teeth)
{
    return pitch * teeth;
}

uint32_t pulses_per_rev(uint32_t motor_fullsteps_per_rev, uint32_t microsteps)
{
    return motor_fullsteps_per_rev * microsteps;
}

float64_t units_per_pulse(float64_t units_per_rev, float64_t pulses_per_rev)
{
    return units_per_rev / pulses_per_rev;
}

/** Convert from length unit to microstep pulses */
int32_t units_to_pulses(float64_t units, float64_t units_per_pulse)
{
    // Implicit conversion to int should just chop off decimal, not round
    return units / units_per_pulse;
}

/** Convert from microstep pulses to length unit */
float64_t pulses_to_units(int32_t pulses, float64_t units_per_pulse)
{
    return pulses * units_per_pulse;
}

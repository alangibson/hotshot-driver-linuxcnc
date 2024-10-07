#include "stdio.h"
#include "signal.h"
#include "time.h"
#include "assert.h"
#include "tmc5041.h"
#include "hotshot.lib.h"

void unitTests() {
    
    uint32_t pitch_cmd = 2;         // GT2 belt
    uint32_t teeth_cmd = 20;        // 20 tooth gear
    uint32_t microsteps_cmd = 256;  
    uint32_t steps_per_rev = 200;   // 1.8 degree step
    float64_t length = 100;         // mm

    uint32_t units_per_rev_1 = units_per_rev(pitch_cmd, teeth_cmd);
    assert(units_per_rev_1 == 40);

    uint32_t pulses_per_rev_1 = pulses_per_rev(steps_per_rev, microsteps_cmd);
    assert(pulses_per_rev_1 == 51200);

    float64_t units_per_pulse_1 = units_per_pulse(units_per_rev_1, pulses_per_rev_1);
    assert(units_per_pulse_1 == 0.00078125);

    int32_t units_to_pulses_1 = units_to_pulses(length, units_per_pulse_1);
    assert(units_to_pulses_1 == 128000);

    int32_t units_to_pulses_2 = units_to_pulses(-length, units_per_pulse_1);
    assert(units_to_pulses_2 == -128000);

    float64_t pulses_to_units_1 = pulses_to_units(units_to_pulses_1, units_per_pulse_1);
    assert(pulses_to_units_1 == length);

    float64_t pulses_to_units_2 = pulses_to_units(-units_to_pulses_1, units_per_pulse_1);
    assert(pulses_to_units_2 == -length);

    uint32_t fclk = 13200000;
    float64_t velocity_time_ref_1 = tmc5041_velocity_time_ref(fclk);
    printf("velocity_time_ref_1: %f\n", velocity_time_ref_1);
    // FIXME assert(velocity_time_ref_1 == 1.271001);

    float64_t acceleration_time_ref_1 = tmc5041_acceleration_time_ref(fclk);
    printf("acceleration_time_ref_1: %f\n", acceleration_time_ref_1);
    // FIXME assert(acceleration_time_ref_1 == 0.012621);
}

int main() {
    printf("Running unit tests\n");
    unitTests();
    printf("Unit tests passed\n");

    return 0;
}
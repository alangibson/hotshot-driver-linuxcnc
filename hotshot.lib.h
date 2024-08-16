#include "tmc/helpers/Types.h"
#include "hotshot.h"

uint32_t microsteps_per_mm(uint32_t fullsteps_per_rev, float64_t linear_mm_per_rev, uint32_t microsteps);
int64_t mm_to_microsteps(uint32_t microstep_per_mm, float64_t mm);
float64_t microsteps_to_mm(uint32_t microsteps_per_mm, int32_t microsteps);
// void follow(joint_t * motor);
void move(joint_t * motor);
void update(joint_t * motor);

float thc_voltage(uint8_t chip, float V_ref);
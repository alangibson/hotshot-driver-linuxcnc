// Convenience functions on top of low level TMC5041 functions

#include "tmc/helpers/Types.h"
#include "bcm2835.h"
#include "hotshot.h"

int64_t mm_to_microsteps(uint32_t microstep_per_mm, float mm)
{
    return mm * microstep_per_mm;
}

uint32_t microsteps_per_mm(uint32_t fullsteps_per_rev, float linear_mm_per_rev, uint32_t microsteps)
{
    return (fullsteps_per_rev * microsteps) / linear_mm_per_rev;
}

spi_status_t follow(joint_t * motor)
{
    // XTARGET: Target position for ramp mode
    // Calculate target position and velocity
    // Implicitly round to lower since were converting from float to int
    int32 xtarget = (*motor->position_cmd) * motor->microstep_per_mm;
    spi_status_t spi_status = tmc5041_set_register_XTARGET(&motor->tmc, xtarget);

    // TODO return xactual_mm
    // int64_t xactual_value = tmc5041_get_register_XACTUAL(&motor->tmc);
    // spi_status.xactual_mm = (float)xactual_value / (float)motor->microstep_per_mm;

    return spi_status;
}

float thc_voltage(uint8_t chip, float V_ref)
{
    //
    // Read from ADC
    // Protocol start bit (S), sgl/diff (D), odd/sign (C), MSBF (M)
    //  S = 1
    //  D = 1 ; 0=diff, 1= Single ended (i.e. 2 channel) mode
    //  C = 0; if D=0, then odd/sign, if D=1, then 0 or 1 for channel number
    //  M = 1
    // Message = { 0000000S, DCM00000, 00000000 }
    // Final byte of zeros is so we get an 3 total bytes back from ADC
    char send_data[3] = {0b00000001, 0b10000000, 0b00000000};
    bcm2835_aux_spi_transfern(send_data, 3);
    // Intrepret ADC response
    // Get sensor value
    uint16_t value = 0x00;
    // We need only data from last 2 bytes.
    // And there we can discard last two bits to get 10 bit value
    // as MCP3002 resolution is 10bits
    //
    // 15 = 0000 1111 with & operation makes sure that we have all data from XXXX DDDD and nothing more.
    value = send_data[1] & 15;
    // Move to left to make room for next piece of data.
    value = value << 6; // 000D DDDD << 6 == 0DDD DD00 0000
    // Now we get the last of data from byte 3, discarding last two bits
    value = value + (send_data[2] >> 2);
    // DDDD DDXXX >> 2 == 00DD DDDD
    // 0DDD DD00 0000 + 00DD DDDD == 0DDD DDDD DDDD
    // Convert to voltage
    // Voltage = ( V_ref * value ) / 1024
    float voltage = (V_ref * value) / 1024;

    // Assumes 50:1 voltage divider
    return voltage * 50;
}


bool get_home_switch_state(joint_t * motor, bool axis_is_homing) {
    if (axis_is_homing) {
        ramp_stat_register_t reg = tmc5041_get_register_RAMP_STAT(&motor->tmc);
        return reg.status_sg;
    }
    return FALSE;
}

bool get_neg_limit_switch_state(joint_t * motor, float axis_max_velocity_cmd) {
    if (axis_max_velocity_cmd < 0) {
        ramp_stat_register_t reg = tmc5041_get_register_RAMP_STAT(&motor->tmc);
        return reg.status_sg;
    }
    return FALSE;
}

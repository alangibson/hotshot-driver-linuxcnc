// Convenience functions on top of low level TMC5041 functions

#include "stdio.h"
#include "tmc/helpers/Types.h"
#include "bcm2835.h"
#include "hotshot.h"

/**
 * Convert linear mm to microsteps based on motor/drive's microstep_per_mm conversion factor.
*/
int64_t mm_to_microsteps(uint32_t microstep_per_mm, float64_t mm)
{
    return mm * microstep_per_mm;
}

/**
 * Convert microsteps to linear mm to based on motor/drive's microstep_per_mm conversion factor.
*/
float64_t microsteps_to_mm(uint32_t microsteps_per_mm, int32_t microsteps) {
    return microsteps / (float64_t)microsteps_per_mm;
}

/**
 * Calculate number of microsteps need to move one linear mm based on 
 * motor/drive parameters.
*/
uint32_t microsteps_per_mm(uint32_t fullsteps_per_rev, float64_t linear_mm_per_rev, uint32_t microsteps)
{
    return (fullsteps_per_rev * microsteps) / linear_mm_per_rev;
}

void move(joint_t * joint)
{
    // VMAX is set on the fly by PID
    int32_t vmax = mm_to_microsteps(joint->microstep_per_mm, *joint->velocity_cmd);

    // RAMPMODE:
    //  1: Velocity mode to positive VMAX (using AMAX acceleration)
    //  2: Velocity mode to negative VMAX (using AMAX acceleration)
    if (vmax > 0)
        tmc5041_set_register_RAMPMODE(&joint->tmc, 1);
    else if (vmax < 0)
        tmc5041_set_register_RAMPMODE(&joint->tmc, 2);
    // TODO else ???
    
    tmc5041_set_register_VMAX(&joint->tmc, vmax);

    // Update pins
    int32_t xactual             = tmc5041_get_register_XACTUAL(&joint->tmc);
    float64_t xactual_mm        = microsteps_to_mm(joint->microstep_per_mm, xactual);
    int32_t vactual             = tmc5041_get_register_VACTUAL(&joint->tmc);
    float64_t vactual_mm        = microsteps_to_mm(joint->microstep_per_mm, vactual);
    *joint->tmc.velocity_fb     = vactual;
    *joint->tmc.position_fb     = xactual;
    *joint->velocity_fb         = vactual_mm;
    *joint->position_fb         = xactual_mm;

    #ifdef DEBUG
    int32_t xtarget = mm_to_microsteps(joint->microstep_per_mm, *joint->position_cmd);
    printf("move(%d, %d): xtarget=%d us,%f mm\n", 
        joint->tmc.chip, joint->tmc.motor, 
        xtarget, *joint->position_cmd);
    printf("move(%d, %d): xactual=%d us,%f mm\n", 
        joint->tmc.chip, joint->tmc.motor, xactual, xactual_mm);
    printf("move(%d, %d): follow error = %d us\n", 
        joint->tmc.chip, joint->tmc.motor, xtarget - xactual);
    printf("move(%d, %d): vmax=%d us/sec,%f mm/sec\n", 
        joint->tmc.chip, joint->tmc.motor, vmax, *joint->velocity_cmd);
    printf("move(%d, %d): vactual=%d us/sec,%f mm/sec\n", 
        joint->tmc.chip, joint->tmc.motor, vactual, vactual_mm);
    #endif
}

void follow(joint_t * joint)
{
    //
    // Plan move
    //

    float64_t last_position_fb_mm = joint->last_position_fb;
    int32_t last_position_fb = mm_to_microsteps(joint->microstep_per_mm, last_position_fb_mm);
    
    int32_t last_needed_move_distance = joint->last_xtarget - 
        mm_to_microsteps(joint->microstep_per_mm, *joint->position_fb);

    // Implicitly round to lower since were converting from float to int
    int32 xtarget = mm_to_microsteps(joint->microstep_per_mm, *joint->position_cmd);
    float64_t xtarget_mm = *joint->position_cmd;
 
    int32_t xactual = tmc5041_get_register_XACTUAL(&joint->tmc);
    float64_t xactual_mm = microsteps_to_mm(joint->microstep_per_mm, xactual);

    int32_t actual_move_distance = xactual - last_position_fb;
    float64_t actual_move_distance_mm = microsteps_to_mm(joint->microstep_per_mm, actual_move_distance);

    int32_t follow_error = joint->last_xtarget - xactual;
    float64_t follow_error_mm = microsteps_to_mm(joint->microstep_per_mm, follow_error);

    int32_t next_move_distance = xtarget - xactual;
    float64_t next_move_distance_mm = microsteps_to_mm(joint->microstep_per_mm, next_move_distance);

    int32_t vactual = tmc5041_get_register_VACTUAL(&joint->tmc);
    float64_t vactual_mm = microsteps_to_mm(joint->microstep_per_mm, vactual);

    int32_t max_velocity = mm_to_microsteps(joint->microstep_per_mm, *joint->max_velocity_cmd);

    int32_t vmax = mm_to_microsteps(joint->microstep_per_mm, *joint->velocity_cmd);
    vmax = abs(vmax);
    float64_t vmax_mm = microsteps_to_mm(joint->microstep_per_mm, vmax);

    int32_t amax = mm_to_microsteps(joint->microstep_per_mm, *joint->acceleration_cmd);
    float64_t amax_mm = microsteps_to_mm(joint->microstep_per_mm, amax);

    //
    // Report
    //

    #ifdef DEBUG
    printf("follow(%d, %d): Prior:\n", joint->tmc.chip, joint->tmc.motor);
    printf("follow(%d, %d):     last_position_fb=%f mm, %d us\n", 
        joint->tmc.chip, joint->tmc.motor, last_position_fb_mm, last_position_fb);
    printf("follow(%d, %d):     last xtarget=%d us, %f mm\n", 
        joint->tmc.chip, joint->tmc.motor, joint->last_xtarget, microsteps_to_mm(joint->microstep_per_mm, joint->last_xtarget));
    printf("follow(%d, %d):     Prior travel actual_move_distance=%d us, %f mm\n", 
        joint->tmc.chip, joint->tmc.motor, actual_move_distance, actual_move_distance_mm);
    
    printf("follow(%d, %d): Current:\n", joint->tmc.chip, joint->tmc.motor);
    printf("follow(%d, %d):     EMC joint velocity %f mm/sec, %d us/sec\n", 
        joint->tmc.chip, joint->tmc.motor,
        *joint->velocity_cmd, mm_to_microsteps(joint->microstep_per_mm, *joint->velocity_cmd));
    printf("follow(%d, %d):     EMC joint acceleration %f mm/sec2, %d us/sec2\n", 
        joint->tmc.chip, joint->tmc.motor,
        *joint->acceleration_cmd, mm_to_microsteps(joint->microstep_per_mm, *joint->acceleration_cmd));
    printf("follow(%d, %d):     Current position xactual_mm=%f mm, %d us\n", 
        joint->tmc.chip, joint->tmc.motor, xactual_mm, xactual);
    printf("follow(%d, %d):     Follow error follow_error=%d us, %f mm\n", 
        joint->tmc.chip, joint->tmc.motor, follow_error, follow_error_mm);
    printf("follow(%d, %d):     Curent velocity vactual=%d us/sec, %f mm/sec\n", 
        joint->tmc.chip, joint->tmc.motor, vactual, vactual_mm);

    printf("follow(%d, %d): Plan:\n", joint->tmc.chip, joint->tmc.motor);
    printf("follow(%d, %d):     Move to position_cmd=%f mm, xtarget=%d us\n", 
        joint->tmc.chip, joint->tmc.motor, xtarget_mm, xtarget);
    printf("follow(%d, %d):     Difference to last position_cmd=%f mm\n", 
        joint->tmc.chip, joint->tmc.motor, *joint->position_cmd - last_position_fb_mm);
    printf("follow(%d, %d):     Move distance next_move_distance=%d us, %f mm\n", 
        joint->tmc.chip, joint->tmc.motor, next_move_distance, next_move_distance_mm);
    printf("follow(%d, %d):     Velocity becomes adjusted vmax=%d us/sec, %f mm/sec\n", 
        joint->tmc.chip, joint->tmc.motor, vmax, vmax_mm);

    printf("follow(%d, %d): Move:\n", joint->tmc.chip, joint->tmc.motor);
    printf("follow(%d, %d):     Position position_cmd=%f mm, %d us\n", 
        joint->tmc.chip, joint->tmc.motor, xtarget_mm, xtarget);
    printf("follow(%d, %d):     Velocity velocity_cmd=%f mm/sec, %d us/sec; max_velocity_cmd=%f mm/sec, %d us/sec\n", 
        joint->tmc.chip, joint->tmc.motor, vmax_mm, vmax,
        *joint->max_velocity_cmd, mm_to_microsteps(joint->microstep_per_mm, *joint->max_velocity_cmd));
    printf("follow(%d, %d):     Acceleration acceleration_cmd=%f mm/sec, %d us/sec; max_acceleration_cmd=%f mm/sec, %d us/sec\n", 
        joint->tmc.chip, joint->tmc.motor, amax_mm, amax,
        *joint->max_acceleration_cmd, mm_to_microsteps(joint->microstep_per_mm, *joint->max_acceleration_cmd));

    #endif

    //
    // Move
    //

    joint->tmc.velocity_cmd     = vmax;
    joint->tmc.acceleration_cmd = amax;

    if (next_move_distance != 0) 
    {
        tmc5041_set_register_XTARGET(&joint->tmc, xtarget);
        // tmc5041_set_register_VSTART(&joint->tmc, vmax);
        // tmc5041_set_register_VSTOP(&joint->tmc, vmax);
        tmc5041_set_register_VMAX(&joint->tmc, vmax);
        tmc5041_set_register_AMAX(&joint->tmc, amax);        
    }

    //
    // Update feedback pins and vars for next iteration
    //

    // Refresh xactual
    xactual = tmc5041_get_register_XACTUAL(&joint->tmc);
    xactual_mm = microsteps_to_mm(joint->microstep_per_mm, xactual);
    // Refresh vactual
    vactual = tmc5041_get_register_VACTUAL(&joint->tmc);
    vactual_mm = microsteps_to_mm(joint->microstep_per_mm, vactual);

    *joint->tmc.velocity_fb     = vactual;
    *joint->tmc.position_fb     = xactual;
    *joint->velocity_fb         = vactual_mm;
    *joint->position_fb         = xactual_mm;
    joint->last_position_fb     = xactual_mm;
    joint->last_xtarget         = xtarget;
    joint->last_move_distance   = next_move_distance;
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

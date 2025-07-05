#include "stdio.h"
#include "math.h"
#include "sys/time.h"
#include "tmc/helpers/Macros.h"
#include "tmc5041.h"
#include "rpi.h"
#include "global.h"
#include "motor.h"

// ----------------------------------------------------------------------------
// Taken from TMC5041.c and modified

void tmc5041_readWriteArray(uint8_t chip, uint8_t *data, size_t length)
{
    rpi_spi_transfernb(data, data, length);
}

int32_t tmc5041_writeDatagram(tmc5041_motor_t * motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
	uint8_t data[5] = {address | TMC5041_WRITE_BIT, x1, x2, x3, x4 };
	tmc5041_readWriteArray(*motor->chip, data, 5);
	int32_t value = ((uint32_t)x1 << 24) | ((uint32_t)x2 << 16) | (x3 << 8) | x4;
    return value;
}

/** Writes a 32bit integer to a register */
int32_t tmc5041_writeInt(tmc5041_motor_t * motor, uint8_t address, int32_t value)
{
    // return tmc5041_write_register(address, value);
    return tmc5041_writeDatagram(motor, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

/** Reads a 32bit integer from a register */
int32_t tmc5041_readInt(tmc5041_motor_t * motor, uint8_t address)
{
	uint8_t data[5] = { 0, 0, 0, 0, 0 };
	data[0] = address;
	tmc5041_readWriteArray(*motor->chip, &data[0], 5);
	data[0] = address;
	tmc5041_readWriteArray(*motor->chip, &data[0], 5);
	return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | (data[3] << 8) | data[4];
}

// Taken from TMC5041.c and modified
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Utility functions

int convert_24bit_to_32bit(int x) 
{
    // printf("convert_24bit_to_32bit: before=%d\n", x);
    // Check if the 24th bit is set (negative number)
    if (x & 0x00800000) {
        // Sign-extend by setting the upper 8 bits
        x |= 0xFF000000;
    }
    // printf("convert_24bit_to_32bit: after=%d\n", x);
    return x;
}

uint8_t tmc5041_microsteps_to_mres(uint16_t usteps)
{
    // 0 = 256 usteps = 0b0000 (default)
    // 1 = 128 usteps = 0b0001
    // 2 =  64 usteps = 0b0010
    // 3 =  32 usteps = 0b0011
    // 4 =  16 usteps = 0b0100
    // 5 =  8 usteps  = 0b0101
    // 6 =  4 usteps  = 0b0110
    // 7 =  2 usteps  = 0b0111
    // 8 =  0 usteps (fullstep)   = 0b1000

    // TODO use this instead:
    // switch(*value)
    // {
    //     case 1:    *value = 8;   break;
    //     case 2:    *value = 7;   break;
    //     case 4:    *value = 6;   break;
    //     case 8:    *value = 5;   break;
    //     case 16:   *value = 4;   break;
    //     case 32:   *value = 3;   break;
    //     case 64:   *value = 2;   break;
    //     case 128:  *value = 1;   break;
    //     case 256:  *value = 0;   break;
    //     default:   *value = -1;  break;
    // }

    if (usteps == 256)
    {
        return 0;
    }
    else if (usteps == 0)
    {
        return 0b1000;
    }

    uint8_t value = 0;
    usteps = usteps == 0 ? 1 : usteps;
    while ((usteps & 0x01) == 0)
    {
        value++;
        usteps >>= 1;
    }
    return 8 - (value > 8 ? 8 : value);
}

float64_t tmc5041_velocity_time_ref(uint32_t fclk)
{
    // Time reference t for velocities: t = 2^24 / fCLK
    // 2^24 / TMC5041_CLOCK_HZ = 16777216 / 13200000 = 1.271001212
    // so 1 time unit (t) is 1.271001212 seconds
    return (2<<(24-1)) / (float64_t)fclk;
}

float64_t tmc5041_acceleration_time_ref(uint32_t fclk)
{
    // Time reference ta² for accelerations: ta² = 2^41 / (fCLK)²
    return pow(2, 41) / (float64_t)pow(fclk, 2);
}

/**
 * Creates and initializes a new tmc5041_motor_t struct.
 * All pointers are initialized to NULL and scalar values to 0.
 * The chip and motor values are set to the provided values.
 * 
 * @param chip The chip number this motor belongs to
 * @param motor The motor number within the chip (0 or 1)
 * @return A pointer to the newly allocated and initialized motor struct
 */
tmc5041_motor_t * tmc5041_motor_create(tmc_chip_t chip, tmc_motor_t motor) {
    // Allocate memory for the struct
    tmc5041_motor_t * m = (tmc5041_motor_t *)malloc(sizeof(tmc5041_motor_t));
    if (!m) return NULL;  // Return NULL if allocation fails

    // Initialize scalar values
    m->mres = 0;
    m->last_position_cmd = 0;
    m->acceleration_cmd = 0;
    m->max_acceleration_cmd = 0;
    m->is_motor_on = false;
    m->velocity_time_ref = 0;
    m->acceleration_time_ref = 0;

    // Allocate and initialize chip and motor numbers
    m->chip = (tmc_chip_t *)malloc(sizeof(tmc_chip_t));
    m->motor = (tmc_motor_t *)malloc(sizeof(tmc_motor_t));
    // if (!m->chip || !m->motor) {
    //     // Clean up if allocation fails
    //     if (m->chip) free(m->chip);
    //     if (m->motor) free(m->motor);
    //     free(m);
    //     return NULL;
    // }
    *m->chip = chip;
    *m->motor = motor;

    // Allocate memory for command variables
    m->position_cmd = (volatile tmc_position_t *)malloc(sizeof(tmc_position_t));
    m->velocity_cmd = (volatile tmc_velocity_t *)malloc(sizeof(tmc_velocity_t));
    m->cs_thresh_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->sg_stop_cmd = (volatile bool *)malloc(sizeof(bool));
    m->run_current_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->hold_current_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->current_hold_delay_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_mode_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_a1_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_d1_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_dmax_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_vstart_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_vstop_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_v1_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->ramp_tzerowait_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->coolstep_sfilt_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->coolstep_seimin_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->coolstep_sedn_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->coolstep_seup_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->coolstep_semin_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->coolstep_semax_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_mode_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_vhigh_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_vhighchm_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_vhighfs_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_tbl_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_hend_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_hstrt_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_toff_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->chop_vsense_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->sw_en_softstop = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->sg_thresh_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->sg_trigger_thresh_cmd = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->vmax_factor_cmd = (volatile float64_t *)malloc(sizeof(float64_t));

    // Check all command variable allocations
    // if (!m->position_cmd || !m->velocity_cmd || !m->cs_thresh_cmd || 
    //     !m->sg_stop_cmd || !m->run_current_cmd || !m->hold_current_cmd ||
    //     !m->current_hold_delay_cmd || !m->ramp_mode_cmd || !m->ramp_a1_cmd ||
    //     !m->ramp_d1_cmd || !m->ramp_dmax_cmd || !m->ramp_vstart_cmd ||
    //     !m->ramp_vstop_cmd || !m->ramp_v1_cmd || !m->ramp_tzerowait_cmd ||
    //     !m->coolstep_sfilt_cmd || !m->coolstep_seimin_cmd || !m->coolstep_sedn_cmd ||
    //     !m->coolstep_seup_cmd || !m->coolstep_semin_cmd || !m->coolstep_semax_cmd ||
    //     !m->chop_mode_cmd || !m->chop_vhigh_cmd || !m->chop_vhighchm_cmd ||
    //     !m->chop_vhighfs_cmd || !m->chop_tbl_cmd || !m->chop_hend_cmd ||
    //     !m->chop_hstrt_cmd || !m->chop_toff_cmd || !m->chop_vsense_cmd ||
    //     !m->sw_en_softstop || !m->sg_thresh_cmd || !m->sg_trigger_thresh_cmd ||
    //     !m->vmax_factor_cmd) {
        
    //     // Free all allocated memory
    //     if (m->chip) free(m->chip);
    //     if (m->motor) free(m->motor);
    //     if (m->position_cmd) free((void*)m->position_cmd);
    //     if (m->velocity_cmd) free((void*)m->velocity_cmd);
    //     if (m->cs_thresh_cmd) free((void*)m->cs_thresh_cmd);
    //     if (m->sg_stop_cmd) free((void*)m->sg_stop_cmd);
    //     if (m->run_current_cmd) free((void*)m->run_current_cmd);
    //     if (m->hold_current_cmd) free((void*)m->hold_current_cmd);
    //     if (m->current_hold_delay_cmd) free((void*)m->current_hold_delay_cmd);
    //     if (m->ramp_mode_cmd) free((void*)m->ramp_mode_cmd);
    //     if (m->ramp_a1_cmd) free((void*)m->ramp_a1_cmd);
    //     if (m->ramp_d1_cmd) free((void*)m->ramp_d1_cmd);
    //     if (m->ramp_dmax_cmd) free((void*)m->ramp_dmax_cmd);
    //     if (m->ramp_vstart_cmd) free((void*)m->ramp_vstart_cmd);
    //     if (m->ramp_vstop_cmd) free((void*)m->ramp_vstop_cmd);
    //     if (m->ramp_v1_cmd) free((void*)m->ramp_v1_cmd);
    //     if (m->ramp_tzerowait_cmd) free((void*)m->ramp_tzerowait_cmd);
    //     if (m->coolstep_sfilt_cmd) free((void*)m->coolstep_sfilt_cmd);
    //     if (m->coolstep_seimin_cmd) free((void*)m->coolstep_seimin_cmd);
    //     if (m->coolstep_sedn_cmd) free((void*)m->coolstep_sedn_cmd);
    //     if (m->coolstep_seup_cmd) free((void*)m->coolstep_seup_cmd);
    //     if (m->coolstep_semin_cmd) free((void*)m->coolstep_semin_cmd);
    //     if (m->coolstep_semax_cmd) free((void*)m->coolstep_semax_cmd);
    //     if (m->chop_mode_cmd) free((void*)m->chop_mode_cmd);
    //     if (m->chop_vhigh_cmd) free((void*)m->chop_vhigh_cmd);
    //     if (m->chop_vhighchm_cmd) free((void*)m->chop_vhighchm_cmd);
    //     if (m->chop_vhighfs_cmd) free((void*)m->chop_vhighfs_cmd);
    //     if (m->chop_tbl_cmd) free((void*)m->chop_tbl_cmd);
    //     if (m->chop_hend_cmd) free((void*)m->chop_hend_cmd);
    //     if (m->chop_hstrt_cmd) free((void*)m->chop_hstrt_cmd);
    //     if (m->chop_toff_cmd) free((void*)m->chop_toff_cmd);
    //     if (m->chop_vsense_cmd) free((void*)m->chop_vsense_cmd);
    //     if (m->sw_en_softstop) free((void*)m->sw_en_softstop);
    //     if (m->sg_thresh_cmd) free((void*)m->sg_thresh_cmd);
    //     if (m->sg_trigger_thresh_cmd) free((void*)m->sg_trigger_thresh_cmd);
    //     if (m->vmax_factor_cmd) free((void*)m->vmax_factor_cmd);
    //     free(m);
    //     return NULL;
    // }

    // Initialize command values to 0
    if (m->position_cmd) *m->position_cmd = 0;
    if (m->velocity_cmd) *m->velocity_cmd = 0;
    if (m->cs_thresh_cmd) *m->cs_thresh_cmd = 0;
    if (m->sg_stop_cmd) *m->sg_stop_cmd = 0;
    if (m->run_current_cmd) *m->run_current_cmd = 0;
    if (m->hold_current_cmd) *m->hold_current_cmd = 0;
    if (m->current_hold_delay_cmd) *m->current_hold_delay_cmd = 0;
    if (m->ramp_mode_cmd) *m->ramp_mode_cmd = 0;
    if (m->ramp_a1_cmd) *m->ramp_a1_cmd = 0;
    if (m->ramp_d1_cmd) *m->ramp_d1_cmd = 0;
    if (m->ramp_dmax_cmd) *m->ramp_dmax_cmd = 0;
    if (m->ramp_vstart_cmd) *m->ramp_vstart_cmd = 0;
    if (m->ramp_vstop_cmd) *m->ramp_vstop_cmd = 0;
    if (m->ramp_v1_cmd) *m->ramp_v1_cmd = 0;
    if (m->ramp_tzerowait_cmd) *m->ramp_tzerowait_cmd = 0;
    if (m->coolstep_sfilt_cmd) *m->coolstep_sfilt_cmd = 0;
    if (m->coolstep_seimin_cmd) *m->coolstep_seimin_cmd = 0;
    if (m->coolstep_sedn_cmd) *m->coolstep_sedn_cmd = 0;
    if (m->coolstep_seup_cmd) *m->coolstep_seup_cmd = 0;
    if (m->coolstep_semin_cmd) *m->coolstep_semin_cmd = 0;
    if (m->coolstep_semax_cmd) *m->coolstep_semax_cmd = 0;
    if (m->chop_mode_cmd) *m->chop_mode_cmd = 0;
    if (m->chop_vhigh_cmd) *m->chop_vhigh_cmd = 0;
    if (m->chop_vhighchm_cmd) *m->chop_vhighchm_cmd = 0;
    if (m->chop_vhighfs_cmd) *m->chop_vhighfs_cmd = 0;
    if (m->chop_tbl_cmd) *m->chop_tbl_cmd = 0;
    if (m->chop_hend_cmd) *m->chop_hend_cmd = 0;
    if (m->chop_hstrt_cmd) *m->chop_hstrt_cmd = 0;
    if (m->chop_toff_cmd) *m->chop_toff_cmd = 0;
    if (m->chop_vsense_cmd) *m->chop_vsense_cmd = 0;
    if (m->sw_en_softstop) *m->sw_en_softstop = 0;
    if (m->sg_thresh_cmd) *m->sg_thresh_cmd = 0;
    if (m->sg_trigger_thresh_cmd) *m->sg_trigger_thresh_cmd = 0;
    if (m->vmax_factor_cmd) *m->vmax_factor_cmd = 0;

    // Allocate and initialize feedback variables
    m->velocity_reached_fb = (volatile bool *)malloc(sizeof(bool));
    m->status_sg_fb = (volatile bool *)malloc(sizeof(bool));
    m->position_reached_fb = (volatile bool *)malloc(sizeof(bool));
    m->event_pos_reached_fb = (volatile bool *)malloc(sizeof(bool));
    m->event_stop_sg_fb = (volatile bool *)malloc(sizeof(bool));
    m->event_stop_r_fb = (volatile bool *)malloc(sizeof(bool));
    m->event_stop_l_fb = (volatile bool *)malloc(sizeof(bool));
    m->status_latch_r_fb = (volatile bool *)malloc(sizeof(bool));
    m->status_latch_l_fb = (volatile bool *)malloc(sizeof(bool));
    m->status_stop_r_fb = (volatile bool *)malloc(sizeof(bool));
    m->status_stop_l_fb = (volatile bool *)malloc(sizeof(bool));
    m->motor_standstill_fb = (volatile bool *)malloc(sizeof(bool));
    m->motor_full_stepping_fb = (volatile bool *)malloc(sizeof(bool));
    m->motor_overtemp_warning_fb = (volatile bool *)malloc(sizeof(bool));
    m->motor_overtemp_alarm_fb = (volatile bool *)malloc(sizeof(bool));
    m->motor_load_fb = (volatile int32_t *)malloc(sizeof(int32_t));
    m->motor_current_fb = (volatile uint32_t *)malloc(sizeof(uint32_t));
    m->motor_stall_fb = (volatile bool *)malloc(sizeof(bool));
    m->position_fb = (volatile int32_t *)malloc(sizeof(int32_t));
    m->velocity_fb = (volatile int32_t *)malloc(sizeof(int32_t));

    // Initialize all feedback values to 0
    if (m->velocity_reached_fb) *m->velocity_reached_fb = 0;
    if (m->status_sg_fb) *m->status_sg_fb = 0;
    if (m->position_reached_fb) *m->position_reached_fb = 0;
    if (m->event_pos_reached_fb) *m->event_pos_reached_fb = 0;
    if (m->event_stop_sg_fb) *m->event_stop_sg_fb = 0;
    if (m->event_stop_r_fb) *m->event_stop_r_fb = 0;
    if (m->event_stop_l_fb) *m->event_stop_l_fb = 0;
    if (m->status_latch_r_fb) *m->status_latch_r_fb = 0;
    if (m->status_latch_l_fb) *m->status_latch_l_fb = 0;
    if (m->status_stop_r_fb) *m->status_stop_r_fb = 0;
    if (m->status_stop_l_fb) *m->status_stop_l_fb = 0;
    if (m->motor_standstill_fb) *m->motor_standstill_fb = 0;
    if (m->motor_full_stepping_fb) *m->motor_full_stepping_fb = 0;
    if (m->motor_overtemp_warning_fb) *m->motor_overtemp_warning_fb = 0;
    if (m->motor_overtemp_alarm_fb) *m->motor_overtemp_alarm_fb = 0;
    if (m->motor_load_fb) *m->motor_load_fb = 0;
    if (m->motor_current_fb) *m->motor_current_fb = 0;
    if (m->motor_stall_fb) *m->motor_stall_fb = 0;
    if (m->position_fb) *m->position_fb = 0;
    if (m->velocity_fb) *m->velocity_fb = 0;

    return m;
}

/**
 * Multiply each velocity value (in machine units per second) with this factor
 * to normalize the velocity to steps per second. 
 * 
 * Based on 17.1 Using the Internal Clock in TMC5041 datasheet.
 * Make sure to SPI select the correct chip before running this function.
 * At a nominal value of the internal clock frequency, 780 steps will be done in 100ms.
 */
float64_t tmc5041_frequency_scaling(tmc5041_motor_t * motor)
{
    
    // 1. You may leave the motor driver disabled during the calibration. 
    motor_end(motor);

    // 2. Start  motor  in  velocity  mode,  with  VMAX=10000  and  AMAX=60000
    // (for  quick  acceleration).  The acceleration phase is ended after a few ms.
    int32_t vmax = 10000;
    int32_t amax = 60000;
    tmc5041_set_register_RAMPMODE(motor, 1); // 1: Velocity mode to positive VMAX
    tmc5041_set_register_VMAX(motor, vmax);
    tmc5041_set_register_AMAX(motor, amax);
    // Wait 10ms
    struct timeval tv_1;
    int wait_1_ms = 10;
    tv_1.tv_sec = wait_1_ms / 1000;
    tv_1.tv_usec = (wait_1_ms % 1000) * 1000;
    select(0, NULL, NULL, NULL, &tv_1);  // No file descriptors, just timeout
    #ifdef DEBUG_SCALING
    printf("hotshot(%d,%d)[scaling]: VMAX=%d, AMAX=%d. Waited %d ms.\n", 
                *motor->chip, *motor->motor, vmax, amax, wait_1_ms);
    #endif

    // 3. Read out XACTUAL twice, at time point t1 and time point t2, e.g. 100ms 
    // later (dt=0.1s). The time difference between both read accesses shall be 
    // exactly timed by the external microcontroller.
    int32_t xactual_t1 = tmc5041_get_register_XACTUAL(motor);
    // Record current time in ms
    struct timeval tv_2;
    gettimeofday(&tv_2, NULL);
    float64_t t1 = (tv_2.tv_sec * 1000LL) + (tv_2.tv_usec / 1000); // Convert seconds to ms and microseconds to ms
    // Wait 100ms
    struct timeval tv_3;
    int wait_2_ms = 100;
    tv_3.tv_sec = wait_2_ms / 1000;
    tv_3.tv_usec = (wait_2_ms % 1000) * 1000;
    select(0, NULL, NULL, NULL, &tv_3);  // No file descriptors, just timeout
    // Again record current time in ms
    struct timeval tv_4;
    gettimeofday(&tv_4, NULL);
    float64_t t2 = (tv_4.tv_sec * 1000LL) + (tv_4.tv_usec / 1000); // Convert seconds to ms and microseconds to ms
    // Calculate exact duration that passed in ms
    float64_t dt = (t2 - t1) / 1000; // in sec
    int32_t xactual_t2 = tmc5041_get_register_XACTUAL(motor);
    #ifdef DEBUG_SCALING
    int32_t actual_steps = xactual_t2 - xactual_t1;
    printf("hotshot(%d,%d)[scaling]: Moved actual steps = %d. Waited sec: target=%f, actual=%f.\n", 
                *motor->chip, *motor->motor, actual_steps, (double)wait_2_ms/1000, dt);
    #endif

    // 4. Stop the motion ramp by setting VMAX=0.
    tmc5041_set_register_VMAX(motor, 0);
    // TODO restore AMAX
    // tmc5041_set_register_AMAX(motor, 0);
    // TODO restore driver on/off state
    // tmc5041_set_register_RAMPMODE(motor, 0); // 0: hold
    // motor_end(motor);
    // Wait 10ms
    struct timeval tv_5;
    int wait_10_ms = 10;
    tv_5.tv_sec = wait_1_ms / 1000;
    tv_5.tv_usec = (wait_1_ms % 1000) * 1000;
    select(0, NULL, NULL, NULL, &tv_5);  // No file descriptors, just timeout
    int32_t vmax_actual = tmc5041_get_register_VACTUAL(motor);
    #ifdef DEBUG_SCALING
    printf("hotshot(%d,%d)[scaling]: Stopped motion ramp. Now VACTUAL=%d, AMAX=%d, RAMPMODE=%d.\n", 
                *motor->chip, *motor->motor, vmax_actual, 0, 0);
    #endif

    // 5. The number of steps done in between of t1 and t2 now can be used to 
    // calculate the factor
    // f = (vmax * dt) / (xactual(t2) - xactual(t1))
    float64_t f = (vmax * dt) / (xactual_t2 - xactual_t1);
    #ifdef DEBUG_SCALING
    printf("hotshot(%d,%d)[scaling]: Scaling factor is %f.\n", 
                *motor->chip, *motor->motor, f);
    #endif

    return f;
}

// Utility functions
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Register access

void tmc5041_set_register_RAMPMODE(tmc5041_motor_t * motor, int32_t rampmode)
{
    // 0: Positioning mode (using all A, D and V parameters)
    // 1: Velocity mode to positive VMAX (using AMAX acceleration)
    // 2: Velocity mode to negative VMAX (using AMAX acceleration)
    // 3: Hold mode (velocity remains unchanged, unless stop event occurs)
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_RAMPMODE_MASK, TMC5041_RAMPMODE_SHIFT, rampmode);
    uint8_t rampmode_message[40] = {TMC5041_RAMPMODE(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(rampmode_message, spi_status, 5);
}

void tmc5041_set_register_XACTUAL(tmc5041_motor_t * motor, int32 xactual)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;
    // XACTUAL
    write_payload = FIELD_SET(write_payload, TMC5041_XACTUAL_MASK, TMC5041_XACTUAL_SHIFT, xactual);
    uint8_t xtarget_message[40] = {TMC5041_XACTUAL(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(xtarget_message, spi_status, 5);
}

void tmc5041_set_register_IHOLD_IRUN(tmc5041_motor_t * motor, uint32_t ihold, uint32_t irun) 
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    int32_t value = 0x00;
    value = FIELD_SET(value, TMC5041_IHOLD_MASK, TMC5041_IHOLD_SHIFT, ihold);
    value = FIELD_SET(value, TMC5041_IRUN_MASK, TMC5041_IRUN_SHIFT, irun);
    tmc5041_writeInt(motor, TMC5041_IHOLD_IRUN(*motor->motor), value);
}

int32_t tmc5041_get_register_XACTUAL(tmc5041_motor_t * motor)
{
    // XACTUAL: Actual motor position (signed)
    return tmc5041_readInt(motor, TMC5041_XACTUAL(*motor->motor));
}

void tmc5041_set_register_VCOOLTHRS(tmc5041_motor_t * motor, int32_t vcoolthrs)
{
    //
    // This is the lower threshold velocity for switching on smart
    // energy CoolStep and StallGuard feature. Further it is the upper
    // operation velocity for StealthChop.
    // Hint: May be adapted to disable CoolStep during acceleration and deceleration phase by setting identical to VMAX.
    // TMC5041 data sheet uses 30 RPM (20 mm/sec on X axis)
    //
    // Lower ramp generator velocity threshold. Below this velocity CoolStep and StallGuard becomes disabled (not used
    // in Step/Dir mode). Adapt to the lower limit of the velocity range where StallGuard2 gives a stable result
    //
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VCOOLTHRS_MASK, TMC5041_VCOOLTHRS_SHIFT, vcoolthrs);
    uint8_t vcoolthrs_message[40] = {TMC5041_VCOOLTHRS(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(vcoolthrs_message, vcoolthrs_message, 5);
}

void tmc5041_set_register_VMAX(tmc5041_motor_t * motor, int32_t vmax) 
{
    // VMAX: Motion ramp target velocity
    //
    // This is the target velocity [in µsteps / t] in velocity mode. It can be changed any time during a motion
    //
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VMAX_MASK, TMC5041_VMAX_SHIFT, vmax);
    uint8_t vmax_message[40] = {TMC5041_VMAX(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(vmax_message, vmax_message, 5);
}

void tmc5041_set_register_AMAX(tmc5041_motor_t * motor, int32_t amax) 
{
    // AMAX
    // Maximum acceleration/deceleration [µsteps / ta²]
    //
    // This is the acceleration and deceleration value for velocity mode.
    // In position mode (RAMP=0), must be lower than A1 (???)
    //
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_AMAX_MASK, TMC5041_AMAX_SHIFT, motor->max_acceleration_cmd);
    uint8_t amax_message[40] = {TMC5041_AMAX(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(amax_message, amax_message, 5);
}

void tmc5041_set_register_VSTART(tmc5041_motor_t * motor, int32_t vstart) 
{
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTART_MASK, TMC5041_VSTART_SHIFT, vstart);
    uint8_t vstart_message[40] = {TMC5041_VSTART(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(vstart_message, vstart_message, 5);
}

void tmc5041_set_register_VSTOP(tmc5041_motor_t * motor, int32_t vstop) 
{
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTOP_MASK, TMC5041_VSTOP_SHIFT, vstop);
    uint8_t vstop_message[40] = {TMC5041_VSTOP(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(vstop_message, vstop_message, 5);
}

void tmc5041_push_register_SW_MODE(tmc5041_motor_t * motor)
{
    // SW_MODE: Reference Switch & StallGuard2 Event Configuration Register
    //
    int32_t write_payload = 0x00;
    // Attention: Do not use soft stop in combination with StallGuard2.
    write_payload = FIELD_SET(write_payload, TMC5041_EN_SOFTSTOP_MASK, TMC5041_EN_SOFTSTOP_SHIFT, *motor->sw_en_softstop);
    // Note: set VCOOLTHRS to a suitable value before enabling this
    write_payload = FIELD_SET(write_payload, TMC5041_SG_STOP_MASK, TMC5041_SG_STOP_SHIFT, *motor->sg_stop_cmd);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_R_INACTIVE_MASK, TMC5041_LATCH_R_INACTIVE_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_R_ACTIVE_MASK, TMC5041_LATCH_R_ACTIVE_SHIFT, 1);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_L_INACTIVE_MASK, TMC5041_LATCH_L_INACTIVE_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_L_ACTIVE_MASK, TMC5041_LATCH_L_ACTIVE_SHIFT, 1);
    write_payload = FIELD_SET(write_payload, TMC5041_SWAP_LR_MASK, TMC5041_SWAP_LR_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_POL_STOP_R_MASK, TMC5041_POL_STOP_R_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_POL_STOP_L_MASK, TMC5041_POL_STOP_L_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_STOP_R_ENABLE_MASK, TMC5041_STOP_R_ENABLE_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_STOP_L_ENABLE_MASK, TMC5041_STOP_L_ENABLE_SHIFT, 0);
    uint8_t swmode[40] = {TMC5041_SWMODE(*motor->motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(swmode, swmode, 5);

}

void tmc5041_push_register_COOLCONF(tmc5041_motor_t * motor)
{
    // COOLCONF: Smart Energy Control CoolStep and StallGuard2
    //
    // When the load increases, SG falls below SEMIN, and CoolStep increases the current.
    // When the load decreases, SG rises above (SEMIN + SEMAX + 1) * 32, and the current is reduced.
    //
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_SFILT_MASK, TMC5041_SFILT_SHIFT, *motor->coolstep_sfilt_cmd);
    write_payload = FIELD_SET(write_payload, TMC5041_SGT_MASK, TMC5041_SGT_SHIFT, *motor->sg_thresh_cmd);
    write_payload = FIELD_SET(write_payload, TMC5041_SEIMIN_MASK, TMC5041_SEIMIN_SHIFT, *motor->coolstep_seimin_cmd);
    write_payload = FIELD_SET(write_payload, TMC5041_SEUP_MASK, TMC5041_SEUP_SHIFT, *motor->coolstep_seup_cmd);
    write_payload = FIELD_SET(write_payload, TMC5041_SEDN_MASK, TMC5041_SEDN_SHIFT, *motor->coolstep_sedn_cmd);
    // coolstep deactivated when SG >= (SEMIN+SEMAX+1)*32
    write_payload = FIELD_SET(write_payload, TMC5041_SEMAX_MASK, TMC5041_SEMAX_SHIFT, *motor->coolstep_semax_cmd);
    // coolstep activated when SG < SEMIN*32
    write_payload = FIELD_SET(write_payload, TMC5041_SEMIN_MASK, TMC5041_SEMIN_SHIFT, *motor->coolstep_semin_cmd);
    uint8_t coolconf[40] = {TMC5041_COOLCONF(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(coolconf, coolconf, 5);

}

void tmc5041_pull_register_DRV_STATUS(tmc5041_motor_t * motor)
{
    drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(motor);
    *motor->motor_standstill_fb = drv_status.standstill;
    *motor->motor_full_stepping_fb = drv_status.full_stepping;
    *motor->motor_overtemp_warning_fb = drv_status.overtemp_warning;
    *motor->motor_overtemp_alarm_fb = drv_status.overtemp_alarm;
    *motor->motor_load_fb = drv_status.sg_result;
    *motor->motor_current_fb = drv_status.cs_actual;
    *motor->motor_stall_fb = drv_status.sg_status;
}

void tmc5041_push_register_IHOLD_IRUN(tmc5041_motor_t * motor) {
    // IHOLD_IRUN: Current Setting
    //
    int32_t write_payload = 0x00;
    // IRUN: Current scale when motor is running (scaling factor N/32 i.e. 1/32, 2/32, … 31/32)
    // For high precision motor operation, work with a current scaling factor in the range 16 to 31,
    // because scaling down the current values reduces the effective microstep resolution by making microsteps coarser.
    write_payload = FIELD_SET(write_payload, TMC5041_IRUN_MASK, TMC5041_IRUN_SHIFT, *motor->run_current_cmd);
    // IHOLD: Identical to IRUN, but for motor in stand still.
    write_payload = FIELD_SET(write_payload, TMC5041_IHOLD_MASK, TMC5041_IHOLD_SHIFT, *motor->hold_current_cmd);
    // IHOLDDELAY: 0 = instant IHOLD
    write_payload = FIELD_SET(write_payload, TMC5041_IHOLDDELAY_MASK, TMC5041_IHOLDDELAY_SHIFT, *motor->current_hold_delay_cmd);
    uint8_t current[40] = {TMC5041_IHOLD_IRUN(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(current, current, 5);
}

void tmc5041_push_register_CHOPCONF(tmc5041_motor_t * motor) {
    // CHOPCONF: Chopper Configuration (i.e. SpreadCycle)
    //
    int32_t write_payload = 0x00;
    // MRES: micro step resolution
    write_payload = FIELD_SET(write_payload, TMC5041_MRES_MASK, TMC5041_MRES_SHIFT, motor->mres);
    // vhighchm: high velocity chopper mode
    write_payload = FIELD_SET(write_payload, TMC5041_VHIGHCHM_MASK, TMC5041_VHIGHCHM_SHIFT, *motor->chop_vhighchm_cmd);
    // vhighfs: high velocity fullstep selection
    write_payload = FIELD_SET(write_payload, TMC5041_VHIGHFS_MASK, TMC5041_VHIGHFS_SHIFT, *motor->chop_vhighfs_cmd);
    // VSENSE: sense resistor voltage based current scaling
    //  0: Low sensitivity, high sense resistor voltage
    //  1: High sensitivity, low sense resistor voltage
    write_payload = FIELD_SET(write_payload, TMC5041_VSENSE_MASK, TMC5041_VSENSE_SHIFT, *motor->chop_vsense_cmd);
    // TBL: blank time select
    write_payload = FIELD_SET(write_payload, TMC5041_TBL_MASK, TMC5041_TBL_SHIFT, *motor->chop_tbl_cmd);
    // CHM: chopper mode 
    write_payload = FIELD_SET(write_payload, TMC5041_CHM_MASK, TMC5041_CHM_SHIFT, *motor->chop_mode_cmd);
    // TODO rndtf
    // TODO disfdcc
    // HEND: hysteresis low value OFFSET sine wave offset
    write_payload = FIELD_SET(write_payload, TMC5041_HEND_MASK, TMC5041_HEND_SHIFT, *motor->chop_hend_cmd);
    // HSTRT: hysteresis start value added to HEND
    write_payload = FIELD_SET(write_payload, TMC5041_HSTRT_MASK, TMC5041_HSTRT_SHIFT, *motor->chop_hstrt_cmd);
    // TOFF: off time and driver enable
    write_payload = FIELD_SET(write_payload, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, *motor->chop_toff_cmd);
    // Start chopper in off mode. We will set this when we turn the motor on.
    // write_payload = FIELD_SET(write_payload, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, 0);
    uint8_t chop_conf[40] = {TMC5041_CHOPCONF(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(chop_conf, chop_conf, 5);
}

void tmc5041_set_register_XTARGET(tmc5041_motor_t * motor, int32 xtarget)
{
    uint32_t write_payload = 0x00;
    // Target position for RAMPMODE=0 (signed).
    // Write a new target position to this register in order to activate the ramp generator positioning in RAMPMODE=0.
    // Initialize all velocity, acceleration and deceleration parameters before.
    //
    // XTARGET
    write_payload = FIELD_SET(write_payload, TMC5041_XTARGET_MASK, TMC5041_XTARGET_SHIFT, xtarget);
    uint8_t message[40] = {TMC5041_XTARGET(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(message, message, 5);
}

int32_t tmc5041_get_register_VACTUAL(tmc5041_motor_t * motor)
{
    // Actual motor velocity from ramp generator (signed)
    int32_t value = tmc5041_readInt(motor, TMC5041_VACTUAL(*motor->motor));
    return convert_24bit_to_32bit(value);
}

ramp_stat_register_t tmc5041_get_register_RAMP_STAT(tmc5041_motor_t * motor)
{
    // Reading the register will clear the stall condition and the motor may
    // re-start motion, unless the motion controller has been stopped.
    // (Flag and interrupt condition are cleared upon reading)
    // This bit is ORed to the interrupt output signal

    int32_t reply = tmc5041_readInt(motor, TMC5041_RAMPSTAT(*motor->motor));

    ramp_stat_register_t reg;
    reg.status_sg          = FIELD_GET(reply, TMC5041_STATUS_SG_MASK, TMC5041_STATUS_SG_SHIFT);
    reg.second_move        = FIELD_GET(reply, TMC5041_SECOND_MOVE_MASK, TMC5041_SECOND_MOVE_SHIFT);
    reg.t_zerowait_active  = FIELD_GET(reply, TMC5041_T_ZEROWAIT_ACTIVE_MASK, TMC5041_T_ZEROWAIT_ACTIVE_SHIFT);
    reg.vzero              = FIELD_GET(reply, TMC5041_VZERO_MASK, TMC5041_VZERO_SHIFT);
    reg.position_reached   = FIELD_GET(reply, TMC5041_POSITION_REACHED_MASK, TMC5041_POSITION_REACHED_SHIFT);
    reg.velocity_reached   = FIELD_GET(reply, TMC5041_VELOCITY_REACHED_MASK, TMC5041_VELOCITY_REACHED_SHIFT);
    reg.event_pos_reached  = FIELD_GET(reply, TMC5041_EVENT_POS_REACHED_MASK, TMC5041_EVENT_POS_REACHED_SHIFT);
    reg.event_stop_sg      = FIELD_GET(reply, TMC5041_EVENT_STOP_SG_MASK, TMC5041_EVENT_STOP_SG_SHIFT);
    reg.event_stop_r       = FIELD_GET(reply, TMC5041_EVENT_STOP_R_MASK, TMC5041_EVENT_STOP_R_SHIFT);
    reg.event_stop_l       = FIELD_GET(reply, TMC5041_EVENT_STOP_L_MASK, TMC5041_EVENT_STOP_L_SHIFT);
    reg.status_latch_r     = FIELD_GET(reply, TMC5041_STATUS_LATCH_R_MASK, TMC5041_STATUS_LATCH_R_SHIFT);
    reg.status_latch_l     = FIELD_GET(reply, TMC5041_STATUS_LATCH_L_MASK, TMC5041_STATUS_LATCH_L_SHIFT);
    reg.status_stop_r      = FIELD_GET(reply, TMC5041_STATUS_STOP_R_MASK, TMC5041_STATUS_STOP_R_SHIFT);
    reg.status_stop_l      = FIELD_GET(reply, TMC5041_STATUS_STOP_L_MASK, TMC5041_STATUS_STOP_L_SHIFT);

    return reg;
}

void tmc5041_pull_register_RAMP_STAT(tmc5041_motor_t * motor)
{
    ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(motor);
    *motor->velocity_reached_fb = ramp_stat.velocity_reached;
    *motor->position_reached_fb = ramp_stat.position_reached;
    *motor->status_sg_fb = ramp_stat.status_sg;
    *motor->event_pos_reached_fb = ramp_stat.event_pos_reached;
    *motor->event_stop_sg_fb = ramp_stat.event_stop_sg;
    *motor->event_stop_r_fb = ramp_stat.event_stop_r;
    *motor->event_stop_l_fb = ramp_stat.event_stop_l;
    *motor->status_latch_r_fb = ramp_stat.status_latch_r;
    *motor->status_latch_l_fb = ramp_stat.status_latch_l;
    *motor->status_stop_r_fb = ramp_stat.status_stop_r;
    *motor->status_stop_l_fb = ramp_stat.status_stop_l;
}

drv_status_register_t tmc5041_get_register_DRV_STATUS(tmc5041_motor_t * motor)
{
    int32_t reply = tmc5041_readInt(motor, TMC5041_DRVSTATUS(*motor->motor));

    drv_status_register_t reg;
    reg.standstill = FIELD_GET(reply, TMC5041_STST_MASK, TMC5041_STST_SHIFT);
    reg.overtemp_warning = FIELD_GET(reply, TMC5041_OTPW_MASK, TMC5041_OTPW_SHIFT);
    reg.overtemp_alarm = FIELD_GET(reply, TMC5041_OT_MASK, TMC5041_OT_SHIFT);
    reg.sg_result = FIELD_GET(reply, TMC5041_SG_RESULT_MASK, TMC5041_SG_RESULT_SHIFT);
    reg.cs_actual = FIELD_GET(reply, TMC5041_CS_ACTUAL_MASK, TMC5041_CS_ACTUAL_SHIFT);
    reg.sg_status = FIELD_GET(reply, TMC5041_STALLGUARD_MASK, TMC5041_STALLGUARD_SHIFT);
    reg.full_stepping = FIELD_GET(reply, TMC5041_FSACTIVE_MASK, TMC5041_FSACTIVE_SHIFT);
    return reg;
}

chopconf_register_t tmc5041_get_register_CHOPCONF(tmc5041_motor_t * motor) {
    int32_t reply = tmc5041_readInt(motor, TMC5041_CHOPCONF(*motor->motor));

    chopconf_register_t reg;
    reg.mres = FIELD_GET(reply, TMC5041_MRES_MASK, TMC5041_MRES_SHIFT);

    return reg;
}

void tmc5041_pull_register_CHOPCONF(tmc5041_motor_t * motor) {
    chopconf_register_t reg = tmc5041_get_register_CHOPCONF(motor);
    motor->mres = reg.mres;
}

int32_t tmc5041_get_register_XLATCH(tmc5041_motor_t * motor) {
    return tmc5041_readInt(motor, TMC5041_XLATCH(*motor->motor));
}

// Register access
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// motor.h interface

void motor_set_velocity(tmc5041_motor_t * motor, motor_velocity_t vmax)
{
    *motor->velocity_cmd = vmax;
}

int32_t motor_get_velocity(tmc5041_motor_t * motor)
{
    // return tmc5041_get_register_VACTUAL(motor) / motor->velocity_time_ref;
    return *motor->velocity_fb;
}

motor_load_t motor_get_load(tmc5041_motor_t * motor)
{
    return *motor->motor_load_fb;
}

int32_t motor_get_position(tmc5041_motor_t * motor)
{
    // return tmc5041_get_register_XACTUAL(motor);
    return *motor->position_fb;
}

void motor_stop(tmc5041_motor_t * motor)
{
    // OPTIONS TO TERMINATE MOTION USING ACCELERATION SETTINGS:
    // a)
    // Switch to velocity mode
    // set VMAX=0 a
    tmc5041_set_register_VMAX(motor, 0);   
}

void motor_homed(tmc5041_motor_t * motor)
{
    // Disable stallguard stop on stall
    tmc5041_push_register_SW_MODE(motor);
    // Switch the ramp generator to hold mode
    tmc5041_set_register_RAMPMODE(motor, TMC5041_MODE_HOLD);
    // TODO and calculate the difference between the latched position and the actual position.
    //      For StallGuard based homing or when using hard stop, XACTUAL stops exactly at the home position, so there is no difference (0).
    // Write the calculated difference into the actual position register.
    tmc5041_set_register_XACTUAL(motor, 0);
    tmc5041_set_register_XTARGET(motor, 0);
    // Now, homing is finished. A move to position 0 will bring back the motor exactly to the switching point.
    // In case StallGuard was used for homing, a read access to RAMP_STAT clears the 
    // StallGuard stop event event_stop_sg and releases the motor from the stop condition.
    tmc5041_get_register_RAMP_STAT(motor);
    // Switch back into positioning mode
    // tmc5041_set_register_RAMPMODE(motor, TMC5041_MODE_POSITION);
}

void motor_set_on(tmc5041_motor_t * motor)
{
    motor->is_motor_on = TRUE;
}

void motor_set_off(tmc5041_motor_t * motor)
{
    motor->is_motor_on = FALSE;
}

// TODO use CHOPCONF register struct here
void motor_on(tmc5041_motor_t * motor)
{
    // Power motor up after tmc5041_motor_off()
    // If we power a motor off, then back on, the XACTUAL is some random number.
    // This sort of makes sense because once a motor has been powered off we can no
    // longer guarantee it's position. So sync XACTUAL with what EMC has commanded.
    int32_t chopconf = tmc5041_readInt(motor, TMC5041_CHOPCONF(*motor->motor));
    chopconf = FIELD_SET(chopconf, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, *motor->chop_toff_cmd);
    tmc5041_writeInt(motor, TMC5041_CHOPCONF(*motor->motor), chopconf);
    
    motor->is_motor_on = TRUE;
}

// TODO use CHOPCONF register struct here
void motor_off(tmc5041_motor_t * motor)
{
    int32_t chopconf = tmc5041_readInt(motor, TMC5041_CHOPCONF(*motor->motor));
    chopconf = FIELD_SET(chopconf, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, 0); // 0 = off
    tmc5041_writeInt(motor, TMC5041_CHOPCONF(*motor->motor), chopconf);
    motor->is_motor_on = FALSE;
}

void motor_rotate(tmc5041_motor_t * motor, motor_dir_t dir) 
{
     // 1: Velocity mode = positive VMAX
    tmc5041_set_register_RAMPMODE(motor, dir);
}

/** Read/write to/from motor driver over SPI bus.
 * No math should happen in this function, or in any functions it calls.
 */
void motor_update(tmc5041_motor_t * motor)
{

    //
    // Writes
    //
    // Turn motor on or off
    if (motor->is_motor_on == TRUE)
    {
        motor_on(motor);
    }
    else
    {
        // LinuxCNC power button is off, so power motor off
        // FIXME it's possible for driver to keep counting steps even after motor_off
        motor_stop(motor);            
        motor_off(motor);
    }

    // Set turn direction
    //  1: Velocity mode to positive VMAX (using AMAX acceleration)
    //  2: Velocity mode to negative VMAX (using AMAX acceleration)
    if (motor->velocity_cmd > 0)
        tmc5041_set_register_RAMPMODE(motor, 1);
    else if (motor->velocity_cmd < 0)
        tmc5041_set_register_RAMPMODE(motor, 2);
    // else vmax == 0. do nothing while decelaration ramp finishes
    // Set velocity
    // VMAX is defined as an unsigned int in the datasheet, so it must be absolute
    tmc5041_set_register_VMAX(motor, abs((*motor->velocity_cmd) * motor->velocity_time_ref));

    // TODO move all math to hotshot_handle_move
    //
    // Reads
    //
    // Driver status
    tmc5041_pull_register_DRV_STATUS(motor);
    // Position
    // *motor->position_fb = motor_get_position(motor);
    *motor->position_fb = tmc5041_get_register_XACTUAL(motor);
    // TODO can we do math in this function since it should be fast?
    // *joint->position_fb     = PULSES_TO_UNITS(*joint->tmc.position_fb, joint->unit_pulse_factor);
    // Velocity
    // *motor->velocity_fb  = motor_get_velocity(motor);
    *motor->velocity_fb  = tmc5041_get_register_VACTUAL(motor) / motor->velocity_time_ref;
    // *motor->velocity_fb  = tmc5041_get_register_VACTUAL(motor);
    // TODO can we do math in this function since it should be fast?
    // *joint->velocity_fb      = PULSES_TO_UNITS(*joint->tmc.velocity_fb, joint->unit_pulse_factor);
    // Stallguard threshold
    tmc5041_push_register_COOLCONF(motor);

}

/**
 * Intialize necessary motor registers from motor configuration.
 * Should only be called once at beginning of program.
 */
void motor_init(tmc5041_motor_t * motor)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;

    // Calculate frequency scaling
    float64_t scale_factor = tmc5041_frequency_scaling(motor);
    motor->velocity_time_ref = scale_factor;
    motor->acceleration_time_ref = tmc5041_acceleration_time_ref(TMC5041_CLOCK_HZ);

    // Prevent unexpected moves before we do anything else
    // Always start in hold mode to prevent unexpected movement
    tmc5041_set_register_VMAX(motor, 0);
    tmc5041_set_register_RAMPMODE(motor, 0);
    
    // Reset position registers in case they are dirty after restarting LinuxCNC
    // TODO or should we sync them up with LinuxCNC on startup?
    tmc5041_set_register_XTARGET(motor, 0);
    tmc5041_set_register_XACTUAL(motor, 0);

    // Power Configuration
    //
    // IHOLD_IRUN: Current Setting
    tmc5041_push_register_IHOLD_IRUN(motor);

    // Chopper configuration
    //
    // CHOPCONF: Chopper Configuration (i.e. SpreadCycle)
    tmc5041_push_register_CHOPCONF(motor);
    // Setting TOFF != 0 turns motor on, so set flag
    motor->is_motor_on = TRUE;
    // VCOOLTHRS
    tmc5041_set_register_VCOOLTHRS(motor, *motor->cs_thresh_cmd);
    //
    // VHIGH
    //
    // This is the lower threshold velocity for switching on smart
    // energy CoolStep and StallGuard feature. Further it is the upper
    // operation velocity for StealthChop.
    // Hint: May be adapted to disable CoolStep during acceleration and deceleration phase by setting identical to VMAX.
    // Enable CoolStep and StallGuard at 5mm per second
    // TMC5041 data sheet uses 30 RPM (20 mm/sec on X axis)
    // 
    // This velocity setting allows velocity dependent switching into
    // a different chopper mode and fullstepping to maximize torque.
    // 
    // Upper ramp generator velocity threshold value. Above this velocity CoolStep becomes disabled
    // (not used in Step/Dir mode). Adapt to the velocity range where StallGuard2 gives a stable result.
    //
    write_payload = 0x00;
    // VHIGH: Set high values for both
    write_payload = FIELD_SET(write_payload, TMC5041_VHIGH_MASK, TMC5041_VHIGH_SHIFT, *motor->chop_vhigh_cmd);
    uint8_t vhigh_message[40] = {TMC5041_VHIGH(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(vhigh_message, spi_status, 5);

    // COOLCONF: Smart Energy Control CoolStep and StallGuard2
    tmc5041_push_register_COOLCONF(motor);

    //
    // Switch Configuration
    //
    // SW_MODE: Reference Switch & StallGuard2 Event Configuration Register
    tmc5041_push_register_SW_MODE(motor);

    //
    // Ramp Configuration
    //

    // AMAX
    tmc5041_set_register_AMAX(motor, motor->max_acceleration_cmd);

    // TZEROWAIT
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_TZEROWAIT_MASK, TMC5041_TZEROWAIT_SHIFT, *motor->ramp_tzerowait_cmd);
    uint8_t tzerowait_message[40] = {TMC5041_TZEROWAIT(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    rpi_spi_transfernb(tzerowait_message, spi_status, 5);

}

void motor_end(tmc5041_motor_t * motor)
{

    // Stop chopper
    motor_off(motor);
    // Clear stallguard
    // tmc5041_motor_clear_stall(motor);
    // Clear stallguard with by reading RAMP_STAT register
    tmc5041_pull_register_RAMP_STAT(motor);
    // Reset XACTUAL to 0
    motor_homed(motor);
}

// TODO motor_load

// motor.h interface
// ----------------------------------------------------------------------------

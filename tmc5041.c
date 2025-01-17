#include "stdio.h"
#include "math.h"
#include "sys/time.h"
#include "bcm2835.h"
#include "rpi.h"
#include "tmc/helpers/Macros.h"
#include "tmc5041.h"

/**
 * Position
 * --------
 * 
 * Position in LinuxCNC is represented as machine units per second (mm or in, i.e. {mm,in}).
 * Values must be converted back and forth using a units-to-usteps conversion factor.
 * 
 * For belt driven systems
 *      steps per unit [{mm,in}] = (motor fullsteps per revolution * microsteps) / pulley circumference [{mm,in}]
 * For leadscrew driven systems
 *      steps per unit [{mm,in}] = (motor fullsteps per revolution * microsteps) / leadscrew pitch [{mm,in}]
 * 
 * Given the following parameters:
 * - Gear pitch ({mm,in}) is tooth spacing. A GT2 belt is
 *      pitch = 2
 * - Number of teeth on the gear
 *      teeth = 20
 * - Full steps per motor revolution. A 1.8 degree stepper is
 *      fs_per_rev = 360 / 1.8 = 200
 * - Chosen microsteps between each step. TMC5041 is by default
 *      microsteps = 256 
 * 
 * Calculate how many linear machine units ({mm,in}) are traveled in one motor revolution
 *      units_per_rev = pitch * teeth = 2 * 20 = 40 [mm]
 * Calculate how many microsteps are needed to turn 1 revolution
 *      usteps_per_rev = fs_per_rev * microsteps = 200 * 256 = 51200 [ustep]
 * 
 * Then calculate how many linear machine units ({mm,in}) are traveled per ustep
 *      units_to_usteps = usteps_per_rev / units_per_rev = 51200 / 40 = 1280
 * and calculate the inverse as well
 *      usteps_to_units = units_per_rev / usteps_per_rev = 40 / 51200 = 0.00078125
 * 
 * To convert from µsteps to machine units ({mm,in}) 
 *      units = usteps * usteps_to_units = 500 * 0.00078125 = 0.390625 [{mm,in}]
 * To convert from machine units ({mm,in}) to µsteps
 *      µsteps = units * units_to_usteps = 0.390625 * 1280 = 500 [usteps]
 * 
 * When configuring microstepping, µsteps must be converted to MRES with 
 * tmc5041_microsteps_to_mres() before writing to a TMC5041 register.
 * 
 * Velocity
 * --------
 * 
 * TMC5041 velocity is defined as
 *      µsteps / t
 * where µsteps is microsteps one of
 *      1,2,4,8,16,32,64,128,256
 * and t is 
 *      t = 2^24 / fCLK
 * 
 * Machine units ({mm, in}) are converted to usteps as detailed in the Position section.
 * 
 * fCLK is the clock frequency of the TMC5041 in Hz. All time, velocity and 
 * acceleration settings are referenced to fCLK. fCLK can come from the internal clock, 
 * or from an external clock depending on if the CLK16 pin is grounded or not.
 * 
 * When using the internal clock, a conversion factor (f) must be empirically found
 * using tmc5041_frequency_scaling() that we can use to convert between seconds and t.
 * Conceptually a 'second' for the TMC5041 using the internal clock is about 1.3 
 * real seconds long. Thus the need for converting between seconds and t.
 * 
 * Velocity in µsteps/sec must be converted to µsteps/t before writing to TMC5041 registers
 * using a conversion factor (f)
 *      f = ( velocity [µsteps/t] * change in time [sec] ) / change in position [µsteps]
 * which is
 *      f = (vmax * dt) / (xactual(t2) - xactual(t1))
 * where
 *      expected change in position [µsteps] = (velocity [µsteps/t] * change in time [sec])
 *      actual change in position [µsteps] = (xactual(t2) - xactual(t1))
 * 
 * To convert from µsteps/sec to µsteps/t, divide by f
 *      usteps_per_t = usteps_per_sec / f
 * To convert from µsteps/t to µsteps/sec, multiply by f
 *      usteps_per_sec = usteps_per_t * f
 * 
 * Acceleration
 * ------------
 * 
 * Acceleration is 
 *      ta² = 2^41 / (fCLK)²
 */

// ----------------------------------------------------------------------------
// Taken from TMC5041.c and modified

void tmc5041_readWriteArray(uint8_t chip, uint8_t *data, size_t length)
{
    bcm2835_spi_transfernb(data, data, length);
}

int32_t tmc5041_writeDatagram(tmc5041_motor_t * motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
	uint8_t data[5] = {address | TMC5041_WRITE_BIT, x1, x2, x3, x4 };
	tmc5041_readWriteArray(*motor->chip, data, 5);
	int32_t value = ((uint32_t)x1 << 24) | ((uint32_t)x2 << 16) | (x3 << 8) | x4;
    return value;
}

int32_t tmc5041_writeInt(tmc5041_motor_t * motor, uint8_t address, int32_t value)
{
    // return tmc5041_write_register(address, value);
    return tmc5041_writeDatagram(motor, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

int32_t tmc5041_readInt(tmc5041_motor_t * motor, uint8_t address)
{
	uint8_t data[5] = { 0, 0, 0, 0, 0 };
	data[0] = address;
	tmc5041_readWriteArray(*motor->chip, &data[0], 5);
	data[0] = address;
	tmc5041_readWriteArray(*motor->chip, &data[0], 5);
	return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | (data[3] << 8) | data[4];
}

spi_status_t parse_spi_status(uint8_t spi_status[40])
{
    return (spi_status_t) {
        .status_stop_l2     = spi_status[0] & 0b01000000,
        .status_stop_l1     = spi_status[0] & 0b00100000,
        .velocity_reached2  = spi_status[0] & 0b00010000,
        .velocity_reached1  = spi_status[0] & 0b00001000,
        .driver_error2      = spi_status[0] & 0b00000100,
        .driver_error1      = spi_status[0] & 0b00000010,
        .reset_flag         = spi_status[0] & 0b00000001
    };
}

// 4.1.2 SPI Status Bits Transferred with Each Datagram Read Back
void log_spi_status(tmc5041_motor_t * motor, uint8_t spi_status[40])
{
    bool status_stop_l2     = spi_status[0] & 0b01000000;
    bool status_stop_l1     = spi_status[0] & 0b00100000;
    bool velocity_reached2  = spi_status[0] & 0b00010000;
    bool velocity_reached1  = spi_status[0] & 0b00001000;
    bool driver_error2      = spi_status[0] & 0b00000100;
    bool driver_error1      = spi_status[0] & 0b00000010;
    bool reset_flag         = spi_status[0] & 0b00000001;
    #ifdef DEBUG
    printf("hotshot (%d:%d): spi_status: status_stop_l2=%x status_stop_l1=%x velocity_reached2=%x velocity_reached1=%x driver_error2=%x driver_error1=%x reset_flag=%x\n",
        *motor->chip, *motor->motor, status_stop_l2, status_stop_l1, velocity_reached2, velocity_reached1, driver_error2, driver_error1, reset_flag);
    #endif
    // Reset by reading GSTAT
    // TODO handle resets
    if (driver_error2 || driver_error1 || reset_flag) {
    //     printf("hotshot: WARNING resetting driver");
        printf("hotshot: WARNING needs driver reset");
    //     config_chip();
    }
}

int convert_24bit_to_32bit(int x) {
    // printf("convert_24bit_to_32bit: before=%d\n", x);
    // Check if the 24th bit is set (negative number)
    if (x & 0x00800000) {
        // Sign-extend by setting the upper 8 bits
        x |= 0xFF000000;
    }
    // printf("convert_24bit_to_32bit: after=%d\n", x);
    return x;
}

// Taken from TMC5041.c and modified
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Register access

/** Set RAMPMODE register value*/
void tmc5041_set_register_RAMPMODE(tmc5041_motor_t * motor, int32_t rampmode)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;
    //
    // 0: Positioning mode (using all A, D and V parameters)
    // 1: Velocity mode to positive VMAX (using AMAX acceleration)
    // 2: Velocity mode to negative VMAX (using AMAX acceleration)
    // 3: Hold mode (velocity remains unchanged, unless stop event occurs)
    //
    // RAMPMODE=0
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_RAMPMODE_MASK, TMC5041_RAMPMODE_SHIFT, rampmode);
    uint8_t rampmode_message[40] = {TMC5041_RAMPMODE(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(rampmode_message, spi_status, 5);
}

void tmc5041_set_register_XACTUAL(tmc5041_motor_t * motor, int32 xactual)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;
    // XACTUAL
    write_payload = FIELD_SET(write_payload, TMC5041_XACTUAL_MASK, TMC5041_XACTUAL_SHIFT, xactual);
    uint8_t xtarget_message[40] = {TMC5041_XACTUAL(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(xtarget_message, spi_status, 5);
}

void tmc5041_set_register_IHOLD_IRUN(tmc5041_motor_t * motor, uint32_t ihold, uint32_t irun) {
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    int32_t value = 0x00;
    value = FIELD_SET(value, TMC5041_IHOLD_MASK, TMC5041_IHOLD_SHIFT, ihold);
    value = FIELD_SET(value, TMC5041_IRUN_MASK, TMC5041_IRUN_SHIFT, irun);

    tmc5041_writeInt(motor, TMC5041_IHOLD_IRUN(*motor->motor), value);

    // uint8_t current[40] = {TMC5041_IHOLD_IRUN(*motor->motor) | TMC_WRITE_BIT, value >> 24, value >> 16, value >> 8, value};
    // bcm2835_spi_transfernb(current, current, 5);
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
    bcm2835_spi_transfernb(vcoolthrs_message, vcoolthrs_message, 5);
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
    bcm2835_spi_transfernb(vmax_message, vmax_message, 5);
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
    bcm2835_spi_transfernb(amax_message, amax_message, 5);
}

void tmc5041_set_register_VSTART(tmc5041_motor_t * motor, int32_t vstart) 
{
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTART_MASK, TMC5041_VSTART_SHIFT, vstart);
    uint8_t vstart_message[40] = {TMC5041_VSTART(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(vstart_message, vstart_message, 5);
}

void tmc5041_set_register_VSTOP(tmc5041_motor_t * motor, int32_t vstop) 
{
    int32_t write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTOP_MASK, TMC5041_VSTOP_SHIFT, vstop);
    uint8_t vstop_message[40] = {TMC5041_VSTOP(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(vstop_message, vstop_message, 5);
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
    bcm2835_spi_transfernb(swmode, swmode, 5);

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
    bcm2835_spi_transfernb(coolconf, coolconf, 5);

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
    bcm2835_spi_transfernb(current, current, 5);
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
    bcm2835_spi_transfernb(chop_conf, chop_conf, 5);
}

int32_t tmc5041_get_register_VACTUAL(tmc5041_motor_t * motor)
{
    // Actual motor velocity from ramp generator (signed)
    int32_t value = tmc5041_readInt(motor, TMC5041_VACTUAL(*motor->motor));
    return convert_24bit_to_32bit(value);
}

spi_status_t tmc5041_set_register_XTARGET(tmc5041_motor_t * motor, int32 xtarget)
{
    uint32_t write_payload = 0x00;
    // Target position for RAMPMODE=0 (signed).
    // Write a new target position to this register in order to activate the ramp generator positioning in RAMPMODE=0.
    // Initialize all velocity, acceleration and deceleration parameters before.
    //
    // XTARGET
    write_payload = FIELD_SET(write_payload, TMC5041_XTARGET_MASK, TMC5041_XTARGET_SHIFT, xtarget);
    uint8_t message[40] = {TMC5041_XTARGET(*motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(message, message, 5);
    return parse_spi_status(message);
}

ramp_stat_register_t tmc5041_get_register_RAMP_STAT(tmc5041_motor_t * motor)
{
    // Reading the register will clear the stall condition and the motor may
    // re-start motion, unless the motion controller has been stopped.
    // (Flag and interrupt condition are cleared upon reading)
    // This bit is ORed to the interrupt output signal

    // int32_t reply = tmc5041_read_register(TMC5041_RAMPSTAT(*motor->motor));
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

// DRV_STATUS register access
//
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
    // TODO    
    // bool open_load_phase_b;
    // bool open_load_phase_a;
    // bool ground_short_phase_b;
    // bool ground_short_phase_a;
    return reg;
}

chopconf_register_t tmc5041_get_register_CHOPCONF(tmc5041_motor_t * motor) {
    int32_t reply = tmc5041_readInt(motor, TMC5041_CHOPCONF(*motor->motor));

    chopconf_register_t reg;
    reg.mres = FIELD_GET(reply, TMC5041_MRES_MASK, TMC5041_MRES_SHIFT);

    return reg;
}

int32_t tmc5041_get_register_XLATCH(tmc5041_motor_t * motor) {
    return tmc5041_readInt(motor, TMC5041_XLATCH(*motor->motor));
}

// Register access
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Configuration

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

void tmc5041_chip_init()
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    // GCONF
    //
    uint8_t gconf[40] = {TMC5041_GSTAT, ____, ____, ____, ____};
    bcm2835_spi_transfernb(gconf, spi_status, 5);
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
    // tmc5041_motor_power_off(motor);
    tmc5041_motor_reset(motor);

    // 2. Start  motor  in  velocity  mode,  with  VMAX=10000  and  AMAX=60000
    // (for  quick  acceleration).  The acceleration phase is ended after a few ms.
    tmc5041_set_register_RAMPMODE(motor, 1); // 1: Velocity mode to positive VMAX
    int32_t vmax = 10000;
    int32_t amax = 60000;
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
                *motor->chip, *motor->motor, actual_steps, wait_2_ms/1000, dt);
    #endif

    // 4. Stop the motion ramp by setting VMAX=0.
    tmc5041_set_register_VMAX(motor, 0);
    // TODO restore AMAX
    // tmc5041_set_register_AMAX(motor, 0);
    // TODO restore driver on/off state
    // tmc5041_set_register_RAMPMODE(motor, 0); // 0: hold
    // tmc5041_motor_reset(motor);
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
 * Set velocity and coolstep threshold
 */
void tmc5041_set_velocity(tmc5041_motor_t * motor, int32_t vmax)
{
    vmax = vmax * motor->velocity_time_ref;
    // vmax = vmax * (*motor->vmax_factor_cmd);
    tmc5041_set_register_VMAX(motor, abs(vmax));
}

int32_t tmc5041_get_velocity(tmc5041_motor_t * motor)
{
    int32_t vactual = tmc5041_get_register_VACTUAL(motor);
    vactual = (vactual / motor->velocity_time_ref);
    // vactual = vactual / (*motor->vmax_factor_cmd);
    return vactual;
}

int32_t tmc5041_get_position(tmc5041_motor_t * motor)
{
    return tmc5041_get_register_XACTUAL(motor);
}

/**
 * Intialize necessary motor registers from motor configuration.
 * Should only be called once at beginning of program.
 */
void tmc5041_motor_init(tmc5041_motor_t * motor)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;

    // Calculate frequency scaling
    float64_t scale_factor = tmc5041_frequency_scaling(motor);
    motor->velocity_time_ref = scale_factor;

    // Prevent unexpected moves before we do anything else
    // Always start in hold mode to prevent unexpected movement
    tmc5041_set_register_VMAX(motor, 0);
    tmc5041_set_register_RAMPMODE(motor, 0);
    // TODO turn off chopper with? *motor->chop_toff_cmd = 0;
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
    bcm2835_spi_transfernb(vhigh_message, spi_status, 5);

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
    bcm2835_spi_transfernb(tzerowait_message, spi_status, 5);
}

void tmc5041_motor_power_off(tmc5041_motor_t * motor)
{
    int32_t chopconf = tmc5041_readInt(motor, TMC5041_CHOPCONF(*motor->motor));
    chopconf = FIELD_SET(chopconf, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, 0); // 0 = off
    tmc5041_writeInt(motor, TMC5041_CHOPCONF(*motor->motor), chopconf);

    motor->is_motor_on = FALSE;
}

void tmc5041_motor_power_on(tmc5041_motor_t * motor)
{
    // Power motor up after tmc5041_motor_off()
    // tmc5041_set_register_IHOLD_IRUN(motor, *motor->hold_current_cmd, *motor->run_current_cmd);

    // If we power a motor off, then back on, the XACTUAL is some random number.
    // This sort of makes sense because once a motor has been powered off we can no
    // longer guarantee it's position. So sync XACTUAL with what EMC has commanded.
    // tmc5041_set_register_XACTUAL(motor, motor->position_cmd);

    int32_t chopconf = tmc5041_readInt(motor, TMC5041_CHOPCONF(*motor->motor));
    chopconf = FIELD_SET(chopconf, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, *motor->chop_toff_cmd);
    tmc5041_writeInt(motor, TMC5041_CHOPCONF(*motor->motor), chopconf);

    motor->is_motor_on = TRUE;
}

void tmc5041_motor_stop(tmc5041_motor_t * motor)
{
    // OPTIONS TO TERMINATE MOTION USING ACCELERATION SETTINGS:
    // a)
    // Switch to velocity mode
    // set VMAX=0 a
    tmc5041_set_register_VMAX(motor, 0);   
}

bool tmc5041_motor_set_home(tmc5041_motor_t * motor)
{
    // Disable stallguard stop on stall
    *motor->sg_stop_cmd = 0;
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

    return TRUE;
}

/**
 * Reset driver back to power-on state.
 */
void tmc5041_motor_reset(tmc5041_motor_t * motor)
{
    // Stop chopper
    tmc5041_motor_power_off(motor);
    // Clear stallguard
    tmc5041_motor_clear_stall(motor);
    // Reset XACTUAL to 0
    tmc5041_motor_set_home(motor);
}

// Put motor in "hold position" mode
void tmc5041_motor_position_hold(tmc5041_motor_t * motor)
{
    tmc5041_set_register_VMAX(motor, 0);
}

/**
 * Clear Stallguard motor stall flag.
 * Returns true if motor was stalled.
 */
bool tmc5041_motor_clear_stall(tmc5041_motor_t * motor)
{
    // Release by setting SW_MODE sg_stop = 0
    // "Disable to release motor after stop event."
    // bool orig_stop_cmd = *motor->sg_stop_cmd;
    // *motor->sg_stop_cmd = 0;
    // tmc5041_push_register_SW_MODE(motor);
    // *motor->sg_stop_cmd = orig_stop_cmd;
    // tmc5041_push_register_SW_MODE(motor);

    // Clear stallguard with by reading RAMP_STAT register
    // ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(motor);
    tmc5041_pull_register_RAMP_STAT(motor);

    // printf("hotshot(%d, %d): RAMP_STAT status_sg=%d, position_reached=%d, velocity_reached=%d, event_pos_reached=%d, event_stop_sg=%d, event_stop_r=%d, event_stop_l=%d, status_latch_r=%d, status_latch_l=%d, status_stop_r=%d, status_stop_l=%d\n", 
    //     motor->chip.chip, motor->motor,
    //     motor->status_sg,
    //     motor->position_reached,
    //     motor->velocity_reached_fb,
    //     motor->event_pos_reached,
    //     motor->event_stop_sg,
    //     motor->event_stop_r,
    //     motor->event_stop_l,
    //     motor->status_latch_r,
    //     motor->status_latch_l,
    //     motor->status_stop_r,
    //     motor->status_stop_l
    // );

    // tmc5041_pull_register_DRV_STATUS(motor);

    // printf("tmc5041_motor_clear_stall: status_sg=%d, motor_load_fb=%d, motor_stall_fb=%d\n",
    //     ramp_stat.status_sg,
    //     *motor->motor_load_fb,
    //     *motor->motor_stall_fb);

    return *motor->motor_stall_fb;
    // return FALSE;
}

void tmc5041_motor_end(tmc5041_motor_t * motor)
{
    rpi_spi_select(*motor->chip);
    tmc5041_motor_reset(motor);
    rpi_spi_unselect();
}

void tmc5041_end(tmc5041_motor_t * motors, size_t motor_count)
{
    for (size_t i = 0; i < motor_count; i++)
    {
        rpi_spi_select(*motors[i].chip);
        tmc5041_motor_reset(&motors[i]);
        rpi_spi_unselect();
    }
}

// Configuration
// ----------------------------------------------------------------------------
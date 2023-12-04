#include "stdio.h"
#include "bcm2835.h"
#include "rpi.h"
#include "tmc5041.h"

// ----------------------------------------------------------------------------
// Taken from TMC5041.c and modified

void tmc5041_readWriteArray(uint8_t chip, uint8_t *data, size_t length) {
    // printf("tmc5041_readWriteArray: select chip=%d\n", chip);
    // bcm2835_spi_chipSelect(chip);
    bcm2835_spi_transfernb(data, data, length);
    // bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

int32_t tmc5041_writeDatagram(tmc5041_motor_t * motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
	uint8_t data[5] = {address | TMC5041_WRITE_BIT, x1, x2, x3, x4 };
	tmc5041_readWriteArray(motor->chip.chip, data, 5);

	int32_t value = ((uint32_t)x1 << 24) | ((uint32_t)x2 << 16) | (x3 << 8) | x4;

	// Write to the shadow register and mark the register dirty
	// address = TMC_ADDRESS(address);
	// tmc5041->config->shadowRegister[address] = value;
	// tmc5041->registerAccess[address] |= TMC_ACCESS_DIRTY;

    return value;
}

int32_t tmc5041_writeInt(tmc5041_motor_t * motor, uint8_t address, int32_t value)
{
    // return tmc5041_write_register(address, value);
    return tmc5041_writeDatagram(motor, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

int32_t tmc5041_readInt(tmc5041_motor_t * motor, uint8_t address)
{
    // return tmc5041_read_register(address);

    // Remove shadow register bit
	// address = TMC_ADDRESS(address);

	// // register not readable -> shadow register copy
	// // if(!TMC_IS_READABLE(tmc5041->registerAccess[address]))
	// // 	return tmc5041->config->shadowRegister[address];

	uint8_t data[5] = { 0, 0, 0, 0, 0 };

	data[0] = address;
	tmc5041_readWriteArray(motor->chip.chip, data, 5);

	data[0] = address;
	tmc5041_readWriteArray(motor->chip.chip, data, 5);

	return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | (data[3] << 8) | data[4];
}

// Taken from TMC5041.c and modified
// ----------------------------------------------------------------------------

spi_status_t parse_spi_status(uint8_t spi_status[40]) {
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
    rtapi_print("hotshot (%d:%d): spi_status: status_stop_l2=%x status_stop_l1=%x velocity_reached2=%x velocity_reached1=%x driver_error2=%x driver_error1=%x reset_flag=%x\n",
        motor->chip, motor->motor, status_stop_l2, status_stop_l1, velocity_reached2, velocity_reached1, driver_error2, driver_error1, reset_flag);
    #endif
    // Reset by reading GSTAT
    // TODO handle resets
    if (driver_error2 || driver_error1 || reset_flag) {
    //     rtapi_print("hotshot: WARNING resetting driver");
        printf("hotshot: WARNING needs driver reset");
    //     config_chip();
    }
}

/** Set RAMPMODE register value*/
void tmc5041_set_register_RAMPMODE(tmc5041_motor_t * motor, uint8_t rampmode)
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
    uint8_t rampmode_message[40] = {TMC5041_RAMPMODE(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(rampmode_message, spi_status, 5);
}

void tmc5041_set_register_XACTUAL(tmc5041_motor_t * motor, int32 xactual)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;
    // XACTUAL
    write_payload = FIELD_SET(write_payload, TMC5041_XACTUAL_MASK, TMC5041_XACTUAL_SHIFT, xactual);
    uint8_t xtarget_message[40] = {TMC5041_XACTUAL(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(xtarget_message, spi_status, 5);
}

int32_t tmc5041_get_register_XACTUAL(tmc5041_motor_t * motor)
{
    // XACTUAL: Actual motor position (signed)
    // return tmc5041_read_register(TMC5041_XACTUAL(motor->motor));
    return tmc5041_readInt(motor, TMC5041_XACTUAL(motor->motor));
}

int32_t tmc5041_get_register_VACTUAL(tmc5041_motor_t * motor)
{
    // Actual motor velocity from ramp generator (signed)
    // return tmc5041_read_register(TMC5041_VACTUAL(motor->motor));
    return tmc5041_readInt(motor, TMC5041_VACTUAL(motor->motor));
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
    uint8_t message[40] = {TMC5041_XTARGET(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(message, message, 5);
    return parse_spi_status(message);
}

// Returns true if StallGuard event is triggered
ramp_stat_register_t tmc5041_get_register_RAMP_STAT(tmc5041_motor_t * motor) {
    // Reading the register will clear the stall condition and the motor may
    // re-start motion, unless the motion controller has been stopped.
    // (Flag and interrupt condition are cleared upon reading)
    // This bit is ORed to the interrupt output signal

    // int32_t reply = tmc5041_read_register(TMC5041_RAMPSTAT(motor->motor));
    int32_t reply = tmc5041_readInt(motor, TMC5041_RAMPSTAT(motor->motor));

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

// DRV_STATUS register access
//
drv_status_register_t tmc5041_get_register_DRV_STATUS(tmc5041_motor_t * motor)
{
    int32_t reply = tmc5041_readInt(motor, TMC5041_DRVSTATUS(motor->motor));

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
    int32_t reply = tmc5041_readInt(motor, TMC5041_CHOPCONF(motor->motor));

    chopconf_register_t reg;
    reg.mres = FIELD_GET(reply, TMC5041_MRES_MASK, TMC5041_MRES_SHIFT);

    return reg;
}

int32_t tmc5041_get_register_XLATCH(tmc5041_motor_t * motor) {
    return tmc5041_readInt(motor, TMC5041_XLATCH(motor->motor));
}

bool tmc5041_motor_set_home(tmc5041_motor_t * motor)
{
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
    tmc5041_set_register_RAMPMODE(motor, TMC5041_MODE_POSITION);

    return TRUE;
}

void tmc5041_motor_reset(tmc5041_motor_t * motor)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;

    // Set TOFF=0 to clear registers
    write_payload = FIELD_SET(write_payload, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, 0b0);
    uint8_t chop_conf[40] = {TMC5041_CHOPCONF(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(chop_conf, spi_status, 5);

    // Reset XACTUAL to 0
    tmc5041_motor_set_home(motor);
}

uint8_t microsteps_to_tmc_mres(uint16_t usteps)
{
    // 0 = 256 usteps = 0b0000 (default)
    // 1 = 128 usteps = 0b0001
    // 2 =  64 usteps = 0b0010
    // 3 =  32 usteps = 0b0011
    // 4 =  16 usteps = 0b0100
    // 5 =  8 usteps =
    // 6 =  4 usteps =
    // 7 =  2 usteps =
    if (usteps == 256)
    {
        return 0;
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

// ----------------------------------------------------------------------------
// Configuration

void tmc5041_chip_init()
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    // GCONF
    //
    uint8_t gconf[40] = {TMC5041_GSTAT, ____, ____, ____, ____};
    bcm2835_spi_transfernb(gconf, spi_status, 5);
}

void tmc5041_motor_set_config_registers(tmc5041_motor_t * motor)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;

    // Ramp Generator Motion Control Register Set
    //
    uint8_t irun = *motor->run_current_cmd;
    uint8_t ihold = *motor->hold_current_cmd;
    uint8_t iholddelay = 1;
    // This is the lower threshold velocity for switching on smart
    // energy CoolStep and StallGuard feature. Further it is the upper
    // operation velocity for StealthChop.
    // Hint: May be adapted to disable CoolStep during acceleration and deceleration phase by setting identical to VMAX.
    // Enable CoolStep and StallGuard at 5mm per second
    // uint32_t vcoolthrs = mm_to_microsteps(motor->microstep_per_mm, 10);
    // TMC5041 data sheet uses 30 RPM (20 mm/sec on X axis)
    // uint32_t vcoolthrs = 1600*2; // ( 30 * 200 * 16 ) / 60
    uint32_t vcoolthrs = *motor->cs_thresh_cmd;
    // This velocity setting allows velocity dependent switching into
    // a different chopper mode and fullstepping to maximize torque.
    uint32_t vhigh = 64000;

    // Ramp Generator Motion Control Register Set
    //
    uint8_t rampmode = 0; // positioning mode
    uint8_t vstart = 0;
    uint8_t vstop = 10;
    // V1=0 disables A1 and D1 phase, use AMAX, DMAX only
    uint32_t v1 = 0;
    // uint16_t amax = mm_to_microsteps(motor->microstep_per_mm, motor->max_acceleration_cmd);
    uint16_t amax = *motor->max_acceleration_cmd;
    uint16_t a1 = amax * 2;
    uint16_t dmax = amax;
    // Do not set DI=0 in positioning mode, even if V1=0!
    uint16_t d1 = a1;
    uint8_t tzerowait = 0;

    // Coolstep
    //
    uint8_t sfilt = 0;
    uint8_t seimin = 0;
    uint8_t sedn = 0;
    uint8_t seup = 0;
    // When the load increases, SG falls below SEMIN, and CoolStep increases the current.
    // When the load decreases, SG rises above (SEMIN + SEMAX + 1) * 32, and the current is reduced.
    uint8_t semin = 3; // coolstep activated when SG < SEMIN*32
    uint8_t semax = 10; // coolstep deactivated when SG >= (SEMIN+SEMAX+1)*32

    // Chopper Configuration
    //
    uint16_t vhighchm = *motor->max_velocity_cmd;
    uint16_t vhighfs = *motor->max_velocity_cmd;
    uint8_t tbl = 0b10;
    uint8_t hend = 0b0;
    uint8_t hstrt = 0b100;
    uint8_t toff = 0b0100;
    uint8_t mres = motor->mres;
    // 0: Low sensitivity, high sense resistor voltage
    // 1: High sensitivity, low sense resistor voltage
    uint8_t vsense = 0;
    // 0 Standard mode (SpreadCycle)
    uint8_t chm = 0;

    // Stallguard parameters
    //
    sg_thresh_t sg_thresh = *motor->sg_thresh_cmd;

    // Calculate max velocity
    // Target velocity should be in absolute units
    // float abs_vtarget_mm_per_sec = fabs(motor->max_velocity_cmd);
    // VMAX: Motion ramp target velocity
    // int32_t vmax = abs_vtarget_mm_per_sec * motor->microstep_per_mm; // in microsteps per second
    int32_t vmax = *motor->max_velocity_cmd;

    // Switch mode
    bool en_softstop = 0;
    bool sg_stop = *motor->sg_stop_cmd;

    //
    // Power Configuration
    //

    // Current Setting
    //
    write_payload = 0x00;
    // IRUN: Current scale when motor is running (scaling factor N/32 i.e. 1/32, 2/32, … 31/32)
    // For high precision motor operation, work with a current scaling factor in the range 16 to 31,
    // because scaling down the current values reduces the effective microstep resolution by making microsteps coarser.
    write_payload = FIELD_SET(write_payload, TMC5041_IRUN_MASK, TMC5041_IRUN_SHIFT, irun);
    // IHOLD: Identical to IRUN, but for motor in stand still.
    write_payload = FIELD_SET(write_payload, TMC5041_IHOLD_MASK, TMC5041_IHOLD_SHIFT, ihold);
    // IHOLDDELAY: 0 = instant IHOLD
    write_payload = FIELD_SET(write_payload, TMC5041_IHOLDDELAY_MASK, TMC5041_IHOLDDELAY_SHIFT, iholddelay);
    uint8_t current[40] = {TMC5041_IHOLD_IRUN(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(current, spi_status, 5);

    // Chopper configuration
    //
    // PWMCONF: StealthChop Configuration
    //
    // write_payload = 0x00;
    // // PWM_AUTOSCALE: Enable automatic current scaling using current measurement or use fixed scaling mode.
    // write_payload = FIELD_SET(write_payload, TMC5041_PWM_AUTOSCALE_MASK, TMC5041_PWM_AUTOSCALE_SHIFT, pwm_autoscale);
    // // PWM_GRAD: Global enable and regulation loop gradient when pwm_autoscale=1
    // write_payload = FIELD_SET(write_payload, TMC5041_PWM_GRAD_MASK, TMC5041_PWM_GRAD_SHIFT, pwm_grad);
    // // PWM_AMPL: User defined PWM amplitude for fixed scaling or amplitude limit for re-entry into StealthChop mode when pwm_autoscale=1.
    // write_payload = FIELD_SET(write_payload, TMC5041_PWM_AMPL_MASK, TMC5041_PWM_AMPL_SHIFT, pwm_ampl);
    // // PWM_FREQ: PWM frequency selection. See 7.1 Two Modes for Current Regulation
    // write_payload = FIELD_SET(write_payload, TMC5041_PWM_FREQ_MASK, TMC5041_PWM_FREQ_SHIFT, pwm_freq);
    // uint8_t pwmconf[40] = {TMC5041_PWMCONF(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    // bcm2835_spi_transfernb(pwmconf, spi_status, 5);
    //
    // CHOPCONF: Chopper Configuration (i.e. SpreadCycle)
    //
    write_payload = 0x00;
    // MRES: micro step resolution
    write_payload = FIELD_SET(write_payload, TMC5041_MRES_MASK, TMC5041_MRES_SHIFT, mres);
    // vhighchm: high velocity chopper mode
    write_payload = FIELD_SET(write_payload, TMC5041_VHIGHCHM_MASK, TMC5041_VHIGHCHM_SHIFT, vhighchm);
    // vhighfs: high velocity fullstep selection
    write_payload = FIELD_SET(write_payload, TMC5041_VHIGHFS_MASK, TMC5041_VHIGHFS_SHIFT, vhighfs);
    // VSENSE: sense resistor voltage based current scaling
    write_payload = FIELD_SET(write_payload, TMC5041_VSENSE_MASK, TMC5041_VSENSE_SHIFT, vsense);
    // TBL: blank time select
    write_payload = FIELD_SET(write_payload, TMC5041_TBL_MASK, TMC5041_TBL_SHIFT, tbl);
    // CHM: chopper mode 
    write_payload = FIELD_SET(write_payload, TMC5041_CHM_MASK, TMC5041_CHM_SHIFT, chm);
    // TODO rndtf
    // TODO disfdcc
    // HEND: hysteresis low value OFFSET sine wave offset
    write_payload = FIELD_SET(write_payload, TMC5041_HEND_MASK, TMC5041_HEND_SHIFT, hend);
    // HSTRT: hysteresis start value added to HEND
    write_payload = FIELD_SET(write_payload, TMC5041_HSTRT_MASK, TMC5041_HSTRT_SHIFT, hstrt);
    // TOFF: off time and driver enable
    write_payload = FIELD_SET(write_payload, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, toff);
    uint8_t chop_conf[40] = {TMC5041_CHOPCONF(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(chop_conf, spi_status, 5);

    //
    // CoolStep and StallGuard2 configuration
    //

    // VCOOLTHRS
    //
    // Lower ramp generator velocity threshold. Below this velocity CoolStep and StallGuard becomes disabled (not used
    // in Step/Dir mode). Adapt to the lower limit of the velocity range where StallGuard2 gives a stable result
    //
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VCOOLTHRS_MASK, TMC5041_VCOOLTHRS_SHIFT, vcoolthrs);
    uint8_t vcoolthrs_message[40] = {TMC5041_VCOOLTHRS(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(vcoolthrs_message, spi_status, 5);

    // VHIGH
    // 
    // Upper ramp generator velocity threshold value. Above this velocity CoolStep becomes disabled
    // (not used in Step/Dir mode). Adapt to the velocity range where StallGuard2 gives a stable result.
    //
    write_payload = 0x00;
    // VHIGH: Set high values for both
    write_payload = FIELD_SET(write_payload, TMC5041_VHIGH_MASK, TMC5041_VHIGH_SHIFT, vhigh);
    uint8_t vhigh_message[40] = {TMC5041_VHIGH(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(vhigh_message, spi_status, 5);

    // COOLCONF: Smart Energy Control CoolStep and StallGuard2
    //
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_SFILT_MASK, TMC5041_SFILT_SHIFT, sfilt);
    write_payload = FIELD_SET(write_payload, TMC5041_SGT_MASK, TMC5041_SGT_SHIFT, sg_thresh);
    write_payload = FIELD_SET(write_payload, TMC5041_SEIMIN_MASK, TMC5041_SEIMIN_SHIFT, seimin);
    write_payload = FIELD_SET(write_payload, TMC5041_SEDN_MASK, TMC5041_SEDN_SHIFT, sedn);
    write_payload = FIELD_SET(write_payload, TMC5041_SEMAX_MASK, TMC5041_SEMAX_SHIFT, semax);
    write_payload = FIELD_SET(write_payload, TMC5041_SEUP_MASK, TMC5041_SEUP_SHIFT, seup);
    write_payload = FIELD_SET(write_payload, TMC5041_SEMIN_MASK, TMC5041_SEMIN_SHIFT, semin);
    uint8_t coolconf[40] = {TMC5041_COOLCONF(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(coolconf, spi_status, 5);

    // SW_MODE: Reference Switch & StallGuard2 Event Configuration Register
    //
    write_payload = 0x00;
    // Attention: Do not use soft stop in combination with StallGuard2.
    write_payload = FIELD_SET(write_payload, TMC5041_EN_SOFTSTOP_MASK, TMC5041_EN_SOFTSTOP_SHIFT, en_softstop);
    // Note: set VCOOLTHRS to a suitable value before enabling this
    write_payload = FIELD_SET(write_payload, TMC5041_SG_STOP_MASK, TMC5041_SG_STOP_SHIFT, sg_stop);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_R_INACTIVE_MASK, TMC5041_LATCH_R_INACTIVE_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_R_ACTIVE_MASK, TMC5041_LATCH_R_ACTIVE_SHIFT, 1);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_L_INACTIVE_MASK, TMC5041_LATCH_L_INACTIVE_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_LATCH_L_ACTIVE_MASK, TMC5041_LATCH_L_ACTIVE_SHIFT, 1);
    write_payload = FIELD_SET(write_payload, TMC5041_SWAP_LR_MASK, TMC5041_SWAP_LR_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_POL_STOP_R_MASK, TMC5041_POL_STOP_R_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_POL_STOP_L_MASK, TMC5041_POL_STOP_L_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_STOP_R_ENABLE_MASK, TMC5041_STOP_R_ENABLE_SHIFT, 0);
    write_payload = FIELD_SET(write_payload, TMC5041_STOP_L_ENABLE_MASK, TMC5041_STOP_L_ENABLE_SHIFT, 0);
    uint8_t swmode[40] = {TMC5041_SWMODE(motor->motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(swmode, spi_status, 5);

    //
    // Ramp Configuration
    //

    // VSTART
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTART_MASK, TMC5041_VSTART_SHIFT, vstart);
    uint8_t vstart_message[40] = {TMC5041_VSTART(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(vstart_message, spi_status, 5);

    // VSTOP
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTOP_MASK, TMC5041_VSTOP_SHIFT, vstop);
    uint8_t vstop_message[40] = {TMC5041_VSTOP(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(vstop_message, spi_status, 5);

    // V1
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_V1__MASK, TMC5041_V1__SHIFT, v1);
    uint8_t v1_message[40] = {TMC5041_V1(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(v1_message, spi_status, 5);

    // A1
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_A1_MASK, TMC5041_A1_SHIFT, a1);
    uint8_t a1_message[40] = {TMC5041_A1(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(a1_message, spi_status, 5);

    // Maximum acceleration/deceleration [µsteps / ta²]
    //
    // This is the acceleration and deceleration value for velocity mode.
    // In position mode (RAMP=0), must be lower than A1 (???)
    //
    // AMAX
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_AMAX_MASK, TMC5041_AMAX_SHIFT, amax);
    uint8_t amax_message[40] = {TMC5041_AMAX(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(amax_message, spi_status, 5);

    // DMAX = AMAX
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_DMAX_MASK, TMC5041_DMAX_SHIFT, dmax);
    uint8_t dmax_message[40] = {TMC5041_DMAX(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(dmax_message, spi_status, 5);

    // D1 = A1
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_D1_MASK, TMC5041_D1_SHIFT, d1);
    uint8_t d1_message[40] = {TMC5041_D1(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(d1_message, spi_status, 5);

    // TZEROWAIT
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_TZEROWAIT_MASK, TMC5041_TZEROWAIT_SHIFT, tzerowait);
    uint8_t tzerowait_message[40] = {TMC5041_TZEROWAIT(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(tzerowait_message, spi_status, 5);

    //
    // Target velocity
    //
    // This is the target velocity [in µsteps / t] in velocity mode. It can be changed any time during a motion
    //
    // VMAX
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VMAX_MASK, TMC5041_VMAX_SHIFT, vmax);
    uint8_t vmax_message[40] = {TMC5041_VMAX(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(vmax_message, spi_status, 5);

    // Ramp mode
    tmc5041_set_register_RAMPMODE(motor, rampmode);
}

void tmc5041_motor_init(tmc5041_motor_t * motor)
{
        
        printf("Configuring motor: chip=%d, motor=%d\n", motor->chip.chip, motor->motor);
        
        #ifdef DEBUG
        rtapi_print("start spi convo\n");
        rtapi_print("Motor is %d %d %d %d\n", 
            motor->chip, motor->motor, motor->pitch, motor->teeth);
        #endif

        #ifdef DEBUG
        rtapi_print("start spi convo with chip %d\n", motor->chip);
        #endif

        printf("Start spi conversation\n");
        rpi_spi_select(motor->chip.chip);

        #ifdef DEBUG
        rtapi_print("Reset motor\n");
        #endif

        printf("Reset motor conversation\n");
        tmc5041_motor_reset(motor);

        #ifdef DEBUG
        rtapi_print("Config motor\n");
        #endif

        printf("Config motor registers\n");
        tmc5041_motor_set_config_registers(motor);

        #ifdef DEBUG
        rtapi_print("end spi convo\n");
        #endif

        printf("End spi conversation\n");
        rpi_spi_unselect();
}

void tmc5041_motor_end(tmc5041_motor_t * motor)
{
    rpi_spi_select(motor->chip.chip);
    tmc5041_motor_reset(motor);
    rpi_spi_unselect();
}

void tmc5041_init(tmc5041_motor_t * motors, size_t motor_count)
{
    #ifdef DEBUG
    rtapi_print("enter config_tmc5041\n");
    #endif

    // Configure chip and motors
    for (size_t i = 0; i < motor_count; i++)
    {

        printf("Configuring motor: chip=%d, motor=%d\n", motors[i].chip.chip, motors[i].motor);
        
        #ifdef DEBUG
        rtapi_print("start spi convo\n");
        rtapi_print("Motor is %d %d %d %d\n", 
            motors[i].chip, motors[i].motor, motors[i].pitch, motors[i].teeth);
        #endif

        #ifdef DEBUG
        rtapi_print("start spi convo with chip %d\n", motors[i].chip);
        #endif

        printf("Start spi conversation\n");
        rpi_spi_select(motors[i].chip.chip);

        #ifdef DEBUG
        rtapi_print("Reset motor\n");
        #endif

        printf("Reset motor conversation\n");
        tmc5041_motor_reset(&motors[i]);

        #ifdef DEBUG
        rtapi_print("Config motor\n");
        #endif

        printf("Config motor\n");
        tmc5041_motor_init(&motors[i]);

        #ifdef DEBUG
        rtapi_print("end spi convo\n");
        #endif

        printf("End spi conversation\n");
        rpi_spi_unselect();
    }

    #ifdef DEBUG
    rtapi_print("exit config_tmc5041\n");
    #endif
}

void tmc5041_end(tmc5041_motor_t * motors, size_t motor_count)
{
    for (size_t i = 0; i < motor_count; i++)
    {
        rpi_spi_select(motors[i].chip.chip);
        tmc5041_motor_reset(&motors[i]);
        rpi_spi_unselect();
    }
}

// Configuration
// ----------------------------------------------------------------------------
#include "bcm2835.h"
#include "tmc5041.h"

#include "bcm2835.c"

// ----------------------------------------------------------------------------
// Taken from TMC5041.c and modified

void tmc5041_readWriteArray(uint8_t chip, uint8_t *data, size_t length) {
    // printf("tmc5041_readWriteArray: select chip=%d\n", chip);
    // bcm2835_spi_chipSelect(chip);
    bcm2835_spi_transfernb(data, data, length);
    // bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

/**
 * Inspired by tmc5041_readInt() in TMC-API.
 * Do not change any types in this function.
*/
// int32_t tmc5041_read_register(uint8_t address) {
// 	uint8_t data[5] = { 0, 0, 0, 0, 0 };
//     data[0] = address;
//     bcm2835_spi_transfernb(data, data, 5);
//     data[0] = address;
//     bcm2835_spi_transfernb(data, data, 5);
//     return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | data[3] << 8 | data[4];
// }

// int32_t tmc5041_write_register(uint8_t address, int32_t value)
// {
//     uint8_t data[5] = {address | TMC5041_WRITE_BIT, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0) };
//     bcm2835_spi_transfernb(data, data, 5);
//     return ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) | data[3] << 8 | data[4];
// }

int32_t tmc5041_writeDatagram(joint_t * motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
	uint8_t data[5] = {address | TMC5041_WRITE_BIT, x1, x2, x3, x4 };
	tmc5041_readWriteArray(motor->chip, data, 5);

	int32_t value = ((uint32_t)x1 << 24) | ((uint32_t)x2 << 16) | (x3 << 8) | x4;

	// Write to the shadow register and mark the register dirty
	// address = TMC_ADDRESS(address);
	// tmc5041->config->shadowRegister[address] = value;
	// tmc5041->registerAccess[address] |= TMC_ACCESS_DIRTY;

    return value;
}

int32_t tmc5041_writeInt(joint_t * motor, uint8_t address, int32_t value)
{
    // return tmc5041_write_register(address, value);
    return tmc5041_writeDatagram(motor, address, BYTE(value, 3), BYTE(value, 2), BYTE(value, 1), BYTE(value, 0));
}

int32_t tmc5041_readInt(joint_t * motor, uint8_t address)
{
    // return tmc5041_read_register(address);

    // Remove shadow register bit
	// address = TMC_ADDRESS(address);

	// // register not readable -> shadow register copy
	// // if(!TMC_IS_READABLE(tmc5041->registerAccess[address]))
	// // 	return tmc5041->config->shadowRegister[address];

	uint8_t data[5] = { 0, 0, 0, 0, 0 };

	data[0] = address;
	tmc5041_readWriteArray(motor->chip, data, 5);

	data[0] = address;
	tmc5041_readWriteArray(motor->chip, data, 5);

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
void log_spi_status(joint_t * motor, uint8_t spi_status[40])
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
void tmc5041_set_register_RAMPMODE(joint_t * motor, uint8_t rampmode)
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

    printf("Set RAMPMODE to %d\n", rampmode);
}

void tmc5041_set_register_XACTUAL(joint_t * motor, int32 xactual)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;
    // XACTUAL
    write_payload = FIELD_SET(write_payload, TMC5041_XACTUAL_MASK, TMC5041_XACTUAL_SHIFT, xactual);
    uint8_t xtarget_message[40] = {TMC5041_XACTUAL(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(xtarget_message, spi_status, 5);
}

int32_t tmc5041_get_register_XACTUAL(joint_t * motor)
{
    // XACTUAL: Actual motor position (signed)
    // return tmc5041_read_register(TMC5041_XACTUAL(motor->motor));
    return tmc5041_readInt(motor, TMC5041_XACTUAL(motor->motor));
}

int32_t tmc5041_get_register_VACTUAL(joint_t * motor)
{
    // Actual motor velocity from ramp generator (signed)
    // return tmc5041_read_register(TMC5041_VACTUAL(motor->motor));
    return tmc5041_readInt(motor, TMC5041_VACTUAL(motor->motor));
}

spi_status_t tmc5041_set_register_XTARGET(joint_t * motor, int32 xtarget)
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
ramp_stat_register_t tmc5041_get_register_RAMP_STAT(joint_t * motor) {
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
drv_status_register_t tmc5041_get_register_DRV_STATUS(joint_t * motor)
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

chopconf_register_t tmc5041_get_register_CHOPCONF(joint_t * motor) {
    int32_t reply = tmc5041_readInt(motor, TMC5041_CHOPCONF(motor->motor));

    chopconf_register_t reg;
    reg.mres = FIELD_GET(reply, TMC5041_MRES_MASK, TMC5041_MRES_SHIFT);

    return reg;
}

int32_t tmc5041_get_register_XLATCH(joint_t * motor) {
    return tmc5041_readInt(motor, TMC5041_XLATCH(motor->motor));
}

bool set_home(joint_t * motor)
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

void reset_motor(joint_t * motor)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;

    // Set TOFF=0 to clear registers
    write_payload = FIELD_SET(write_payload, TMC5041_TOFF_MASK, TMC5041_TOFF_SHIFT, 0b0);
    uint8_t chop_conf[40] = {TMC5041_CHOPCONF(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
    bcm2835_spi_transfernb(chop_conf, spi_status, 5);

    // Reset XACTUAL to 0
    set_home(motor);
}

// Taken from TMC5041_eval.c
// uint32_t moveTo(joint_t * motor, int32_t position)
// {
//     // TODO
// 	// if(TMC5041.vMaxModified[motor])
// 	// {
// 	// 	tmc5041_writeInt(motorToIC(motor), TMC5041_VMAX(motor), TMC5041_config->shadowRegister[TMC5041_VMAX(motor)]);
// 	// 	TMC5041.vMaxModified[motor] = false;
// 	// }

//     tmc5041_set_register_XTARGET(motor, position);

//     // int32_t write_payload = 0x00;
//     // write_payload = FIELD_SET(write_payload, TMC5041_XTARGET_MASK, TMC5041_XTARGET_SHIFT, 10000);
//     // uint8_t xtarget[40] = {TMC5041_XTARGET(motor->motor) | TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload};
//     // bcm2835_spi_transfernb(xtarget, xtarget, 5);

// 	// tmc5041_writeInt(motor, TMC5041_XTARGET(motor->motor), position);
// 	// tmc5041_writeInt(motorToIC(motor), TMC5041_RAMPMODE(motor), TMC5041_MODE_POSITION);

// 	return TMC_ERROR_NONE;
// }

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

// Requires
//  #include "rpi.c"
//  #include "bcm2835.c"

#include "bcm2835.h"
#include "rpi.h"
// #include "hotshot_init.h"

bool needs_setup = TRUE;

void config_chip()
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    // GCONF
    //
    uint8_t gconf[40] = {TMC5041_GSTAT, ____, ____, ____, ____};
    bcm2835_spi_transfernb(gconf, spi_status, 5);
}

void config_motor(joint_t * motor)
{
    static uint8_t spi_status[40] = {____, ____, ____, ____, ____};
    uint32_t write_payload = 0x00;

    // Ramp Generator Motion Control Register Set
    //
    uint8_t irun = *motor->tmc_run_current_cmd;
    uint8_t ihold = *motor->tmc_hold_current_cmd;
    uint8_t iholddelay = 1;
    // This is the lower threshold velocity for switching on smart
    // energy CoolStep and StallGuard feature. Further it is the upper
    // operation velocity for StealthChop.
    // Hint: May be adapted to disable CoolStep during acceleration and deceleration phase by setting identical to VMAX.
    // Enable CoolStep and StallGuard at 5mm per second
    // uint32_t vcoolthrs = mm_to_microsteps(motor->microstep_per_mm, 10);
    // TMC5041 data sheet uses 30 RPM (20 mm/sec on X axis)
    // uint32_t vcoolthrs = 1600*2; // ( 30 * 200 * 16 ) / 60
    uint32_t vcoolthrs = *motor->tmc_cs_thresh_cmd;
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
    uint16_t amax = *motor->tmc_max_acceleration_cmd;
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
    uint16_t vhighchm = *motor->tmc_max_velocity_cmd;
    uint16_t vhighfs = *motor->tmc_max_velocity_cmd;
    uint8_t tbl = 0b10;
    uint8_t hend = 0b0;
    uint8_t hstrt = 0b100;
    uint8_t toff = 0b0100;
    uint8_t mres = motor->tmc_mres;
    // 0: Low sensitivity, high sense resistor voltage
    // 1: High sensitivity, low sense resistor voltage
    uint8_t vsense = 0;
    // 0 Standard mode (SpreadCycle)
    uint8_t chm = 0;

    // Stallguard parameters
    //
    pin_sg_thresh_t sg_thresh = *motor->tmc_sg_thresh_cmd;

    // Calculate max velocity
    // Target velocity should be in absolute units
    // float abs_vtarget_mm_per_sec = fabs(motor->max_velocity_cmd);
    // VMAX: Motion ramp target velocity
    // int32_t vmax = abs_vtarget_mm_per_sec * motor->microstep_per_mm; // in microsteps per second
    int32_t vmax = *motor->tmc_max_velocity_cmd;

    // Switch mode
    bool en_softstop = 0;
    bool sg_stop = *motor->tmc_sg_stop_cmd;

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

void config_tmc5041(joint_t * motors, size_t motor_count)
{
    #ifdef DEBUG
    rtapi_print("enter config_tmc5041\n");
    #endif

    // Configure chip and motors
    for (size_t i = 0; i < motor_count; i++)
    {

        printf("Configuring motor: chip=%d, motor=%d\n", motors[i].chip, motors[i].motor);
        
        #ifdef DEBUG
        rtapi_print("start spi convo\n");
        rtapi_print("Motor is %d %d %d %d\n", 
            motors[i].chip, motors[i].motor, motors[i].pitch, motors[i].teeth);
        #endif

        #ifdef DEBUG
        rtapi_print("start spi convo with chip %d\n", motors[i].chip);
        #endif

        printf("Start spi conversation\n");
        start_spi_conversation(motors[i].chip);

        #ifdef DEBUG
        rtapi_print("Reset motor\n");
        #endif

        printf("Reset motor conversation\n");
        reset_motor(&motors[i]);

        #ifdef DEBUG
        rtapi_print("Config motor\n");
        #endif

        printf("Config motor\n");
        config_motor(&motors[i]);

        #ifdef DEBUG
        rtapi_print("end spi convo\n");
        #endif

        printf("End spi conversation\n");
        end_spi_conversation();
    }

    #ifdef DEBUG
    rtapi_print("exit config_tmc5041\n");
    #endif
}

void teardown_tmc5041(joint_t motors[])
{
    // Configure chip and motors
    // HACK
    size_t size = 4;
    for (size_t i = 0; i < size; i++)
    {
        start_spi_conversation(motors[i].chip);
        reset_motor(&motors[i]);
        end_spi_conversation();
    }
}


bool setup_once(joint_t * motors, uint8_t motor_count) {

    // Intialize Broadcom driver
    printf("Init BCM2835\n");
    int init_success = bcm2835_init();

    // Set up RPi GPIO
    printf("Set up GPIO\n");
    setup_gpio();

    // Initialize SPI
    printf("Set up SPI\n");
    setup_spi0();
    setup_spi1();

    // Configure TMC5041
    printf("Set up TMC5041\n");
    config_tmc5041(motors, motor_count);

    return FALSE;
}

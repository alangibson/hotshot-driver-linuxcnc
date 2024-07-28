#include "stdio.h"
#include <math.h>
#include "bcm2835.h"
#include "rpi.h"
#include "tmc5041.h"
#include <unistd.h>


// 0 = X, 1 = YL, 2 = YR, 3 = Z
#define MOTOR_COUNT 1
joint_t motors[MOTOR_COUNT];

// FIXME copied from hotshot.comp
/**
 * Set up all hardware. 
 * Should only ever be called once.
 */
bool setup_once() {

    printf("Intialize Broadcom driver\n");

    // Intialize Broadcom driver
    bcm2835_init();

    printf("Set up RPi GPIO\n");

    // Set up RPi GPIO
    rpi_setup_gpio();

    printf("Set up RPi SPI\n");

    // Set up RPi SPI
    rpi_setup_spi0();
    rpi_setup_spi1();

    printf("Initialize TMC5041s\n");

    // Initialize TMC5041s
    config_tmc5041(motors, MOTOR_COUNT);

    return FALSE;
}

int main() {

    printf("Configure motors\n");

    // Configure motors
    //
    // X
    pin_position_t position_cmd = 100000;
    pin_enable_t is_enabled = 1;
    int axis_x_pitch = 2;
    int axis_x_teeth = 200;
    uint16_t axis_x_microsteps_cmd = 256;
    pin_velocity_t axis_x_max_velocity_cmd = 100.0;
    float axis_x_max_acceleration_cmd = 100.0;
    uint32_t axis_x_microstep_per_mm = microsteps_per_mm(axis_x_teeth, axis_x_pitch * axis_x_teeth, axis_x_microsteps_cmd);
    float64_t tmc_max_velocity_fb = fabs(axis_x_max_velocity_cmd) * axis_x_microstep_per_mm;
    uint32_t tmc_max_acceleration_fb = mm_to_microsteps(axis_x_microstep_per_mm, axis_x_max_acceleration_cmd);
    bool sg_stop_cmd = 1;
    pin_sg_thresh_t sg_thresh_cmd = 8;
    motors[0] = (joint_t) {
        .chip                               = 0,
        .motor                              = 0,
        .pitch                              = axis_x_pitch,
        .teeth                              = axis_x_teeth,
        .is_enabled                         = &is_enabled,
        .is_homing                          = FALSE,
        .mm_per_rev                         = axis_x_pitch * axis_x_teeth,
        .microstep_per_mm                   = axis_x_microstep_per_mm,
        .position_cmd                       = &position_cmd,
        .max_velocity_cmd                   = &axis_x_max_velocity_cmd,
        .max_acceleration_cmd               = axis_x_max_acceleration_cmd,
        .microsteps_cmd                     = axis_x_microsteps_cmd,
        .tmc_mres_cmd                       = microsteps_to_tmc_mres(axis_x_microsteps_cmd),
        .tmc_run_current_cmd                = 30,
        .tmc_hold_current_cmd               = 3,
        .tmc_cs_thresh_cmd                  = 50000,
        .tmc_sg_stop_cmd                    = &sg_stop_cmd,
        .tmc_sg_thresh_cmd                  = &sg_thresh_cmd,
        .position_fb                        = malloc(sizeof(pin_position_t)),
        .velocity_fb                        = malloc(sizeof(pin_velocity_t)),
        .tmc_max_velocity_fb                = &tmc_max_velocity_fb,
        .tmc_max_acceleration_fb            = &tmc_max_acceleration_fb,
        .sg_stop_fb                         = malloc(sizeof(pin_sg_stop_t)),
        .home_sw_fb                         = malloc(sizeof(pin_home_sw_t)),
        .tmc_position_fb                    = malloc(sizeof(pin_tmc_position_t)),
        .tmc_velocity_fb                    = malloc(sizeof(pin_tmc_velocity_t)),
        .tmc_motor_standstill_fb            = malloc(sizeof(pin_tmc_motor_standstill_t)),
        .tmc_motor_full_stepping_fb         = malloc(sizeof(pin_tmc_motor_full_stepping_t)),
        .tmc_motor_overtemp_warning_fb      = malloc(sizeof(pin_tmc_motor_overtemp_warning_t)),
        .tmc_motor_overtemp_alarm_fb        = malloc(sizeof(pin_tmc_motor_overtemp_alarm_t)),
        .tmc_motor_load_fb                  = malloc(sizeof(pin_tmc_motor_load_t)),
        .tmc_motor_current_fb               = malloc(sizeof(pin_tmc_motor_current_t)),
        .tmc_motor_stall_fb                 = malloc(sizeof(pin_tmc_motor_stall_t)),
        .tmc_microstep_resolution_fb        = malloc(sizeof(pin_tmc_microstep_resolution_t)),
        .torch_breakaway_fb                 = malloc(sizeof(pin_torch_breakaway_t)),
    };

    printf("Setup once\n");
    setup_once();
    
    rpi_start_spi_conversation(motors[0].chip);
    printf("Set XTARGET\n");
    int32_t expected_xtarget = position_cmd;
    spi_status_t spi_status = tmc5041_set_register_XTARGET(&motors[0], expected_xtarget);
    rpi_end_spi_conversation();

    sleep(3);

    rpi_start_spi_conversation(motors[0].chip);
    printf("Check connection to chip\n");
    int32_t actual_xtarget = tmc5041_get_register_XTARGET(&motors[0]);
    int32_t xactual = tmc5041_get_register_XACTUAL(&motors[0]);
    int32_t vactual = tmc5041_get_register_VACTUAL(&motors[0]);
    ihold_irun_register_t ihold_irun = tmc5041_get_register_IHOLD_IRUN(&motors[0]);
    chopconf_register_t chopconf = tmc5041_get_register_CHOPCONF(&motors[0]);
    printf("spi_status: velocity_reached1=%d, driver_error1=%d, reset_flag=%d\n", 
        spi_status.velocity_reached1, spi_status.driver_error1, spi_status.reset_flag);
    printf("XACTUAL: %d\n", xactual);
    printf("VACTUAL: %d\n", vactual);
    printf("XTARGET: expected=%d, actual=%d\n", expected_xtarget, actual_xtarget);
    printf("IHOLD_IRUN: ihold=%d, irun=%d\n", ihold_irun.ihold, ihold_irun.irun);
    printf("CHOPCONF: mres=%d, toff=%d, tbl=%d, hstrt=%d, hend=%d\n", 
        chopconf.mres, chopconf.toff, chopconf.tbl, chopconf.hstrt, chopconf.hend);
    rpi_end_spi_conversation();

    // printf("Shutdown motors\n");
    // rpi_start_spi_conversation(motors[0].chip);
    // teardown_tmc5041(motors, MOTOR_COUNT);
    // rpi_end_spi_conversation();

    // printf("Loop over motors\n");

    // for (int i = 0; i < MOTOR_COUNT; i++) 
    // {
    //     printf("Handle joint\n");

    //     // TODO in a loop
    //     // handle joints
    //     handle_joint(&motors[i]);
    //     // handle_joint(&motors[1]);
    //     // handle_joint(&motors[2]);
    //     // handle_joint(&motors[3]);

    //     printf("Report joint\n");

    //     // TODO report on joint state
    //     report_joint(&motors[i]);
    //     // report_joint(&motors[1]);
    //     // report_joint(&motors[2]);
    //     // report_joint(&motors[3]);

    // }
}
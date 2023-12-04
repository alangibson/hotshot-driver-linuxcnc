#include "stdio.h"
#include <signal.h>
#include "time.h"
// #include "hotshot_init.h"
// #include "tmc5041.h"

#include "rpi.c"
// Also does #include "tmc5041.h"
#include "tmc5041.c"
#include "hotshot_init.c"

// in us
#define SERVO_PERIOD 1000

volatile bool ctrlCPressed = false;

// Function to be executed when Ctrl+C is pressed
void handleCtrlC(int signal) {
    printf("\nCtrl+C pressed. Exiting...\n");
    ctrlCPressed = true;
}

int main() {
    // Set up the Ctrl+C signal handler
    if (signal(SIGINT, handleCtrlC) == SIG_ERR) {
        perror("Signal setup failed");
        return 1;
    }

    uint32_t report_period = 1000000; // us
    uint32_t last_move = 0;
    uint32_t last_report = 0;

    int32_t xtarget = 0;

    // Input pins set with setp
    volatile uint32_t axis_x_pitch = 2;
    volatile uint32_t axis_x_teeth = 20;
    volatile uint32_t axis_x_tmc_sg_thresh_cmd = 5;
    volatile uint32_t axis_x_tmc_cs_thresh_cmd = 10000;
    volatile uint32_t axis_x_tmc_hold_current_cmd = 6;
    volatile uint32_t axis_x_tmc_run_current_cmd = 23;
    volatile bool axis_x_tmc_sg_stop_cmd = 0;
    // Output pins derived from other input pins
    volatile float64_t axis_x_tmc_max_velocity_cmd = 80000;
    volatile uint32_t axis_x_tmc_max_acceleration_cmd = 8000;
    // Input pins set by EMC
    volatile bool axis_x_is_enabled = 1;
    volatile bool axis_x_is_homing = 0;
    volatile pin_tmc_position_t axis_x_tmc_position_cmd;
    volatile pin_tmc_position_t axis_x_tmc_velocity_cmd;
    motors[0] = (joint_t) {
        .chip                       = BCM2835_SPI_CS0,
        .motor                      = 0,
        .pitch                      = &axis_x_pitch,
        .teeth                      = &axis_x_teeth,
        .is_enabled                 = &axis_x_is_enabled,
        .is_homing                  = &axis_x_is_homing,
        .tmc_run_current_cmd        = &axis_x_tmc_run_current_cmd,
        .tmc_hold_current_cmd       = &axis_x_tmc_hold_current_cmd,
        .tmc_max_velocity_cmd       = &axis_x_tmc_max_velocity_cmd,
        .tmc_max_acceleration_cmd   = &axis_x_tmc_max_acceleration_cmd,
        .tmc_position_cmd           = &axis_x_tmc_position_cmd,
        .tmc_velocity_cmd           = &axis_x_tmc_velocity_cmd,
        .tmc_sg_stop_cmd            = &axis_x_tmc_sg_stop_cmd,
        .tmc_cs_thresh_cmd          = &axis_x_tmc_cs_thresh_cmd,
        .tmc_sg_thresh_cmd          = &axis_x_tmc_sg_thresh_cmd
    };

    // Input pins set with setp
    volatile uint32_t axis_yl_pitch = 2;
    volatile uint32_t axis_yl_teeth = 20;
    volatile uint32_t axis_yl_tmc_sg_thresh_cmd = 5;
    volatile uint32_t axis_yl_tmc_cs_thresh_cmd = 10000;
    volatile uint32_t axis_yl_tmc_hold_current_cmd = 6;
    volatile uint32_t axis_yl_tmc_run_current_cmd = 23;
    volatile bool axis_yl_tmc_sg_stop_cmd = 0;
    // Output pins derived from other input pins
    volatile float64_t axis_yl_tmc_max_velocity_cmd = 80000;
    volatile uint32_t axis_yl_tmc_max_acceleration_cmd = 8000;
    // Input pins set by EMC
    volatile bool axis_yl_is_enabled = 1;
    volatile bool axis_yl_is_homing = 0;
    volatile pin_tmc_position_t axis_yl_tmc_position_cmd;
    volatile pin_tmc_position_t axis_yl_tmc_velocity_cmd;
    motors[1] = (joint_t) {
        .chip                       = BCM2835_SPI_CS0,
        .motor                      = 1,
        .pitch                      = &axis_yl_pitch,
        .teeth                      = &axis_yl_teeth,
        .is_enabled                 = &axis_yl_is_enabled,
        .is_homing                  = &axis_yl_is_homing,
        .tmc_run_current_cmd        = &axis_yl_tmc_run_current_cmd,
        .tmc_hold_current_cmd       = &axis_yl_tmc_hold_current_cmd,
        .tmc_max_velocity_cmd       = &axis_yl_tmc_max_velocity_cmd,
        .tmc_max_acceleration_cmd   = &axis_yl_tmc_max_acceleration_cmd,
        .tmc_position_cmd           = &axis_yl_tmc_position_cmd,
        .tmc_velocity_cmd           = &axis_yl_tmc_velocity_cmd,
        .tmc_sg_stop_cmd            = &axis_yl_tmc_sg_stop_cmd,
        .tmc_cs_thresh_cmd          = &axis_yl_tmc_cs_thresh_cmd,
        .tmc_sg_thresh_cmd          = &axis_yl_tmc_sg_thresh_cmd
    };

    // Input pins set with setp
    volatile uint32_t axis_yr_pitch = 2;
    volatile uint32_t axis_yr_teeth = 20;
    volatile uint32_t axis_yr_tmc_sg_thresh_cmd = 5;
    volatile uint32_t axis_yr_tmc_cs_thresh_cmd = 10000;
    volatile uint32_t axis_yr_tmc_hold_current_cmd = 6;
    volatile uint32_t axis_yr_tmc_run_current_cmd = 23;
    volatile bool axis_yr_tmc_sg_stop_cmd = 0;
    // Output pins derived from other input pins
    volatile float64_t axis_yr_tmc_max_velocity_cmd = 80000;
    volatile uint32_t axis_yr_tmc_max_acceleration_cmd = 8000;
    // Input pins set by EMC
    volatile bool axis_yr_is_enabled = 1;
    volatile bool axis_yr_is_homing = 0;
    volatile pin_tmc_position_t axis_yr_tmc_position_cmd;
    volatile pin_tmc_position_t axis_yr_tmc_velocity_cmd;
    motors[2] = (joint_t) {
        .chip                       = BCM2835_SPI_CS1,
        .motor                      = 0,
        .pitch                      = &axis_yr_pitch,
        .teeth                      = &axis_yr_teeth,
        .is_enabled                 = &axis_yr_is_enabled,
        .is_homing                  = &axis_yr_is_homing,
        .tmc_run_current_cmd        = &axis_yr_tmc_run_current_cmd,
        .tmc_hold_current_cmd       = &axis_yr_tmc_hold_current_cmd,
        .tmc_max_velocity_cmd       = &axis_yr_tmc_max_velocity_cmd,
        .tmc_max_acceleration_cmd   = &axis_yr_tmc_max_acceleration_cmd,
        .tmc_position_cmd           = &axis_yr_tmc_position_cmd,
        .tmc_velocity_cmd           = &axis_yr_tmc_velocity_cmd,
        .tmc_sg_stop_cmd            = &axis_yr_tmc_sg_stop_cmd,
        .tmc_cs_thresh_cmd          = &axis_yr_tmc_cs_thresh_cmd,
        .tmc_sg_thresh_cmd          = &axis_yr_tmc_sg_thresh_cmd
    };

    // Input pins set with setp
    volatile uint32_t axis_z_pitch = 2;
    volatile uint32_t axis_z_teeth = 20;
    volatile uint32_t axis_z_tmc_sg_thresh_cmd = 5;
    volatile uint32_t axis_z_tmc_cs_thresh_cmd = 10000;
    volatile uint32_t axis_z_tmc_hold_current_cmd = 6;
    volatile uint32_t axis_z_tmc_run_current_cmd = 23;
    volatile bool axis_z_tmc_sg_stop_cmd = 0;
    // Output pins derived from other input pins
    volatile float64_t axis_z_tmc_max_velocity_cmd = 80000;
    volatile uint32_t axis_z_tmc_max_acceleration_cmd = 8000;
    // Input pins set by EMC
    volatile bool axis_z_is_enabled = 1;
    volatile bool axis_z_is_homing = 0;
    volatile pin_tmc_position_t axis_z_tmc_position_cmd;
    volatile pin_tmc_position_t axis_z_tmc_velocity_cmd;
    motors[3] = (joint_t) {
        .chip                       = BCM2835_SPI_CS1,
        .motor                      = 1,
        .pitch                      = &axis_z_pitch,
        .teeth                      = &axis_z_teeth,
        .is_enabled                 = &axis_z_is_enabled,
        .is_homing                  = &axis_z_is_homing,
        .tmc_run_current_cmd        = &axis_z_tmc_run_current_cmd,
        .tmc_hold_current_cmd       = &axis_z_tmc_hold_current_cmd,
        .tmc_max_velocity_cmd       = &axis_z_tmc_max_velocity_cmd,
        .tmc_max_acceleration_cmd   = &axis_z_tmc_max_acceleration_cmd,
        .tmc_position_cmd           = &axis_z_tmc_position_cmd,
        .tmc_velocity_cmd           = &axis_z_tmc_velocity_cmd,
        .tmc_sg_stop_cmd            = &axis_z_tmc_sg_stop_cmd,
        .tmc_cs_thresh_cmd          = &axis_z_tmc_cs_thresh_cmd,
        .tmc_sg_thresh_cmd          = &axis_z_tmc_sg_thresh_cmd
    };

    printf("Calling setup_once\n");
    setup_once(motors, MOTOR_COUNT);
    printf("Done calling setup_once\n");

    // printf("Move motor %d to %d\n", motors[0].motor, 1000000);
    // moveTo(&motors[0], 1000000);

    // printf("Move motor %d to %d\n", motors[0].motor, 1000000);
    // tmc5041_set_register_XTARGET(&motors[0], 1000000);

    // Move all motors
    // for (size_t i = 0; i < MOTOR_COUNT; i++) {
    //     // xtarget += 10;
    //     xtarget = 1000000;
    //     bcm2835_spi_chipSelect(motors[i].chip);
    //     spi_status_t spi_status = tmc5041_set_register_XTARGET(&motors[i], xtarget);
    //     bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
    //     // follow() requires motor->position_cmd and motor->microstep_per_mm
    //     // follow(&motors[i]);
    // }

    // This simulates the HAL loop
    while (!ctrlCPressed) {
        uint32_t tick = clock();

        // Move all motors
        if (tick - last_move >= SERVO_PERIOD) {
            // printf("%d - %d = %d >= %d\n", tick, last_move, tick - last_move, SERVO_PERIOD);
            last_move = tick;

            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                xtarget += 100;
                bcm2835_spi_chipSelect(motors[i].chip);
                spi_status_t spi_status = tmc5041_set_register_XTARGET(&motors[i], xtarget);
            }
        }

        // Report on all motors
        if ((tick - last_report) >= report_period) {
            last_report = tick;

            printf("-------------------------------------------------------------------\n");
            
            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                bcm2835_spi_chipSelect(motors[i].chip);
    
                printf("chip=%d\n", motors[i].chip);

                int32_t gstat = tmc5041_readInt(&motors[i], TMC5041_GSTAT);
                printf("GSTAT\n");
                printf("    reset=%d\n", FIELD_GET(gstat, TMC5041_RESET_MASK, TMC5041_RESET_SHIFT));
                printf("    drv_err_1=%d\n", FIELD_GET(gstat, TMC5041_DRV_ERR1_MASK, TMC5041_DRV_ERR1_SHIFT));
                printf("    drv_err_2=%d\n", FIELD_GET(gstat, TMC5041_DRV_ERR2_MASK, TMC5041_DRV_ERR2_SHIFT));
                printf("    undervoltage=%d\n", FIELD_GET(gstat, TMC5041_UV_CP_MASK, TMC5041_UV_CP_SHIFT));

                printf("chip=%d motor=%d\n", motors[i].chip, motors[i].motor);

                // printf("SPI_STATUS motor=%d\n", motors[i].motor);
                // printf("    driver_error1=%d\n", spi_status.driver_error1);
                // printf("    driver_error2=%d\n", spi_status.driver_error2);
                // printf("    reset_flag=%d\n", spi_status.reset_flag);
                // printf("    status_stop_l1=%d\n", spi_status.status_stop_l1);
                // printf("    status_stop_l2=%d\n", spi_status.status_stop_l2);
                // printf("    velocity_reached1=%d\n", spi_status.velocity_reached1);
                // printf("    velocity_reached2=%d\n", spi_status.velocity_reached2);

                printf("XTARGET motor=%d xtarget=%d\n", motors[i].motor, xtarget);

                int32_t xactual = tmc5041_get_register_XACTUAL(&motors[i]);
                printf("XACTUAL motor=%d xactual=%d\n", motors[i].motor, xactual);

                int32_t vactual = tmc5041_get_register_VACTUAL(&motors[i]);
                printf("VACTUAL motor=%d vactual=%d\n", motors[i].motor, vactual);
                drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(&motors[i]);
                printf("DRV_STATUS motor=%d\n", motors[i].motor);
                printf("    standstill=%d\n", drv_status.standstill);
                printf("    full_stepping=%d\n", drv_status.full_stepping);
                printf("    overtemp_warning=%d\n", drv_status.overtemp_warning);
                printf("    overtemp_alarm=%d\n", drv_status.overtemp_alarm);
                printf("    sg_result=%d\n", drv_status.sg_result);
                printf("    cs_actual=%d\n", drv_status.cs_actual);
                printf("    sg_status=%d\n", drv_status.sg_status);
                
                ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&motors[i]);
                printf("RAMP_STAT motor=%d\n", motors[i].motor);
                printf("    status_sg=%d\n", ramp_stat.status_sg);
                printf("    second_move=%d\n", ramp_stat.second_move);
                printf("    t_zerowait_active=%d\n", ramp_stat.t_zerowait_active);
                printf("    vzero=%d\n", ramp_stat.vzero);
                printf("    position_reached=%d\n", ramp_stat.position_reached);
                printf("    velocity_reached=%d\n", ramp_stat.velocity_reached);
                printf("    event_pos_reached=%d\n", ramp_stat.event_pos_reached);
                printf("    event_stop_sg=%d\n", ramp_stat.event_stop_sg);
                printf("    event_stop_r=%d\n", ramp_stat.event_stop_r);
                printf("    event_stop_l=%d\n", ramp_stat.event_stop_l);
                printf("    status_latch_r=%d\n", ramp_stat.status_latch_r);
                printf("    status_latch_l=%d\n", ramp_stat.status_latch_l);
                printf("    status_stop_r=%d\n", ramp_stat.status_stop_r);
                printf("    status_stop_l=%d\n", ramp_stat.status_stop_l);

                int32_t ramp_mode = tmc5041_readInt(&motors[i], TMC5041_RAMPMODE(motors[i].motor));
                printf("RAMPMODE motor=%d ramp_mode=%d\n", motors[i].motor, ramp_mode);

                printf("\n");

                // bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
            }

            printf("-------------------------------------------------------------------\n");
        }
    }

    printf("TODO shut down driver\n");

    return 0;
}

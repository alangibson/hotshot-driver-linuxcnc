#include "stdio.h"
#include <signal.h>
#include "time.h"
#include "tmc/helpers/Types.h"
#include "global.h"
#include "rpi.h"
#include "hotshot.hal.h"

// in us
#define SERVO_PERIOD 1000

#undef MOTOR_COUNT
#define MOTOR_COUNT 1

bool ctrlCPressed = false;
bool needs_setup = TRUE;
// 0 = X, 1 = YL, 2 = YR, 3 = Z
joint_t motors[MOTOR_COUNT];

// Function to be executed when Ctrl+C is pressed
void handleCtrlC(int signal) {
    printf("\nCtrl+C pressed. Exiting...\n");
    ctrlCPressed = true;
}

int main() {
    printf("Entering main\n");

    // Set up the Ctrl+C signal handler
    if (signal(SIGINT, handleCtrlC) == SIG_ERR) {
        perror("Signal setup failed");
        return 1;
    }

    uint32_t report_period = 1000000; // us
    uint32_t last_move = 0;
    uint32_t last_report = 0;

    int32_t xtarget = 0;

    // This simulates the HAL loop
    while (!ctrlCPressed) {
        uint32_t tick = clock();

        // Move all motors
        if (tick - last_move >= SERVO_PERIOD) {
            last_move = tick;

            xtarget += 10; // in mm

            if (needs_setup) {
                
                printf("Running setup once\n");

                // Statically set in INI and then set via setp
                // FIXME not being used
                float axis_x_max_velocity_cmd       = 400;
                float axis_x_max_acceleration_cmd   = 100;
                // Set via setp
                uint32_t axis_x_pitch               = 2;
                uint32_t axis_x_teeth               = 20;
                uint32_t axis_x_microstep_per_mm    = 256;
                // Set dynamically by HAL
                bool axis_x_enable                  = TRUE;
                bool axis_x_is_homing               = FALSE;
                float64_t axis_x_position_cmd       = 0; // TODO
                // Set dynamically by Hotshot
                float64_t axis_x_position_fb;
                float64_t axis_x_velocity_fb;
                bool axis_x_sg_stop_fb;
                bool axis_x_home_sw;
                bool torch_breakaway;

                // Set via setp
                uint32_t axis_x_tmc_run_current_cmd         = 23;
                uint32_t axis_x_tmc_hold_current_cmd        = 6;
                bool axis_x_tmc_sg_stop_cmd                 = TRUE;
                uint32_t axis_x_tmc_cs_thresh_cmd           = 40000; // in ppt
                uint32_t axis_x_tmc_sg_thresh_cmd           = 5;
                // Set dynamically by Hotshot
                // FIXME should be derived from axis_x_max_velocity_cmd
                float64_t axis_x_tmc_max_velocity_cmd       = 80000;
                // FIXME should be derived from axis_x_max_acceleration_cmd
                uint32_t axis_x_tmc_max_acceleration_cmd    = 6000;
                int32_t axis_x_tmc_position_fb;
                int32_t axis_x_tmc_velocity_fb;
                bool axis_X_motor_standstill_fb;
                bool axis_x_motor_full_stepping_fb;
                bool axis_x_motor_overtemp_warning_fb;
                bool axis_x_motor_overtemp_alarm_fb;
                int32_t axis_x_motor_load_fb;
                uint32_t axis_x_motor_current_fb;
                bool axis_x_motor_stall_fb;
                uint32_t axis_x_microstep_resolution_fb;

                tmc5041_chip_t chip0 = (tmc5041_chip_t) {
                    .chip = 0
                };
                tmc5041_chip_t chip1 = (tmc5041_chip_t) {
                    .chip = 1
                };
                motors[0] = (joint_t) {
                    // Statically set in INI and then set via setp
                    .max_velocity_cmd       = axis_x_max_velocity_cmd,
                    .max_acceleration_cmd   = axis_x_max_acceleration_cmd,
                    
                    // Set via setp
                    .pitch                  = &axis_x_pitch,
                    .teeth                  = &axis_x_teeth,
                    .microstep_per_mm       = &axis_x_microstep_per_mm,
                    // .microsteps_cmd         = axis_x_microsteps_cmd,
                    // .mm_per_rev             = axis_x_pitch * axis_x_teeth,
                    

                    // Set dynamically by HAL
                    .is_enabled             = &axis_x_enable,
                    .is_homing              = &axis_x_is_homing,
                    .position_cmd           = &axis_x_position_cmd,
                    
                    // Set dynamically by Hotshot
                    .position_fb            = &axis_x_position_fb,
                    .velocity_fb            = &axis_x_velocity_fb,
                    .sg_stop_fb             = &axis_x_sg_stop_fb,
                    .home_sw_fb             = &axis_x_home_sw,
                    .torch_breakaway_fb     = &torch_breakaway,
                    
                    .tmc = (tmc5041_motor_t) {
                        // Statically set in code
                        .chip = chip0,
                        .motor = 0,

                        // Set via setp
                        .run_current_cmd        = &axis_x_tmc_run_current_cmd,
                        .hold_current_cmd       = &axis_x_tmc_hold_current_cmd,
                        .sg_stop_cmd            = &axis_x_tmc_sg_stop_cmd,
                        .cs_thresh_cmd          = &axis_x_tmc_cs_thresh_cmd,
                        .sg_thresh_cmd          = &axis_x_tmc_sg_thresh_cmd,

                        // Set dynamically by Hotshot
                        // These also serve as a feedback 
                        .position_fb           = &axis_x_tmc_position_fb,
                        .velocity_fb           = &axis_x_tmc_velocity_fb,
                        .motor_standstill_fb    = &axis_X_motor_standstill_fb,

                        .motor_full_stepping_fb     = &axis_x_motor_full_stepping_fb,
                        .motor_overtemp_warning_fb  = &axis_x_motor_overtemp_warning_fb,
                        .motor_overtemp_alarm_fb    = &axis_x_motor_overtemp_alarm_fb,
                        .motor_load_fb              = &axis_x_motor_load_fb,
                        .motor_current_fb           = &axis_x_motor_current_fb,
                        .motor_stall_fb             = &axis_x_motor_stall_fb,
                        .microstep_resolution_fb    = &axis_x_microstep_resolution_fb,

                        .max_velocity_cmd           = &axis_x_tmc_max_velocity_cmd,
                        .max_acceleration_cmd       = &axis_x_tmc_max_acceleration_cmd,

                        // Set dynamically by Hotshot
                        // .max_velocity_fb        = NULL,
                        // .max_acceleration_fb    = NULL
                        // .max_velocity_cmd        = (float)fabs(axis_x_max_velocity_cmd) * axis_x_microstep_per_mm;
                        // .max_acceleration_cmd    = mm_to_microsteps(axis_x_microstep_per_mm, axis_x_max_acceleration_cmd);
                    }
                };
                // motors[1] = (joint_t) {
                //     .tmc = (tmc5041_motor_t) {
                //         .chip = chip0
                //     }
                // };
                // motors[2] = (joint_t) {
                //     .tmc = (tmc5041_motor_t) {
                //         .chip = chip1,
                //     }
                // };
                // motors[3] = (joint_t) {
                //     .tmc = (tmc5041_motor_t) {
                //         .chip = chip1
                //     }
                // };
                needs_setup = hotshot_init(motors, MOTOR_COUNT);
            }

            // Fake setting position_cmd pin like HAL would
            *motors[0].position_cmd = xtarget;
            *motors[0].is_enabled = 1;
            // *motors[0].sg_stop_fb = 0;

            handle_joints(motors, MOTOR_COUNT);
        }

        // Report on all motors
        if ((tick - last_report) >= report_period) {
            last_report = tick;

            printf("-------------------------------------------------------------------\n");
            
            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                rpi_spi_select(motors[i].tmc.chip.chip);
    
                printf("chip=%d\n", motors[i].tmc.chip.chip);

                int32_t gstat = tmc5041_readInt(&motors[i].tmc, TMC5041_GSTAT);
                printf("GSTAT\n");
                printf("    reset=%d\n", FIELD_GET(gstat, TMC5041_RESET_MASK, TMC5041_RESET_SHIFT));
                printf("    drv_err_1=%d\n", FIELD_GET(gstat, TMC5041_DRV_ERR1_MASK, TMC5041_DRV_ERR1_SHIFT));
                printf("    drv_err_2=%d\n", FIELD_GET(gstat, TMC5041_DRV_ERR2_MASK, TMC5041_DRV_ERR2_SHIFT));
                printf("    undervoltage=%d\n", FIELD_GET(gstat, TMC5041_UV_CP_MASK, TMC5041_UV_CP_SHIFT));

                printf("chip=%d motor=%d\n", motors[i].tmc.chip.chip, motors[i].tmc.motor);

                // printf("SPI_STATUS motor=%d\n", motors[i].tmc.motor);
                // printf("    driver_error1=%d\n", spi_status.driver_error1);
                // printf("    driver_error2=%d\n", spi_status.driver_error2);
                // printf("    reset_flag=%d\n", spi_status.reset_flag);
                // printf("    status_stop_l1=%d\n", spi_status.status_stop_l1);
                // printf("    status_stop_l2=%d\n", spi_status.status_stop_l2);
                // printf("    velocity_reached1=%d\n", spi_status.velocity_reached1);
                // printf("    velocity_reached2=%d\n", spi_status.velocity_reached2);

                printf("xtarget motor=%d xtarget=%d\n", motors[i].tmc.motor, xtarget);

                int32_t xactual = tmc5041_get_register_XACTUAL(&motors[i].tmc);
                printf("XACTUAL motor=%d xactual=%d\n", motors[i].tmc.motor, xactual);

                int32_t vactual = tmc5041_get_register_VACTUAL(&motors[i].tmc);
                printf("VACTUAL motor=%d vactual=%d\n", motors[i].tmc.motor, vactual);
                drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(&motors[i].tmc);
                printf("DRV_STATUS motor=%d\n", motors[i].tmc.motor);
                printf("    standstill=%d\n", drv_status.standstill);
                printf("    full_stepping=%d\n", drv_status.full_stepping);
                printf("    overtemp_warning=%d\n", drv_status.overtemp_warning);
                printf("    overtemp_alarm=%d\n", drv_status.overtemp_alarm);
                printf("    sg_result=%d\n", drv_status.sg_result);
                printf("    cs_actual=%d\n", drv_status.cs_actual);
                printf("    sg_status=%d\n", drv_status.sg_status);
                
                // ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&motors[i].tmc);
                // printf("RAMP_STAT motor=%d\n", motors[i].tmc.motor);
                // printf("    status_sg=%d\n", ramp_stat.status_sg);
                // printf("    second_move=%d\n", ramp_stat.second_move);
                // printf("    t_zerowait_active=%d\n", ramp_stat.t_zerowait_active);
                // printf("    vzero=%d\n", ramp_stat.vzero);
                // printf("    position_reached=%d\n", ramp_stat.position_reached);
                // printf("    velocity_reached=%d\n", ramp_stat.velocity_reached);
                // printf("    event_pos_reached=%d\n", ramp_stat.event_pos_reached);
                // printf("    event_stop_sg=%d\n", ramp_stat.event_stop_sg);
                // printf("    event_stop_r=%d\n", ramp_stat.event_stop_r);
                // printf("    event_stop_l=%d\n", ramp_stat.event_stop_l);
                // printf("    status_latch_r=%d\n", ramp_stat.status_latch_r);
                // printf("    status_latch_l=%d\n", ramp_stat.status_latch_l);
                // printf("    status_stop_r=%d\n", ramp_stat.status_stop_r);
                // printf("    status_stop_l=%d\n", ramp_stat.status_stop_l);

                int32_t ramp_mode = tmc5041_readInt(&motors[i].tmc, TMC5041_RAMPMODE(motors[i].tmc.motor));
                printf("RAMPMODE motor=%d ramp_mode=%d\n", motors[i].tmc.motor, ramp_mode);

                printf("\n");
            }

            printf("-------------------------------------------------------------------\n");
        }
    }

    printf("Shutting down\n");
    hotshot_end(motors, MOTOR_COUNT);

    return 0;
}

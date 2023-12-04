#include "stdio.h"
#include <signal.h>
#include "time.h"
#include "tmc/helpers/Types.h"
#include "global.h"
#include "rpi.h"
#include "hotshot.hal.h"

// in us
#define SERVO_PERIOD 1000

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

            xtarget += 100;

            // TODO fill motors[]
            if (needs_setup) {
                tmc5041_chip_t chip0 = (tmc5041_chip_t) {
                    .chip = 0
                };
                tmc5041_chip_t chip1 = (tmc5041_chip_t) {
                    .chip = 1
                };
                motors[0] = (joint_t) {
                    .tmc = (tmc5041_motor_t) {
                        .chip = chip0
                    }
                };
                needs_setup = hotshot_init(motors, MOTOR_COUNT);
            }

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

                printf("XTARGET motor=%d xtarget=%d\n", motors[i].tmc.motor, xtarget);

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
                
                ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&motors[i].tmc);
                printf("RAMP_STAT motor=%d\n", motors[i].tmc.motor);
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

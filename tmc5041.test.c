#include "stdio.h"
#include "signal.h"
#include "time.h"
#include "rpi.h"
#include "tmc5041.h"

// in us
#define SERVO_PERIOD 1000

volatile bool ctrlCPressed = false;

// 0 = X, 1 = YL, 2 = YR, 3 = Z
#define MOTOR_COUNT 4
tmc5041_motor_t tmc5041_motors[MOTOR_COUNT];

// Function to be executed when Ctrl+C is pressed
void handleCtrlC(int signal) {
    printf("\nCtrl+C pressed. Exiting...\n");
    ctrlCPressed = true;
}

// int main() {
//     rpi_init();

//     tmc5041_init(tmc5041_motors, MOTOR_COUNT);

//     rpi_spi_select(tmc5041_motors[i].chip);

//     // TODO go to position

//     rpi_spi_unselect();

//     tmc5041_end();

//     rpi_end();
// }

int main() {
    // Set up the Ctrl+C signal handler
    if (signal(SIGINT, handleCtrlC) == SIG_ERR) {
        perror("Signal setup failed");
        return 1;
    }

    printf("Initializing Raspberry Pi\n");
    rpi_init();

    uint32_t report_period = 1000000; // us
    uint32_t last_move = 0;
    uint32_t last_report = 0;

    int32_t xtarget = 0;

    // Initialize the chip and motor values
    volatile tmc_chip_t chip0 = 0;
    volatile tmc_chip_t chip1 = 1;
    volatile tmc_motor_t motor0 = 0;
    volatile tmc_motor_t motor1 = 1;

    // X axis - chip 0, motor 0
    tmc5041_motors[0] = *tmc5041_motor_create(0, 0);
    // Initialize command values
    *tmc5041_motors[0].sg_thresh_cmd = 5;
    *tmc5041_motors[0].cs_thresh_cmd = 10000;
    *tmc5041_motors[0].hold_current_cmd = 8;
    *tmc5041_motors[0].run_current_cmd = 23;
    *tmc5041_motors[0].sg_stop_cmd = 0;
    *tmc5041_motors[0].velocity_cmd = 80000;
    *tmc5041_motors[0].position_cmd = 0;
    tmc5041_motors[0].acceleration_cmd = 8000;
    tmc5041_motors[0].velocity_time_ref = tmc5041_velocity_time_ref(TMC5041_CLOCK_HZ);
    tmc5041_motors[0].acceleration_time_ref = tmc5041_acceleration_time_ref(TMC5041_CLOCK_HZ);
    tmc5041_motors[0].mres = tmc5041_microsteps_to_mres(256);
    *tmc5041_motors[0].chop_toff_cmd = 5;

    // YL axis - chip 0, motor 1
    tmc5041_motors[1] = *tmc5041_motor_create(0, 1);
    // Initialize command values
    *tmc5041_motors[1].sg_thresh_cmd = 5;
    *tmc5041_motors[1].cs_thresh_cmd = 10000;
    *tmc5041_motors[1].hold_current_cmd = 8;
    *tmc5041_motors[1].run_current_cmd = 23;
    *tmc5041_motors[1].sg_stop_cmd = 0;
    *tmc5041_motors[1].velocity_cmd = 80000;
    tmc5041_motors[1].acceleration_cmd = 8000;
    *tmc5041_motors[1].position_cmd = 0;
    *tmc5041_motors[1].chop_toff_cmd = 5;

    // YR axis - chip 1, motor 0
    tmc5041_motors[2] = *tmc5041_motor_create(1, 0);
    // Initialize command values
    *tmc5041_motors[2].sg_thresh_cmd = 5;
    *tmc5041_motors[2].cs_thresh_cmd = 10000;
    *tmc5041_motors[2].hold_current_cmd = 8;
    *tmc5041_motors[2].run_current_cmd = 23;
    *tmc5041_motors[2].sg_stop_cmd = 0;
    *tmc5041_motors[2].velocity_cmd = 80000;
    tmc5041_motors[2].acceleration_cmd = 8000;
    *tmc5041_motors[2].position_cmd = 0;
    *tmc5041_motors[2].chop_toff_cmd = 5;

    // Z axis - chip 1, motor 1
    tmc5041_motors[3] = *tmc5041_motor_create(1, 1);
    // Initialize command values
    *tmc5041_motors[3].sg_thresh_cmd = 5;
    *tmc5041_motors[3].cs_thresh_cmd = 10000;
    *tmc5041_motors[3].hold_current_cmd = 8;
    *tmc5041_motors[3].run_current_cmd = 23;
    *tmc5041_motors[3].sg_stop_cmd = 0;
    *tmc5041_motors[3].velocity_cmd = 80000;
    tmc5041_motors[3].acceleration_cmd = 8000;
    *tmc5041_motors[3].position_cmd = 0;
    *tmc5041_motors[3].chop_toff_cmd = 5;

    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        rpi_spi_select(*tmc5041_motors[i].chip);

        // TODO This doesnt do much of anything
        // motor_on(&tmc5041_motors[i]);

        tmc5041_pull_register_DRV_STATUS(&tmc5041_motors[i]);
        tmc5041_pull_register_RAMP_STAT(&tmc5041_motors[i]);
        tmc5041_pull_register_CHOPCONF(&tmc5041_motors[i]);
        tmc5041_log_motor_state(&tmc5041_motors[i]);

        // float64_t freq_scaling = tmc5041_frequency_scaling(&tmc5041_motors[i]);
        // printf("  Frequency scaling: %f\n", freq_scaling);

        // Power up motors
        tmc5041_motor_init(&tmc5041_motors[i]);
        
        rpi_spi_unselect();
    }

    return 0;

    // This simulates the HAL loop
    while (!ctrlCPressed) {
        uint32_t tick = clock();

        // Move all motors
        if (tick - last_move >= SERVO_PERIOD) {
            last_move = tick;

            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                xtarget += 100;
                rpi_spi_select(*tmc5041_motors[i].chip);
                spi_status_t spi_status = tmc5041_set_register_XTARGET(&tmc5041_motors[i], xtarget);
            }
        }

        // Report on all motors
        if ((tick - last_report) >= report_period) {
            last_report = tick;

            printf("-------------------------------------------------------------------\n");
            
            for (size_t i = 0; i < MOTOR_COUNT; i++) {
                rpi_spi_select(*tmc5041_motors[i].chip);
    
                printf("chip=%d motor=%d\n", *tmc5041_motors[i].chip, *tmc5041_motors[i].motor);

                int32_t gstat = tmc5041_readInt(&tmc5041_motors[i], TMC5041_GSTAT);
                printf("GSTAT\n");
                printf("    reset=%d\n", FIELD_GET(gstat, TMC5041_RESET_MASK, TMC5041_RESET_SHIFT));
                printf("    drv_err_1=%d\n", FIELD_GET(gstat, TMC5041_DRV_ERR1_MASK, TMC5041_DRV_ERR1_SHIFT));
                printf("    drv_err_2=%d\n", FIELD_GET(gstat, TMC5041_DRV_ERR2_MASK, TMC5041_DRV_ERR2_SHIFT));
                printf("    undervoltage=%d\n", FIELD_GET(gstat, TMC5041_UV_CP_MASK, TMC5041_UV_CP_SHIFT));

                // printf("SPI_STATUS motor=%d\n", tmc5041_motors[i].motor);
                // printf("    driver_error1=%d\n", spi_status.driver_error1);
                // printf("    driver_error2=%d\n", spi_status.driver_error2);
                // printf("    reset_flag=%d\n", spi_status.reset_flag);
                // printf("    status_stop_l1=%d\n", spi_status.status_stop_l1);
                // printf("    status_stop_l2=%d\n", spi_status.status_stop_l2);
                // printf("    velocity_reached1=%d\n", spi_status.velocity_reached1);
                // printf("    velocity_reached2=%d\n", spi_status.velocity_reached2);

                printf("XTARGET motor=%d xtarget=%d\n", *tmc5041_motors[i].motor, xtarget);

                int32_t xactual = tmc5041_get_register_XACTUAL(&tmc5041_motors[i]);
                printf("XACTUAL motor=%d xactual=%d\n", *tmc5041_motors[i].motor, xactual);

                int32_t vactual = tmc5041_get_velocity(&tmc5041_motors[i]);
                printf("VACTUAL motor=%d vactual=%d\n", *tmc5041_motors[i].motor, vactual);
                drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(&tmc5041_motors[i]);
                printf("DRV_STATUS motor=%d\n", *tmc5041_motors[i].motor);
                printf("    standstill=%d\n", drv_status.standstill);
                printf("    full_stepping=%d\n", drv_status.full_stepping);
                printf("    overtemp_warning=%d\n", drv_status.overtemp_warning);
                printf("    overtemp_alarm=%d\n", drv_status.overtemp_alarm);
                printf("    sg_result=%d\n", drv_status.sg_result);
                printf("    cs_actual=%d\n", drv_status.cs_actual);
                printf("    sg_status=%d\n", drv_status.sg_status);
                
                ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&tmc5041_motors[i]);
                printf("RAMP_STAT motor=%d\n", *tmc5041_motors[i].motor);
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

                int32_t ramp_mode = tmc5041_readInt(&tmc5041_motors[i], TMC5041_RAMPMODE(*tmc5041_motors[i].motor));
                printf("RAMPMODE motor=%d ramp_mode=%d\n", *tmc5041_motors[i].motor, ramp_mode);

                printf("\n");
            }

            printf("-------------------------------------------------------------------\n");
        }
    }

    printf("Shutting down\n");
    // tmc5041_end(tmc5041_motors, MOTOR_COUNT);
    
    // Free allocated memory
    for (size_t i = 0; i < MOTOR_COUNT; i++) {
        free((void*)tmc5041_motors[i].chip);
        free((void*)tmc5041_motors[i].motor);
        // The motor struct itself is stack-allocated in the array, no need to free it
    }
    
    rpi_end();

    return 0;
}

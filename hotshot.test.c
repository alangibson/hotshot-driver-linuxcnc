#include "stdio.h"
#include "signal.h"
#include "time.h"
#include "assert.h"
#include "tmc/helpers/Types.h"
#include "global.h"
#include "rpi.h"
#include "hotshot.hal.h"

// in us
// #define SERVO_PERIOD 1000
// like BASE_PERIOD
#define SERVO_PERIOD 100

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

void unitTests() {
    
    uint32_t pitch_cmd = 2;
    uint32_t teeth_cmd = 20;
    uint32_t microsteps_cmd = 256;
    uint32_t steps_per_rev = 200;
    uint32_t microstep_per_mm = microsteps_per_mm(
        steps_per_rev, pitch_cmd * teeth_cmd, microsteps_cmd);
    assert(microstep_per_mm == 1280);

    microstep_per_mm = 1280;
    float64_t mm = 100;
    int64_t mm_to_microstep = mm_to_microsteps(microstep_per_mm, mm);
    assert(mm_to_microstep == 128000);

    microstep_per_mm = 1280;
    mm = -100;
    mm_to_microstep = mm_to_microsteps(microstep_per_mm, mm);
    assert(mm_to_microstep == -128000);

    microstep_per_mm = 1280;
    mm = -100.33;
    mm_to_microstep = mm_to_microsteps(microstep_per_mm, mm);
    assert(mm_to_microstep == -128422); // float would be -128422.4

    microstep_per_mm = 1280;
    int32_t microsteps = 100;
    float64_t microstep_to_mm = microsteps_to_mm(microstep_per_mm, microsteps);
    assert(microstep_to_mm == 0.078125);

    microstep_per_mm = 1280;
    microsteps = -100;
    microstep_to_mm = microsteps_to_mm(microstep_per_mm, microsteps);
    assert(microstep_to_mm == -0.078125);
}

int main() {
    printf("Running unit tests\n");
    unitTests();
    printf("Unit tests passed\n");

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
    float64_t axis_x_position_cmd               = 0;
    float64_t axis_x_velocity_cmd               = 50.0;

    // Statically set in INI and then set via setp
    // FIXME not being used
    float64_t axis_x_max_velocity_cmd           = 400;
    float64_t axis_x_max_acceleration_cmd       = 100;
    // Set via setp
    uint32_t axis_x_pitch                       = 2;
    uint32_t axis_x_teeth                       = 20;
    uint32_t axis_x_microsteps_cmd              = 256;
    uint32_t axis_x_motor_fullsteps_per_rev_cmd = 200;
    // Set dynamically by HAL
    bool axis_x_enable                          = TRUE;
    bool axis_x_is_homing                       = FALSE;
    // Set via setp
    uint32_t axis_x_tmc_run_current_cmd         = 23;
    uint32_t axis_x_tmc_hold_current_cmd        = 6;
    bool axis_x_tmc_sg_stop_cmd                 = FALSE;
    uint32_t axis_x_tmc_cs_thresh_cmd           = 50000; // in ppt
    uint32_t axis_x_tmc_current_hold_delay_cmd  = 1;
    int32_t axis_x_tmc_chop_vhighchm_cmd = 0;;
    int32_t axis_x_tmc_chop_vhighfs_cmd = 0;
    int32_t axis_x_tmc_chop_vsense_cmd = 0;
    int32_t axis_x_tmc_chop_tbl_cmd             = 2;
    int32_t axis_x_tmc_chop_mode_cmd            = 0;
    int32_t axis_x_tmc_chop_hend_cmd            = 3;
    int32_t axis_x_tmc_chop_hstrt_cmd           = 5;
    int32_t axis_x_tmc_chop_toff_cmd            = 5;
    int32_t axis_x_tmc_chop_vhigh_cmd = 0;
    int32_t axis_x_tmc_coolstep_sfilt_cmd = 0;
    int32_t axis_x_tmc_sg_thresh_cmd = 0;
    int32_t axis_x_tmc_coolstep_seimin_cmd = 0;
    int32_t axis_x_tmc_coolstep_seup_cmd = 0;
    int32_t axis_x_tmc_coolstep_sedn_cmd = 0;
    int32_t axis_x_tmc_coolstep_semax_cmd = 0;
    int32_t axis_x_tmc_coolstep_semin_cmd = 0;
    int32_t axis_x_tmc_sw_en_softstop = 0;
    int32_t axis_x_tmc_ramp_vstart_cmd          = 0;
    int32_t axis_x_tmc_ramp_vstop_cmd           = 10;
    int32_t axis_x_tmc_ramp_v1_cmd = 0;
    int32_t axis_x_tmc_ramp_a1_cmd = 0;
    int32_t axis_x_tmc_ramp_dmax_cmd = 0;
    int32_t axis_x_tmc_ramp_d1_cmd = 0;
    int32_t axis_x_tmc_ramp_tzerowait_cmd = 0;
    int32_t axis_x_tmc_ramp_mode_cmd = 0;
    float64_t axis_x_velocity_factor = 0;
    // Set dynamically by Hotshot
    float64_t axis_x_position_fb;
    float64_t axis_x_velocity_fb;
    bool axis_x_home_sw;
    bool torch_breakaway;
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
        .max_velocity_cmd       = &axis_x_max_velocity_cmd,
        .max_acceleration_cmd   = &axis_x_max_acceleration_cmd,
        // Set via setp
        .pitch_cmd              = &axis_x_pitch,
        .teeth_cmd              = &axis_x_teeth,
        .microsteps_cmd         = &axis_x_microsteps_cmd,
        .motor_fullsteps_per_rev_cmd = &axis_x_motor_fullsteps_per_rev_cmd,
        // Set dynamically by HAL
        .velocity_cmd           = &axis_x_velocity_cmd,
        .enable_cmd         = &axis_x_enable,
        .homing_cmd          = &axis_x_is_homing,
        .position_cmd           = &axis_x_position_cmd,
        // Set dynamically by Hotshot
        .position_fb            = &axis_x_position_fb,
        .velocity_fb            = &axis_x_velocity_fb,
        .home_sw_fb             = &axis_x_home_sw,
        .torch_breakaway_fb     = &torch_breakaway,               
        .velocity_factor        = &axis_x_velocity_factor,
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
            .current_hold_delay_cmd = &axis_x_tmc_current_hold_delay_cmd,
            .chop_vhighchm_cmd      = &axis_x_tmc_chop_vhighchm_cmd,
            .chop_vhighfs_cmd       = &axis_x_tmc_chop_vhighfs_cmd,
            .chop_vsense_cmd        = &axis_x_tmc_chop_vsense_cmd,
            .chop_tbl_cmd           = &axis_x_tmc_chop_tbl_cmd,
            .chop_mode_cmd          = &axis_x_tmc_chop_mode_cmd,
            .chop_hend_cmd          = &axis_x_tmc_chop_hend_cmd,
            .chop_hstrt_cmd         = &axis_x_tmc_chop_hstrt_cmd,
            .chop_toff_cmd          = &axis_x_tmc_chop_toff_cmd,
            .chop_vhigh_cmd         = &axis_x_tmc_chop_vhigh_cmd,
            .coolstep_sfilt_cmd     = &axis_x_tmc_coolstep_sfilt_cmd,
            .sg_thresh_cmd          = &axis_x_tmc_sg_thresh_cmd,
            .coolstep_seimin_cmd    = &axis_x_tmc_coolstep_seimin_cmd,
            .coolstep_seup_cmd      = &axis_x_tmc_coolstep_seup_cmd,
            .coolstep_sedn_cmd      = &axis_x_tmc_coolstep_sedn_cmd,
            .coolstep_semax_cmd     = &axis_x_tmc_coolstep_semax_cmd,
            .coolstep_semin_cmd     = &axis_x_tmc_coolstep_semin_cmd,
            .sw_en_softstop         = &axis_x_tmc_sw_en_softstop,
            .ramp_vstart_cmd        = &axis_x_tmc_ramp_vstart_cmd,
            .ramp_vstop_cmd         = &axis_x_tmc_ramp_vstop_cmd,
            .ramp_v1_cmd            = &axis_x_tmc_ramp_v1_cmd,
            .ramp_a1_cmd            = &axis_x_tmc_ramp_a1_cmd,
            .ramp_dmax_cmd          = &axis_x_tmc_ramp_dmax_cmd,
            .ramp_d1_cmd            = &axis_x_tmc_ramp_d1_cmd,
            .ramp_tzerowait_cmd     = &axis_x_tmc_ramp_tzerowait_cmd,
            .ramp_mode_cmd          = &axis_x_tmc_ramp_mode_cmd,

            // Set dynamically by Hotshot
            // These also serve as a feedback 
            .position_fb                = &axis_x_tmc_position_fb,
            .velocity_fb                = &axis_x_tmc_velocity_fb,
            .motor_standstill_fb        = &axis_X_motor_standstill_fb,
            .motor_full_stepping_fb     = &axis_x_motor_full_stepping_fb,
            .motor_overtemp_warning_fb  = &axis_x_motor_overtemp_warning_fb,
            .motor_overtemp_alarm_fb    = &axis_x_motor_overtemp_alarm_fb,
            .motor_load_fb              = &axis_x_motor_load_fb,
            .motor_current_fb           = &axis_x_motor_current_fb,
            .motor_stall_fb             = &axis_x_motor_stall_fb,
            .microstep_resolution_fb    = &axis_x_microstep_resolution_fb,
        }
    };

    rpi_init();
    // hotshot_init(motors, MOTOR_COUNT);

    // This simulates the HAL loop
    while (!ctrlCPressed) {
        uint32_t tick = clock();

        // Move all motors
        if (tick - last_move >= SERVO_PERIOD) {
            last_move = tick;

            // xtarget += 10; // in mm
            *motors[0].enable_cmd = TRUE;
            *motors[0].velocity_cmd = 25.0;

            hotshot_handle_joints(motors, MOTOR_COUNT);
        }

        /*
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

                printf("PINS\n");
                printf("    position_fb=%d\n", *motors[i].position_fb);
                printf("    tmc_position_fb=%d\n", *motors[i].tmc.position_fb);

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
        */
    }

    printf("Shutting down\n");
    hotshot_end(motors, MOTOR_COUNT);

    return 0;
}

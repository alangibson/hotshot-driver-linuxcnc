// Glue code tying Hotshot to LinuxCNC montion controller

#include "stdio.h"
#include "stdbool.h"
#include "global.h"
#include "bcm2835.h"
#include "rpi.h"
#include "hotshot.hal.h"

void handle_joint(joint_t * motor) {

    if (*motor->is_enabled) {

        #ifdef DEBUG
        rtapi_print("joint x is enabled. chip=%d motor=%d pitch=%d teeth=%d\n", 
            motor->chip, motor->motor, motor->pitch, motor->teeth);
        #endif

        rpi_spi_select(motor->chip);


        // Check switches
        //
        // Note: Reading RAMP_STAT clears sg_stop
        ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&motor->tmc);
        // Note: status_sg and event_stop_sg appear to be the same
        *motor->sg_stop_fb = ramp_stat.event_stop_sg;
        // *motor->sg_stop_fb = ramp_stat.status_sg;
        // TODO
        // axis_x_tmc_position_reached_fb = ramp_stat.position_reached;
        // axis_x_tmc_t_zerowait_active_fb = ramp_stat.t_zerowait_active;
        // axis_x_tmc_velocity_reached_fb = ramp_stat.velocity_reached;
        // printf("status_sg=%d\n", *motor->sg_stop_fb);

        #ifdef DEBUG
        rtapi_print("ramp_stat axis_x_sg_stop_fb %d \n", (*motor->sg_stop_fb));
        #endif

        // Handle StallGuard state
        //
        if (*motor->sg_stop_fb) {
            // StallGuard has been triggered

            #ifdef DEBUG
            rtapi_print("SG triggered sg_stop_cmd=%d sg_thresh_cmd=%d\n", 
                motor->tmc_sg_stop_cmd, motor->tmc_sg_thresh_cmd);
            #endif

            if (motor->is_homing) {
                // We are homing and we have hit a Stallguard stop event, so trigger home switch

                #ifdef DEBUG
                rtapi_print("Setting home\n");
                #endif

                motor->home_sw = tmc5041_motor_set_home(&motor->tmc);
            } else {
                // We've hit a Stallguard stop, but we're not homing

                // TODO shut off torch
                
                // Stop motion controller or motor will twitch every time we read RAMP_STAT
                tmc5041_motor_halt(&motor->tmc);

                *motor->torch_breakaway_fb = TRUE;

                // TODO "breakaway" for the x axis should indicate negative limit switch is triggered
                // There is currently no negative limit switch for X and YL/YR
            }             
        } else {
            // No active StallGuard switches, so move motor and reset all switches

            #ifdef DEBUG
            rtapi_print("Moving motor to %f\n", (*motor->position_cmd));
            #endif

            // Move motor
            //
            spi_status_t spi_status = follow(motor);

            #ifdef DEBUG
            rtapi_print("Followed position to %f\n", spi_status.xactual_mm);
            #endif

            // FIXME
            // *motor->position_fb = spi_status.xactual_mm;

            #ifdef DEBUG
            rtapi_print("position_fb is %f\n", *motor->position_fb);
            #endif

            *motor->position_fb = tmc5041_get_register_XACTUAL(&motor->tmc);

            #ifdef DEBUG
            rtapi_print("Motor actual position now %f\n", *motor->position_fb);
            #endif

            // Ensure switches aren't triggered
            //
            motor->home_sw = FALSE;
            // FIXME
            *motor->torch_breakaway_fb = FALSE;

            #ifdef DEBUG
            rtapi_print("Finished resetting switches\n");
            #endif
        }

        #ifdef DEBUG
        rtapi_print("Get velocity\n");
        #endif

        // Get velocity
        //
        *motor->tmc.velocity_fb = tmc5041_get_register_VACTUAL(&motor->tmc);
        float vactual_mm = (float)(*motor->tmc.velocity_fb) / (float)(*motor->microstep_per_mm);
        *motor->velocity_fb = vactual_mm;
       
        #ifdef DEBUG
        rtapi_print("vactual_mm is %f\n", vactual_mm);
        rtapi_print("Get driver state\n");
        #endif

        // Get driver state
        // 
        drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(&motor->tmc);
        *motor->tmc.motor_standstill_fb = drv_status.standstill;
        *motor->tmc.motor_full_stepping_fb = drv_status.full_stepping;
        *motor->tmc.motor_overtemp_warning_fb = drv_status.overtemp_warning;
        *motor->tmc.motor_overtemp_alarm_fb = drv_status.overtemp_alarm;
        *motor->tmc.motor_load_fb = drv_status.sg_result;
        *motor->tmc.motor_current_fb = drv_status.cs_actual;
        *motor->tmc.motor_stall_fb = drv_status.sg_status;

        #ifdef DEBUG
        rtapi_print("vactual_mm is %f\n", vactual_mm);
        rtapi_print("Get chopper state\n");
        #endif

        // Get chopper state
        //
        chopconf_register_t chopconf = tmc5041_get_register_CHOPCONF(&motor->tmc);
        *motor->tmc.microstep_resolution_fb = chopconf.mres;

        #ifdef DEBUG
        rtapi_print("Done handling joint\n");
        #endif

        rpi_spi_unselect();
    } else {
        // Joint is not enabled in LinuxCNC

        // Cut power to motor
        // Can only do this when we config_motor inside motor->is_enabled condition
        // TODO reset_motor(&motors[0]);
        motor->is_on = FALSE;

        // Clear torch breakaway
        // TODO we should really do this when we transition from unenabled to enabled power
        *motor->torch_breakaway_fb = FALSE;
    }
}

void handle_joints(joint_t * motors, uint8_t motor_count) {
    // Do something with each joint
    for (uint8_t i = 0; i < motor_count; i++) {
        handle_joint(&motors[i]);
    }
}

bool hotshot_init(joint_t * motors, uint8_t motor_count)
{
    rpi_init();
    // tmc5041_init(motors, MOTOR_COUNT);
    for (uint8_t i = 0; i < motor_count; i++) {
        tmc5041_motor_init(&motors[i].tmc);
    }
    return FALSE;
}

void hotshot_end(joint_t * motors, uint8_t motor_count)
{
    // tmc5041_end(motors, MOTOR_COUNT);
    for (uint8_t i = 0; i < motor_count; i++) {
        tmc5041_motor_end(&motors[i].tmc);
    }
    rpi_end();
}

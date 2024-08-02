// Glue code tying Hotshot to LinuxCNC montion controller

#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "global.h"
#include "bcm2835.h"
#include "rpi.h"
#include "hotshot.hal.h"

bool hotshot_joint_init(joint_t * joint)
{
    #ifdef DEBUG
    printf("hotshot_joint_init: pitch=%d teeth=%d\n", *joint->pitch_cmd, *joint->teeth_cmd);
    printf("hotshot_joint_init: microsteps_cmd=%d\n", *joint->microsteps_cmd);
    #endif

    uint32_t microstep_per_mm = microsteps_per_mm(
        *joint->motor_fullsteps_per_rev_cmd, 
        (*joint->pitch_cmd) * (*joint->teeth_cmd), 
        *joint->microsteps_cmd);

    joint->microstep_per_mm = microstep_per_mm;

    #ifdef DEBUG
    printf("hotshot_joint_init: microstep_per_mm=%d\n", microstep_per_mm);
    printf("hotshot_joint_init: joint->microstep_per_mm=%d\n", joint->microstep_per_mm );
    #endif

    uint32_t tmc_max_acceleration_cmd = mm_to_microsteps(microstep_per_mm, *joint->max_acceleration_cmd);

    #ifdef DEBUG
    printf("hotshot_joint_init: tmc_max_acceleration_cmd=%d\n", tmc_max_acceleration_cmd);
    #endif

    joint->tmc.max_acceleration_cmd = tmc_max_acceleration_cmd;

    #ifdef DEBUG
    printf("hotshot_joint_init: joint->tmc.max_acceleration_cmd=%d\n", joint->tmc.max_acceleration_cmd);
    #endif

    // uint32_t a1 = tmc_max_acceleration_cmd * 2;
    // *joint->tmc.ramp_a1_cmd = a1;
    // printf("calling motor init 1\n");
    // *joint->tmc.ramp_dmax_cmd = tmc_max_acceleration_cmd;
    // printf("calling motor init 2\n");
    // joint->tmc.ramp_d1_cmd = joint->tmc.ramp_a1_cmd;

    #ifdef DEBUG
    printf("calling motor init\n");
    #endif

    tmc5041_motor_init(&joint->tmc);
}

void hotshot_init(joint_t * joints, uint8_t motor_count)
{
    rpi_init();
    // for (uint8_t i = 0; i < motor_count; i++) {
    //     hotshot_joint_init(&joints[i]);
    // }
}

void handle_joint(joint_t * joint)
{
    rpi_spi_select(joint->tmc.chip.chip);

    if (*joint->is_enabled_cmd)
    {
        if (! joint->is_setup)
        {
            hotshot_joint_init(joint);
            joint->is_setup = TRUE;
        }
        // else
        // {
        //     // Always call this so we can configure drivers on the fly
        //     tmc5041_motor_set_config_registers(&joint->tmc);
        // }

        if (! joint->tmc.is_power_on)
        {
            joint->tmc.position_cmd = mm_to_microsteps(joint->microstep_per_mm, *joint->position_cmd);
            tmc5041_motor_power_on(&joint->tmc);
        }

        // Check switches
        //
        // Note: Reading RAMP_STAT clears sg_stop
        // FIXME if we read RAMP_STAT register, then Stallguard never stops motor 
        //       since reading this register clears the stop flag.
        //       We need a different way to read Stallguard status; 
        //       maybe use *joint->tmc.motor_stall_fb instead?
        // ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&joint->tmc);
        // *joint->sg_stop_fb = ramp_stat.event_stop_sg;

        // TODO
        // axis_x_tmc_position_reached_fb = ramp_stat.position_reached;
        // axis_x_tmc_t_zerowait_active_fb = ramp_stat.t_zerowait_active;
        // axis_x_tmc_velocity_reached_fb = ramp_stat.velocity_reached;

        // Handle StallGuard state
        //
        if (*joint->sg_stop_fb)
        {
            // StallGuard has been triggered
            printf("Stallguard triggered\n");

            #ifdef DEBUG
            printf("handle_joint(%d, %d): StallGuard triggered sg_stop_cmd=%d sg_thresh_cmd=%d\n", 
                joint->tmc.chip, joint->tmc.motor, *joint->tmc.sg_stop_cmd, *joint->tmc.sg_thresh_cmd);
            printf("handle_joint(%d, %d): Disable axis\n",
                joint->tmc.chip, joint->tmc.motor);
            #endif

            if (*joint->is_homing_cmd) {
                // We are homing and we have hit a Stallguard stop event, so trigger home switch

                printf("handle_joint(%d, %d): Is homing\n",
                    joint->tmc.chip, joint->tmc.motor);

                #ifdef DEBUG
                printf("handle_joint(%d, %d): Setting home position\n",
                    joint->tmc.chip, joint->tmc.motor);
                #endif

                joint->home_sw = tmc5041_motor_set_home(&joint->tmc);
            } else {
                // We've hit a Stallguard stop, but we're not homing.
                //
                // TODO if torch is on, then trigger torch breakaway
                // TODO if torch is not on, then assume we have hit an axis limit

                printf("handle_joint(%d, %d): Is NOT homing\n",
                    joint->tmc.chip, joint->tmc.motor);

                // TODO Don't power off, go to holding current instead
                // tmc5041_motor_power_off(&joint->tmc);

                // #ifdef DEBUG
                // printf("handle_joint(%d, %d): Triggering torch breakaway\n",
                //     joint->tmc.chip, joint->tmc.motor);
                // #endif

                // TODO shut off torch
                
                // Stop motion controller or joint will twitch every time we read RAMP_STAT
                // TODO we should probably just set XTARGET=XACTUAL and leave chopper on
                // tmc5041_motor_power_off(&joint->tmc);

                // *joint->torch_breakaway_fb = TRUE;

                // TODO "breakaway" for the x axis should indicate negative limit switch is triggered
                // There is currently no negative limit switch for X and YL/YR
            }             
        } else
        {
            // No active StallGuard switches, so move joint and reset all switches

            // Move joint
            move(joint);
            // Update pins
            update(joint);

            // Ensure switches aren't triggered
            joint->home_sw = FALSE;
            *joint->torch_breakaway_fb = FALSE;
        }

        // Get driver state
        // 
        drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(&joint->tmc);
        *joint->tmc.motor_standstill_fb = drv_status.standstill;
        *joint->tmc.motor_full_stepping_fb = drv_status.full_stepping;
        *joint->tmc.motor_overtemp_warning_fb = drv_status.overtemp_warning;
        *joint->tmc.motor_overtemp_alarm_fb = drv_status.overtemp_alarm;
        *joint->tmc.motor_load_fb = drv_status.sg_result;
        *joint->tmc.motor_current_fb = drv_status.cs_actual;
        *joint->tmc.motor_stall_fb = drv_status.sg_status;

        // Get chopper state
        //
        // chopconf_register_t chopconf = tmc5041_get_register_CHOPCONF(&joint->tmc);
        // *joint->tmc.microstep_resolution_fb = chopconf.mres;

    } else {
        // Joint is not enabled in LinuxCNC

        // joint->is_setup = FALSE;

        // Cut power to joint
        tmc5041_motor_power_off(&joint->tmc);
    }

    rpi_spi_unselect();
}

void handle_joints(joint_t * joint, uint8_t motor_count) {
    // Do something with each joint
    for (uint8_t i = 0; i < motor_count; i++) {
        handle_joint(&joint[i]);
    }
}

void hotshot_end(joint_t * joint, uint8_t motor_count)
{
    // tmc5041_end(joint, MOTOR_COUNT);
    for (uint8_t i = 0; i < motor_count; i++) {
        tmc5041_motor_end(&joint[i].tmc);
    }
    rpi_end();
}

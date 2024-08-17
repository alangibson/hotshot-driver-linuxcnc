// Glue code tying Hotshot to LinuxCNC montion controller.
// hotshot.comp should only call functions in this file.

#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "hal.h"
#include "global.h"
#include "bcm2835.h"
#include "rpi.h"
#include "hotshot.hal.h"

/**
 * Initialize Hotshot board and driver.
 */
void hotshot_init()
{
    rpi_init();
}

/**
 * Initialize a single joint. 
 * Right now this only configures the motor, not switches.
 */
bool hotshot_joint_init(joint_t * joint)
{
    #ifdef DEBUG
    rtapi_print("hotshot_joint_init: pitch=%d teeth=%d\n", *joint->pitch_cmd, *joint->teeth_cmd);
    rtapi_print("hotshot_joint_init: microsteps_cmd=%d\n", *joint->microsteps_cmd);
    #endif

    uint32_t microstep_per_mm = microsteps_per_mm(
        *joint->motor_fullsteps_per_rev_cmd, 
        (*joint->pitch_cmd) * (*joint->teeth_cmd), 
        *joint->microsteps_cmd);

    joint->microstep_per_mm = microstep_per_mm;

    #ifdef DEBUG
    rtapi_print("hotshot_joint_init: microstep_per_mm=%d\n", microstep_per_mm);
    rtapi_print("hotshot_joint_init: joint->microstep_per_mm=%d\n", joint->microstep_per_mm );
    #endif

    uint32_t tmc_max_acceleration_cmd = mm_to_microsteps(microstep_per_mm, *joint->max_acceleration_cmd);

    #ifdef DEBUG
    rtapi_print("hotshot_joint_init: tmc_max_acceleration_cmd=%d\n", tmc_max_acceleration_cmd);
    #endif

    joint->tmc.max_acceleration_cmd = tmc_max_acceleration_cmd;

    #ifdef DEBUG
    rtapi_print("hotshot_joint_init: joint->tmc.max_acceleration_cmd=%d\n", joint->tmc.max_acceleration_cmd);
    #endif

    // uint32_t a1 = tmc_max_acceleration_cmd * 2;
    // *joint->tmc.ramp_a1_cmd = a1;
    // rtapi_print("calling motor init 1\n");
    // *joint->tmc.ramp_dmax_cmd = tmc_max_acceleration_cmd;
    // rtapi_print("calling motor init 2\n");
    // joint->tmc.ramp_d1_cmd = joint->tmc.ramp_a1_cmd;

    #ifdef DEBUG
    rtapi_print("calling motor init\n");
    #endif

    tmc5041_motor_init(&joint->tmc);

    return TRUE; // successful
}

void hotshot_handle_homing(joint_t * joint)
{
    // Return immediately if we are not homing.
    if (*joint->homing_cmd != 1) {
        // Switch can never be triggered on if we are not homing
        *joint->home_sw_fb = 0;
        joint->home_sw_debounce = 0;
        return;
    }

    // This happens for us on every thread iteration
    // tmc5041_pull_register_DRV_STATUS(&joint->tmc);

    // TODO get this via pin
    int32_t SG_TRIGGER_THRESH = 100;

    int32_t sg_load = *joint->tmc.motor_load_fb;
    int32_t vactual = tmc5041_get_register_VACTUAL(&joint->tmc);
    int32_t velocity_cmd = *joint->tmc.velocity_cmd;
    // TODO Maybe when velocity_cmd and velocity_fb are within some percent of each other?
    int32_t velocity_diff = velocity_cmd - *joint->tmc.velocity_fb;
    int32_t threshold_diff = abs(vactual) - *joint->tmc.cs_thresh_cmd;

    if (*joint->home_sw_fb == 1
        && ( sg_load > SG_TRIGGER_THRESH || velocity_cmd == 0 ) )
    {
        // Reset switch

        if (joint->home_sw_debounce > 5)
        {
            // Debounce threshold crossed. Switch off.
            // rtapi_print("Switch triggered off\n");

            *joint->home_sw_fb = 0;
            joint->home_sw_debounce = 0;
        }
        else
        {
            // Increment debounce counter
            joint->home_sw_debounce += 1;
        }
    }
    else if (velocity_diff == 0 && sg_load == 0)
    {
        // If everything is 0, do nothing. This just shouldn't be possible, but it happens.
        // No clue why this happens.
    }
    else if (
        *joint->home_sw_fb == 0
        && sg_load <= SG_TRIGGER_THRESH 
        && velocity_diff == 0 
        && threshold_diff > 0
        && *joint->tmc.velocity_fb != 0 )
    {
        // rtapi_print("Home threshold met. %d < %d. velocity_fb=%d, velocity_diff=%d. threshold_diff=%d. switch=%d. debounce=%d\n", 
        //     sg_load, SG_TRIGGER_THRESH, *joint->tmc.velocity_fb, velocity_diff, threshold_diff,
        //     *joint->home_sw_fb, joint->home_sw_debounce);

        if (joint->home_sw_debounce > 5)
        {
            // Debounce threshold crossed. Switch on.
            // rtapi_print("Switch triggered on\n");

            *joint->home_sw_fb = 1;
            joint->home_sw_debounce = 0;
        }
        else
        {
            // Increment debounce counter
            joint->home_sw_debounce += 1;
        }
    }
    else
    {
        // Just reset debounce counter

        // rtapi_print("Debounce reset. %d < %d. velocity_cmd=%d, velocity_fb=%d. threshold_diff=%d. switch=%d\n", 
        //     sg_load, SG_TRIGGER_THRESH, *joint->tmc.velocity_cmd, *joint->tmc.velocity_fb, threshold_diff, *joint->home_sw_fb);
        
        joint->home_sw_debounce = 0;
    }

    // Get motor's current status
    // tmc5041_pull_register_DRV_STATUS(&joint->tmc);
    // rtapi_print("hotshot(%d, %d): DRV_STATUS status_sg=%d, motor_load_fb=%d, motor_stall_fb=%d\n",
    //     joint->tmc.chip.chip, joint->tmc.motor,
    //     ramp_stat.status_sg,
    //     *motor->motor_load_fb,
    //     *motor->motor_stall_fb);

    // tmc5041_pull_register_RAMP_STAT(&joint->tmc);
    // rtapi_print("hotshot(%d, %d): RAMP_STAT status_sg=%d, position_reached=%d, velocity_reached=%d, event_pos_reached=%d, event_stop_sg=%d, event_stop_r=%d, event_stop_l=%d, status_latch_r=%d, status_latch_l=%d, status_stop_r=%d, status_stop_l=%d\n", 
    //     joint->tmc.chip.chip, joint->tmc.motor,
    //     *joint->tmc.status_sg_fb,
    //     *joint->tmc.position_reached_fb,
    //     *joint->tmc.velocity_reached_fb,
    //     *joint->tmc.event_pos_reached_fb,
    //     *joint->tmc.event_stop_sg_fb,
    //     *joint->tmc.event_stop_r_fb,
    //     *joint->tmc.event_stop_l_fb,
    //     *joint->tmc.status_latch_r_fb,
    //     *joint->tmc.status_latch_l_fb,
    //     *joint->tmc.status_stop_r_fb,
    //     *joint->tmc.status_stop_l_fb
    // );
}

void hotshot_handle_stall(joint_t * joint)
{
    // // TODO get velocity_reached from spi_status
    tmc5041_pull_register_DRV_STATUS(&joint->tmc);
    int32_t vactual = tmc5041_get_register_VACTUAL(&joint->tmc);

    // FIXME detect a stallguard stop, not just a stall!
    // Maybe check if VACTUAL == 0?
    if (*joint->tmc.motor_stall_fb == 1 && vactual == 0)
    {
        // Avoid joint following errors in LinuxCNC by immediately notifying
        // it that we have stopped the motor.
        // *joint->torch_breakaway_fb = TRUE;

        // tmc5041_motor_position_hold(&joint->tmc);
        tmc5041_motor_clear_stall(&joint->tmc);

    //     // if (*joint->homing_cmd) {

    //     //     #ifdef DEBUG
    //     //     rtapi_print("hotshot_handle_move(%d, %d): Is homing\n",
    //     //         joint->tmc.chip, joint->tmc.motor);
    //     //     rtapi_print("hotshot_handle_move(%d, %d): Setting home position\n",
    //     //         joint->tmc.chip, joint->tmc.motor);
    //     //     #endif

    //     //     // We are homing and we have hit a Stallguard stop event, 
    //     //     // so trigger home switch
    //     //     joint->home_sw = tmc5041_motor_set_home(&joint->tmc);

    //     // } else {
    //     //     // We've hit a Stallguard stop, but we're not homing.

    //     //     #ifdef DEBUG
    //     //     rtapi_print("hotshot_handle_move(%d, %d): Is NOT homing\n",
    //     //         joint->tmc.chip, joint->tmc.motor);
    //     //     rtapi_print("hotshot_handle_move(%d, %d): Triggering torch breakaway\n",
    //     //         joint->tmc.chip, joint->tmc.motor);
    //     //     #endif

    //     //     // Set torch breakaway pin and let LinuxCNC handle the rest
    //     //     *joint->torch_breakaway_fb = TRUE;
    //     // }             

    //     tmc5041_motor_clear_stall(&joint->tmc);

    }
    else
    {
        // Note: don't clear breakaway here or the signal will bounce
        // since not all motors will stall at the same time.
        // *joint->torch_breakaway_fb = FALSE;
    }
}

void hotshot_handle_move(joint_t * joint)
{
    if (*joint->enable_cmd == 1) // Power is on in LinuxCNC
    {
        // // Run joint setup exactly once
        if (! joint->is_setup)
        {
            joint->is_setup = hotshot_joint_init(joint);
        }
        else
        {
        //     // TODO update any registers that are always safe to update

        //     // Always call this so we can configure drivers on the fly
        //     // tmc5041_motor_set_config_registers(&joint->tmc);

        //     // These inputs should always be up to date
            tmc5041_pull_register_DRV_STATUS(&joint->tmc);
        //     // These outputs should always be up to date
        //     // tmc5041_push_register_COOLCONF(&joint->tmc);
        }

        // TODO toggle hold mode instead of turning driver power on and off
        // if (! joint->tmc.is_motor_on) // TMC5041 chopper is on
        // {
        //     joint->tmc.position_cmd = mm_to_microsteps(joint->microstep_per_mm, *joint->position_cmd);
        //     tmc5041_motor_power_on(&joint->tmc);
        // }

        // Check switches
        //
        // Note: Reading RAMP_STAT clears sg_stop
        // FIXME if we read RAMP_STAT register, then Stallguard never stops motor 
        //       since reading this register clears the stop flag.
        //       We need a different way to read Stallguard status; 
        //       maybe use *joint->tmc.motor_stall_fb instead?
        // ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&joint->tmc);
        //
        // TODO
        // axis_x_tmc_position_reached_fb = ramp_stat.position_reached;
        // axis_x_tmc_t_zerowait_active_fb = ramp_stat.t_zerowait_active;
        // axis_x_tmc_velocity_reached_fb = ramp_stat.velocity_reached;
        //
        // Get driver state
        // 
        // drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(&joint->tmc);
        // *joint->tmc.motor_standstill_fb = drv_status.standstill;
        // *joint->tmc.motor_full_stepping_fb = drv_status.full_stepping;
        // *joint->tmc.motor_overtemp_warning_fb = drv_status.overtemp_warning;
        // *joint->tmc.motor_overtemp_alarm_fb = drv_status.overtemp_alarm;
        // *joint->tmc.motor_load_fb = drv_status.sg_result;
        // *joint->tmc.motor_current_fb = drv_status.cs_actual;
        // *joint->tmc.motor_stall_fb = drv_status.sg_status;
        // tmc5041_pull_register_DRV_STATUS(&joint->tmc);
        //
        // tmc5041_pull_register_RAMP_STAT(&joint->tmc);
        // tmc5041_motor_clear_stall(&joint->tmc);

        // Handle StallGuard state
        //
        // if (*joint->tmc.motor_stall_fb) // StallGuard has been triggered
        // {
        //     #ifdef DEBUG
        //     rtapi_print("Stallguard triggered\n");
        //     rtapi_print("hotshot_handle_move(%d, %d): StallGuard triggered sg_stop_cmd=%d sg_thresh_cmd=%d\n", 
        //         joint->tmc.chip, joint->tmc.motor, *joint->tmc.sg_stop_cmd, *joint->tmc.sg_thresh_cmd);
        //     rtapi_print("hotshot_handle_move(%d, %d): Disable axis\n",
        //         joint->tmc.chip, joint->tmc.motor);
        //     #endif

        //     // Go into "hold position" mode so motor doesn't twitch
        //     // tmc5041_motor_position_hold(&joint->tmc);
        //     // tmc5041_motor_clear_stall(&joint->tmc);

        //     if (*joint->homing_cmd) {

        //         #ifdef DEBUG
        //         rtapi_print("hotshot_handle_move(%d, %d): Is homing\n",
        //             joint->tmc.chip, joint->tmc.motor);
        //         rtapi_print("hotshot_handle_move(%d, %d): Setting home position\n",
        //             joint->tmc.chip, joint->tmc.motor);
        //         #endif

        //         // We are homing and we have hit a Stallguard stop event, 
        //         // so trigger home switch
        //         joint->home_sw = tmc5041_motor_set_home(&joint->tmc);

        //     } else {
        //         // We've hit a Stallguard stop, but we're not homing.

        //         #ifdef DEBUG
        //         rtapi_print("hotshot_handle_move(%d, %d): Is NOT homing\n",
        //             joint->tmc.chip, joint->tmc.motor);
        //         rtapi_print("hotshot_handle_move(%d, %d): Triggering torch breakaway\n",
        //             joint->tmc.chip, joint->tmc.motor);
        //         #endif

        //         // Set torch breakaway pin and let LinuxCNC handle the rest
        //         *joint->torch_breakaway_fb = TRUE;
        //     }             
        // } 
        // else // StallGuard NOT triggered
        // {
        //     // No active StallGuard switches, so move joint and reset all switches
        //     //
        //     // Move joint
        //     move(joint);
        //     // Update pins
        //     update(joint);

        //     // Ensure switches aren't triggered because there's no other way to clear them
        //     // joint->home_sw = FALSE;
        //     *joint->torch_breakaway_fb = FALSE;
        // }

        // Move joint
        move(joint);
        // Update pins
        update(joint);

    } else {
        // Joint is not enabled in LinuxCNC.
        // This can happen automatically when torch breakaway is fired.

        // Cut power to joint
        // tmc5041_motor_power_off(&joint->tmc);

        tmc5041_motor_position_hold(&joint->tmc);

        // Clear stallguard with by reading RAMP_STAT register
        // ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&joint->tmc);
        
        // TODO chancel any moves
        // VACTUAL = 0
        // XTARGET = XACTUAL
    }
}

void hotshot_handle_joints(joint_t * joints, uint8_t motor_count) {
    // Do something with each joints
    for (uint8_t i = 0; i < motor_count; i++) {

        rpi_spi_select(joints[i].tmc.chip.chip); 

        // if (! joints[i].is_setup)
        // {
        //     joints[i].is_setup = hotshot_joint_init(&joints[i]);
        // }
        // // These should always be up to date
        // tmc5041_pull_register_DRV_STATUS(&joints[i].tmc);

        // Run joint setup exactly once
        // if (! joints[i].is_setup)
        // {
        //     joints[i].is_setup = hotshot_joint_init(&joints[i]);
        // }
        // else
        // {
        //     tmc5041_pull_register_DRV_STATUS(&joints[i].tmc);

        //     hotshot_handle_move(&joints[i]);
        //     hotshot_handle_homing(&joints[i]);

        // }

        hotshot_handle_move(&joints[i]);
        hotshot_handle_homing(&joints[i]);

        rpi_spi_unselect();
    }
}

/**
 * Shut down Hotshot board and driver.
 * Should only be called once at shutdown.
 */
void hotshot_end(joint_t * joint, uint8_t motor_count)
{
    // tmc5041_end(joint, MOTOR_COUNT);
    for (uint8_t i = 0; i < motor_count; i++) {
        tmc5041_motor_end(&joint[i].tmc);
    }
    rpi_end();
}

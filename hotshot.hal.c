// Glue code tying Hotshot to LinuxCNC montion controller.
// hotshot.comp should only call functions in this file.

#include "stdio.h"
#include "stdbool.h"
#include "math.h"
#include "hal.h"
#include "global.h"
#include "bcm2835.h"
#include "rpi.h"
#include "hotshot.h"
#include "hotshot.lib.h"
#include "hotshot.hal.h"

// #define DEBUG_HOMING 1

#define HOME_UNKNOWN 0
#define HOME_SEARCHING 1
#define HOME_BACKING 2
#define HOME_LATCHING 3
#define HOME_HOMED 4

/**
 * Initialize Hotshot board and driver.
 */
void hotshot_init(joint_t * joints, uint8_t motor_count)
{
    // Init the Raspberry Pi
    printf("hotshot: Initializing Raspberry Pi driver\n");
    rpi_init();
    printf("hotshot: Initializing Raspberry Pi driver complete\n");

    // Init each TMC5041 motor
    for (uint8_t i = 0; i < motor_count; i++)
    {
        rpi_spi_select(*joints[i].tmc.chip); 
        hotshot_joint_init(&joints[i]);
        rpi_spi_unselect();
    }
}

/**
 * Initialize a single joint. 
 * Right now this only configures the motor, not switches.
 */
bool hotshot_joint_init(joint_t * joint)
{
    // Set microstepping
    joint->tmc.mres = tmc5041_microsteps_to_mres(*joint->microsteps_cmd);

    joint->unit_pulse_factor = units_per_pulse(
        (float64_t)units_per_rev((*joint->pitch_cmd), (*joint->teeth_cmd)), 
        (float64_t)pulses_per_rev((*joint->motor_fullsteps_per_rev_cmd), (*joint->microsteps_cmd))
    );
    joint->tmc.max_acceleration_cmd = units_to_pulses(*joint->max_acceleration_cmd, joint->unit_pulse_factor);

    tmc5041_motor_init(&joint->tmc);

    return TRUE; // successful
}

void hotshot_handle_homing(joint_t * joint)
{
    // if (*joint->tmc.chip == 1 && *joint->tmc.motor == 0)
    //     rtapi_print("hotshot(%d,%d): homing_cmd=%d, home_state=%d\n", 
    //         *joint->tmc.chip,  *joint->tmc.motor, 
    //         *joint->homing_cmd, *joint->homing_state_fb);

    // Reset and return immediately if we are not homing.
    if (*joint->homing_cmd != 1) {
        // Switch can never be triggered on if we are not homing
        *joint->home_sw_fb = 0;
        joint->home_sw_debounce = 0;
        // Reset state so we can home again
        *joint->homing_state_fb = HOME_UNKNOWN;
        return;
    }

    // if (*joint->tmc.chip == 1 && *joint->tmc.motor == 0)
    //     rtapi_print("hotshot(%d,%d): home_state=%d\n", 
    //         *joint->tmc.chip,  *joint->tmc.motor, *joint->homing_state_fb);

    // Calculate everything we need to know in order to determine where
    // we are in the homing sequence.
    //
    int32_t sg_trigger_thresh = *joint->tmc.sg_trigger_thresh_cmd;
    int32_t sg_load = *joint->tmc.motor_load_fb;
    // int32_t vactual = tmc5041_get_velocity(&joint->tmc);
    int32_t vactual = *joint->tmc.velocity_fb;
    int32_t threshold_diff = abs(vactual) - *joint->tmc.cs_thresh_cmd;
    int32_t stall_diff = sg_load - sg_trigger_thresh;
    // TODO compare current velocity against HOME_SEARCH_VEL and HOME_LATCH_VEL ?
    // HACK force velocity_diff to 0 since, when using pid, it is always some small value
    // TODO Maybe when velocity_cmd and velocity_fb are within some percent of each other?
    // int32_t velocity_cmd = *joint->tmc.velocity_cmd;
    // int32_t velocity_diff = velocity_cmd - *joint->tmc.velocity_fb;
    int32_t velocity_diff = 0;

    // Enter SEARCHING state?
    if (*joint->homing_state_fb < HOME_SEARCHING)
    {
        // homing_cmd is true, so we are searching at a minimum.
        #ifdef DEBUG_HOMING
        rtapi_print("hotshot(%d,%d): State->HOME_SEARCHING. %d -> %d\n", 
            *joint->tmc.chip, *joint->tmc.motor, *joint->homing_state_fb, HOME_SEARCHING);
        #endif

        *joint->homing_state_fb = HOME_SEARCHING;
    }

    if  // Enter BACKING state?
    (
        *joint->homing_state_fb == HOME_SEARCHING
        && *joint->home_sw_fb == 0
        && stall_diff <= 0
        && velocity_diff == 0 
        && threshold_diff > 0
        && *joint->tmc.velocity_fb != 0
    )
    {
        // Switch has been triggered while in searching phase

        // Indicate switch on
        if (joint->home_sw_debounce > HOME_SEARCHING_DEBOUNCE)
        {
            #ifdef DEBUG_HOMING
            rtapi_print("hotshot(%d,%d): State->HOME_BACKING. %d -> %d\n", 
                *joint->tmc.chip, *joint->tmc.motor, *joint->homing_state_fb, HOME_BACKING);
            #endif

            // Debounce threshold crossed. Switch on.
            *joint->home_sw_fb = 1;
            joint->home_sw_debounce = 0;

            // Searching phase is complete. Go to next state.
            *joint->homing_state_fb = HOME_BACKING;
           
        }
        else
        {
            // Increment debounce counter
            joint->home_sw_debounce += 1;
        }
    }
    else if // Enter LATCHING state?
    (
        *joint->homing_state_fb == HOME_BACKING
        && velocity_diff == 0 
        && *joint->tmc.velocity_fb != 0
    )
    {
        // Backing phase is complete, so turn switch off

        // Indicate switch off after a period of distance
        // TODO we need to move far enough back that we can reach VMAX
        if (joint->home_sw_debounce > HOME_BACKING_DEBOUNCE)
        {
            #ifdef DEBUG_HOMING
            rtapi_print("hotshot(%d,%d): State->HOME_LATCHING. %d -> %d\n", 
                *joint->tmc.chip, *joint->tmc.motor, *joint->homing_state_fb, HOME_LATCHING);
            #endif

            // Debounce threshold crossed. Switch on.
            *joint->home_sw_fb = 0;
            joint->home_sw_debounce = 0;

            // Backing phase is complete. Go to next state.
            *joint->homing_state_fb = HOME_LATCHING;
        }
        else
        {
            // Increment debounce counter
            joint->home_sw_debounce += 1;
        }    
    }
    else if // Enter HOMED state?
    (
        *joint->homing_state_fb == HOME_LATCHING
        && *joint->home_sw_fb == 0
        && stall_diff <= 0
        && velocity_diff == 0 
        && threshold_diff > 0
        && *joint->tmc.velocity_fb != 0
    )
    {
        // Switch has been triggered while in latching phase

        // Indicate switch on
        if (joint->home_sw_debounce > HOME_LATCHING_DEBOUNCE)
        {
            #ifdef DEBUG_HOMING
            rtapi_print("hotshot(%d,%d): State->HOME_HOMED. %d -> %d\n", 
                *joint->tmc.chip, *joint->tmc.motor, *joint->homing_state_fb, HOME_HOMED);
            #endif

            // Debounce threshold crossed. Switch on.
            *joint->home_sw_fb = 1;
            joint->home_sw_debounce = 0;

            // Backing phase is complete. Go to next state.
            *joint->homing_state_fb = HOME_HOMED;
        }
        else
        {
            // Increment debounce counter
            joint->home_sw_debounce += 1;
        }
    }
    else 
    {
        // We havent changed states, so do nothing but reset debouce counter

        #ifdef DEBUG_HOMING
        rtapi_print("hotshot(%d,%d): %d < %d. stall_diff=%d, velocity_fb=%d, velocity_diff=%d, threshold_diff=%d. switch=%d\n", 
            *joint->tmc.chip, *joint->tmc.motor, 
            sg_load, sg_trigger_thresh, 
            stall_diff,
            *joint->tmc.velocity_fb, 
            velocity_diff, 
            threshold_diff, 
            *joint->home_sw_fb);
        #endif

        joint->home_sw_debounce = 0;
    }
}

void hotshot_handle_move(joint_t * joint)
{
    joint->tmc.is_motor_on = *joint->enable_cmd;

    // Move joint and update pins
    // VMAX is set on the fly by PID
    *joint->tmc.velocity_cmd = UNITS_TO_PULSES(*joint->velocity_cmd, joint->unit_pulse_factor);
    // Debugging use only since we don't use positioning mode
    *joint->tmc.position_cmd = UNITS_TO_PULSES(*joint->position_cmd, joint->unit_pulse_factor);

    *joint->position_fb = PULSES_TO_UNITS(*joint->tmc.position_fb, joint->unit_pulse_factor);
    *joint->velocity_fb = PULSES_TO_UNITS(*joint->tmc.velocity_fb, joint->unit_pulse_factor);
}

void hotshot_update_joint(joint_t * joint)
{
    // Always keep these registers up to date
    //
    // Driver status
    tmc5041_pull_register_DRV_STATUS(&joint->tmc);
    // Position
    *joint->tmc.position_fb = tmc5041_get_position(&joint->tmc);
    *joint->position_fb     = PULSES_TO_UNITS(*joint->tmc.position_fb, joint->unit_pulse_factor);
    // Velocity
    *joint->tmc.velocity_fb  = tmc5041_get_velocity(&joint->tmc);
    *joint->velocity_fb      = PULSES_TO_UNITS(*joint->tmc.velocity_fb, joint->unit_pulse_factor);
    // Stallguard threshold
    tmc5041_push_register_COOLCONF(&joint->tmc);
}

void hotshot_handle_joints(joint_t * joints, uint8_t motor_count) {
    // Do something with each joint
    for (uint8_t i = 0; i < motor_count; i++)
    {
        rpi_spi_select(*joints[i].tmc.chip); 

        // hotshot_update_joint(&joints[i]);

        // Move joints
        hotshot_handle_move(&joints[i]);
        hotshot_handle_homing(&joints[i]);

        // hotshot_update_joint(&joints[i]);

        rpi_spi_unselect();
    }
}

/** Read/write to/from TMC5041 over SPI bus.
 * No math should happen in this function, or in any functions it calls.
 */
void hotshot_spi(joint_t * joints, uint8_t motor_count)
{
    for (uint8_t i = 0; i < motor_count; i++)
    {
        rpi_spi_select(*joints[i].tmc.chip); 
        
        //
        // Writes
        //
        // Turn motor on or off
        if (joints[i].tmc.is_motor_on == TRUE)
        {
            tmc5041_motor_power_on(&joints[i].tmc);
        }
        else
        {
            // LinuxCNC power button is off, so power motor off
            // FIXME it's possible for driver to keep counting steps even after tmc5041_motor_power_off
            tmc5041_motor_position_hold(&joints[i].tmc);            
            tmc5041_motor_power_off(&joints[i].tmc);
        }
        // Set turn direction
        //  1: Velocity mode to positive VMAX (using AMAX acceleration)
        //  2: Velocity mode to negative VMAX (using AMAX acceleration)
        if (*joints[i].tmc.velocity_cmd > 0)
            tmc5041_set_register_RAMPMODE(&joints[i].tmc, 1);
        else if (*joints[i].tmc.velocity_cmd < 0)
            tmc5041_set_register_RAMPMODE(&joints[i].tmc, 2);
        // else vmax == 0. do nothing while decelaration ramp finishes
        // Set velocity
        // VMAX is defined as an unsigned int in the datasheet, so it must be absolute
        tmc5041_set_velocity(&joints[i].tmc, *joints[i].tmc.velocity_cmd);

        // TODO move all math to hotshot_handle_move
        //
        // Reads
        //
        // Driver status
        tmc5041_pull_register_DRV_STATUS(&joints[i].tmc);
        // Position
        *joints[i].tmc.position_fb = tmc5041_get_position(&joints[i].tmc);
        // Velocity
        *joints[i].tmc.velocity_fb  = tmc5041_get_velocity(&joints[i].tmc);
        // Stallguard threshold
        tmc5041_push_register_COOLCONF(&joints[i].tmc);

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

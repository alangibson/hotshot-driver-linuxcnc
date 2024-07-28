// bcm2835.c
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>

#include "bcm2835.h"
#include "tmc/ic/TMC5041/TMC5041.h"
#include "hotshot.h"

// ============================================================================
// Test

// Busy wait for a motor to reach requested position.
// Mostly useful for debugging.
void wait_for_move(uint8_t chip, uint8_t motor, float64_t mm_per_microstep) {

    while (true) {

        // rpi_start_spi_conversation(chip);

        // printf("--------------------\n");

        // // Log initial requested move params
        // // printf("move: xtarget=%d, v1=%d, vmax=%d, a1=%d, amax=%d, d1=%d, dmax=%d\n", xtarget, v1, vmax, a1, amax, d1, dmax);

        // // XACTUAL: Actual motor position (signed)
        // uint8_t xactual[40] = { TMC5041_XACTUAL(motor), ____, ____, ____, ____ };
        // bcm2835_spi_transfernb(xactual, xactual, 5); // query
        // bcm2835_spi_transfernb(xactual, xactual, 5); // read
        // printf("xactual <- %02x %02x %02x %02x %02x \n", xactual[0], xactual[1], xactual[2], xactual[3], xactual[4]);
        // int32_t xactual_value = xactual[1] << 24 | xactual[2] << 16 | xactual[3] << 8 | xactual[4];
        // // int32_t xactual_mm = xactual_value * mm_per_microstep;
        // // printf("    xactual=%d, xactual_mm=%d\n", xactual_value, xactual_mm);
        // printf("    xactual=%d\n", xactual_value);

        // float xactual_mm_value = xactual_mm(chip, motor, mm_per_microstep);
        // printf("    xactual_mm=%f\n", xactual_mm_value);

        // // VACTUAL: Actual motor velocity from ramp generator (signed)
        // uint8_t vactual_message[40] = { TMC5041_VACTUAL(motor), ____, ____, ____, ____ };
        // bcm2835_spi_transfernb(vactual_message, vactual_message, 5); // query
        // bcm2835_spi_transfernb(vactual_message, vactual_message, 5); // read
        // printf("vactual <- %02x %02x %02x %02x %02x \n", vactual_message[0], vactual_message[1], vactual_message[2], vactual_message[3], vactual_message[4]);
        // uint32_t vactual_value = vactual_message[1] << 24 | vactual_message[2] << 16 | vactual_message[3] << 8 | vactual_message[4];
        // // float vactual_mm = vactual_value * mm_per_microstep;
        // // printf("    vactual=%d, vactual_mm=%f\n", vactual_value, vactual_mm);
        // printf("    vactual=%d\n", vactual_value);

        // // RAMP_STAT: Ramp and Reference Switch Status Register
        // uint8_t rampstat_message[40] = { TMC5041_RAMPSTAT(motor), ____, ____, ____, ____ };
        // bcm2835_spi_transfernb(rampstat_message, rampstat_message, 5); // query
        // bcm2835_spi_transfernb(rampstat_message, rampstat_message, 5); // read
        // printf("rampstat <- %02x %02x %02x %02x %02x \n", rampstat_message[0], rampstat_message[1], rampstat_message[2], rampstat_message[3], rampstat_message[4]);
        // uint32_t rampstat_payload = rampstat_message[1] << 24 | rampstat_message[2] << 16 | rampstat_message[3] << 8 | rampstat_message[4];
        // bool rampstat_sg_mask           = FIELD_GET(rampstat_payload, TMC5041_STATUS_SG_MASK, TMC5041_STATUS_SG_SHIFT);
        // bool rampstat_second_move       = FIELD_GET(rampstat_payload, TMC5041_SECOND_MOVE_MASK, TMC5041_SECOND_MOVE_SHIFT);
        // bool rampstat_t_zerowait_active = FIELD_GET(rampstat_payload, TMC5041_T_ZEROWAIT_ACTIVE_MASK, TMC5041_T_ZEROWAIT_ACTIVE_SHIFT);
        // bool rampstat_vzero             = FIELD_GET(rampstat_payload, TMC5041_VZERO_MASK, TMC5041_VZERO_SHIFT);
        // bool rampstat_position_reached  = FIELD_GET(rampstat_payload, TMC5041_POSITION_REACHED_MASK, TMC5041_POSITION_REACHED_SHIFT);
        // bool rampstat_velocity_reached  = FIELD_GET(rampstat_payload, TMC5041_VELOCITY_REACHED_MASK, TMC5041_VELOCITY_REACHED_SHIFT);
        // bool event_pos_reached          = FIELD_GET(rampstat_payload, TMC5041_EVENT_POS_REACHED_MASK, TMC5041_EVENT_POS_REACHED_SHIFT);
        // bool event_stop_sg              = FIELD_GET(rampstat_payload, TMC5041_EVENT_STOP_SG_MASK, TMC5041_EVENT_STOP_SG_SHIFT);
        // bool event_stop_r               = FIELD_GET(rampstat_payload, TMC5041_EVENT_STOP_R_MASK, TMC5041_EVENT_STOP_R_SHIFT);
        // bool event_stop_l               = FIELD_GET(rampstat_payload, TMC5041_EVENT_STOP_L_MASK, TMC5041_EVENT_STOP_L_SHIFT);
        // bool status_latch_r             = FIELD_GET(rampstat_payload, TMC5041_STATUS_LATCH_R_MASK, TMC5041_STATUS_LATCH_R_SHIFT);
        // bool status_latch_l             = FIELD_GET(rampstat_payload, TMC5041_STATUS_LATCH_L_MASK, TMC5041_STATUS_LATCH_L_SHIFT);
        // bool status_stop_r              = FIELD_GET(rampstat_payload, TMC5041_STATUS_STOP_R_MASK, TMC5041_STATUS_STOP_R_SHIFT);
        // bool status_stop_l              = FIELD_GET(rampstat_payload, TMC5041_STATUS_STOP_L_MASK, TMC5041_STATUS_STOP_L_SHIFT);
        // printf("    sg_mask=%d, second_move=%d, t_zerowait_active=%d, vzero=%d, position_reached=%d, velocity_reached=%d\n", 
        //     rampstat_sg_mask, rampstat_second_move, rampstat_t_zerowait_active, rampstat_vzero, rampstat_position_reached, rampstat_velocity_reached
        // );
        // printf("    event_pos_reached=%d, event_stop_sg=%d, event_stop_r=%d, event_stop_l=%d, status_latch_r=%d, status_latch_l=%d\n", 
        //     event_pos_reached, event_stop_sg, event_stop_r, event_stop_l, status_latch_r, status_latch_l
        // );
        // printf("    status_latch_r=%d, status_latch_l=%d, status_stop_r=%d, status_stop_l=%d\n", 
        //     status_latch_r, status_latch_l, status_stop_r, status_stop_l
        // );

        // // Avoid busy loop
        // sleep(1);

        // // Exit loop when move is done
        // if (rampstat_position_reached == 1)
        //     break;

        // rpi_end_spi_conversation();
    }
}


// Motor characteristics
// step = 1.8 degree
// #define fullsteps_per_rev 200
// GT2 belt
#define BELT_PITCH 2
// 20 tooth gear
#define GEAR_TEETH 20
// Lead is mm of linear movement per full turn
// lead = starts * pitch
#define SCREW_PITCH 8

// Motor parameters
#define MICROSTEPS 256
// Set in EXTRA_SETUP()
// uint32_t microstep_per_mm;
// uint8_t mres;

// 0 = X, 1 = YL, 2 = YR, 3 = Z
motor_t motors[4];

/**
 * To run:
 *  sudo C_INCLUDE_PATH="$HOME/dev/plasma/TMC-API" halcompile --install hotshot.comp
 *  
*/
int main() {
   
    motors[0] = (motor_t){
        .chip = BCM2835_SPI_CS0,
        .motor = 0,
        .mm_per_rev = BELT_PITCH * GEAR_TEETH,
        .microsteps = MICROSTEPS,
        .microstep_per_mm = microsteps_per_mm(200, BELT_PITCH * GEAR_TEETH, MICROSTEPS),
        .max_acceleration = 750.0};

    printf("Hello world!\n");

    // EXTRA_SETUP();

    // float mm_per_rev        = belt_pitch * gear_teeth;
    // float mm_per_fullstep   = mm_per_rev / fullsteps_per_rev;
    // float mm_per_microstep  = mm_per_fullstep / MICROSTEPS;

    // // Create a common SPI_STATUS variable
    // uint8_t spi_status[40] = { ____, ____, ____, ____, ____ };
    // // and a write operation payload variable
    // uint32_t write_payload = 0x00;

    // uint8_t chip = BCM2835_SPI_CS0;
    // uint8_t motor = 0; // 0 or 1

    // rpi_setup_spi0();
    // config_tmc5041(MICROSTEPS);

    // //
    // // Move motor
    // //

    // // Motor free parameters

    // float xtarget_mm            = 100.0;  // absolute position along an axis
    // float vtarget_mm_per_sec    = 6.0; // machine units / sec
    // move_motor(chip, motor, mm_per_microstep, xtarget_mm, vtarget_mm_per_sec);

    // //
    // // Read moving motor status in a loop
    // //

    // wait_for_move(chip, motor, mm_per_microstep);

    // move_motor(chip, motor, mm_per_microstep, 10.0, vtarget_mm_per_sec);

    // //
    // // Close SPI conversation
    // //

    // // rpi_end_spi_conversation();
    // motor = TMC_DEFAULT_MOTOR;

    // //
    // // Shut down SPI
    // //

    // rpi_teardown_spi0();

    return 0;
}

// Test
// ============================================================================
/*
 * Motor power supply must be on for board to work!
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <bcm2835.h>
#include <tmc/ic/TMC5041/TMC5041.h>

struct {
    uint8_t chip;
    uint8_t motor;
    uint16_t microsteps;
    int16_t fullsteps_per_rev;
    uint8_t belt_pitch;
    uint8_t gear_teeth;
    float64_t mm_per_microstep;
} motor_t;

// 4.1.2 SPI Status Bits Transferred with Each Datagram Read Back
void log_spi_status(uint8_t spi_status[40]) {
    bool status_stop_l2 =       spi_status[0] & 0b01000000;
    bool status_stop_l1 =       spi_status[0] & 0b00100000;
    bool velocity_reached2 =    spi_status[0] & 0b00010000;
    bool velocity_reached1 =    spi_status[0] & 0b00001000;
    bool driver_error2 =        spi_status[0] & 0b00000100;
    bool driver_error1 =        spi_status[0] & 0b00000010;
    bool reset_flag =           spi_status[0] & 0b00000001;
    // printf("    <- %02x %02x %02x %02x %02x \n", spi_status[0], spi_status[1], spi_status[2], spi_status[3], spi_status[4]);
    printf("    spi_status: status_stop_l2=%x status_stop_l1=%x velocity_reached2=%x velocity_reached1=%x driver_error2=%x driver_error1=%x reset_flag=%x\n",
        status_stop_l2, status_stop_l1, velocity_reached2, velocity_reached1, driver_error2, driver_error1, reset_flag
    );
}

// Call once to configure SPI
void setup_spi() {
    // Initialise the library by opening /dev/mem (if you are root) 
    // or /dev/gpiomem (if you are not) and getting pointers to the internal 
    // memory for BCM 2835 device registers.
    int init_success = bcm2835_init();

    // Start SPI operations. 
    // Forces RPi SPI0 pins P1-19 (MOSI), P1-21 (MISO), P1-23 (CLK), 
    // P1-24 (CE0) and P1-26 (CE1) to alternate function ALT0, 
    // which enables those pins for SPI interface.
    int spi_begin_success = bcm2835_spi_begin();

    // Set SPI parameters
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST); // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3); // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256); // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); // the default
}

// Call once to unconfgure SPI
void teardown_spi() {
    // Unselect chip
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);

    // End SPI operations. SPI0 pins P1-19 (MOSI), P1-21 (MISO), 
    // P1-23 (CLK), P1-24 (CE0) and P1-26 (CE1) are returned to 
    // their default INPUT behaviour.
    bcm2835_spi_end();
    
    // Close the library, deallocating any allocated memory and closing /dev/mem
    bcm2835_close();
}

void start_spi_conversation(uint8_t cs) {
    // Select a chip
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
}

void end_spi_conversation() {
    // Unselect chip
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
}

// void write_array(uint8_t reg, uint8_t payload[32]) {
//     uint8_t spi_status[40] = { ____, ____, ____, ____, ____ };
//     uint8_t message[40] = { reg|TMC_WRITE_BIT, payload[0], payload[1], payload[2], payload[3] };
//     bcm2835_spi_transfernb(reg, spi_status, 5);
// }

// void write_value(uint8_t reg, uint32_t payload) {
//     uint8_t pl[32] = { payload >> 24, payload >> 16, payload >> 8, payload };
//     write_array(reg, pl);
// }

// uint8_t* read_array(uint8_t reg) {
//     static uint8_t spi_status[40] = { ____, ____, ____, ____, ____ };
//     uint8_t message[40] = { reg, ____, ____, ____, ____ };
//     bcm2835_spi_transfernb(message, spi_status, 5); // query
//     bcm2835_spi_transfernb(message, spi_status, 5); // read
//     return spi_status;    
// }

// uint32_t read_value(uint8_t reg) {
//     uint8_t spi_status[40] = read_array(reg);
//     uint32_t value = spi_status[1] << 24 | spi_status[2] << 16 | spi_status[3] << 8 | spi_status[4];
//     return value;
// }

void config_chip(uint8_t chip) {
    static uint8_t spi_status[40] = { ____, ____, ____, ____, ____ };

    start_spi_conversation(chip);

    // GCONF
    //
    // Build SPI message
    uint8_t gconf[40] = { TMC5041_GSTAT, ____, ____, ____, ____ };
    // Send message
    bcm2835_spi_transfernb(gconf, spi_status, 5);

    end_spi_conversation();
}

void config_motor(uint8_t chip, uint8_t motor) {

    static uint8_t spi_status[40] = { ____, ____, ____, ____, ____ };
    uint32_t write_payload = 0x00;

    uint8_t rampmode = 0; // positioning mode

    start_spi_conversation(BCM2835_SPI_CS0);

    // Chopper Configuration
    // CHOPCONF: TOFF=5/0b0101, HSTRT=4/0b100, HEND=1, TBL=2/0b10, CHM=0 (spreadCycle), MRES=0 (256)
    //
    // Build message payload
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_TBL_MASK,      TMC5041_TBL_SHIFT,      0b10);
    write_payload = FIELD_SET(write_payload, TMC5041_HEND_MASK,     TMC5041_HEND_SHIFT,     0b1);
    write_payload = FIELD_SET(write_payload, TMC5041_HSTRT_MASK,    TMC5041_HSTRT_SHIFT,    0b100);
    write_payload = FIELD_SET(write_payload, TMC5041_TOFF_MASK,     TMC5041_TOFF_SHIFT,     0b0101);
    // Build SPI message
    uint8_t chop_conf[40] = { TMC5041_CHOPCONF(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    // Send message
    bcm2835_spi_transfernb(chop_conf, spi_status, 5);
    
    // Driver current control
    // IHOLD_IRUN: IHOLD=5, IRUN=31 (max. current), IHOLDDELAY=1 
    //
    // Build message payload
    write_payload = 0x00;
    // TODO write_payload = FIELD_SET(write_payload, TMC5041_TBL_MASK,      TMC5041_TBL_SHIFT,      0b10);
    // uint8_t current[40] = { TMC5041_IHOLD_IRUN(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    // Build SPI message
    uint8_t current[40] = { TMC5041_IHOLD_IRUN(motor)|TMC_WRITE_BIT, 0x00, 0x01, 0x1F, 0x05 };
    // Send message
    bcm2835_spi_transfernb(current, spi_status, 5);

    // Ramp mode
    //
    // 0: Positioning mode (using all A, D and V parameters)
    // 1: Velocity mode to positive VMAX (using AMAX acceleration)
    // 2: Velocity mode to negative VMAX (using AMAX acceleration)
    // 3: Hold mode (velocity remains unchanged, unless stop event occurs)
    //
    // RAMPMODE=0
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_RAMPMODE_MASK,      TMC5041_RAMPMODE_SHIFT,      rampmode);
    uint8_t rampmode_message[40] = { TMC5041_RAMPMODE(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(rampmode_message, spi_status, 5);

    end_spi_conversation();
}

// TODO Move motor to an absolute position, expressed in mm
void move_motor(uint8_t chip, uint8_t motor, float64_t mm_per_microstep, int32_t xtarget_mm, uint16_t vtarget_mm_per_min) {

    static uint8_t spi_status[40] = { ____, ____, ____, ____, ____ };
    uint32_t write_payload = 0x00;

    // Calculate basic ramp params 
    //
    // Ramp parameters
    uint8_t vstart              = 0;
    uint8_t vstop               = 10;
    uint16_t a1                 = 10000;
    uint16_t amax               = a1 / 2;
    uint16_t dmax               = amax;
    uint16_t d1                 = a1;

    //
    // Calculate desired move params
    //
    // Calculate velocity
    int32_t vmax                = ( vtarget_mm_per_min / 60 ) / mm_per_microstep; // in microsteps per second
    int32_t v1                  = vmax / 2;
    // Calculate target position and velocity
    int32_t xtarget             = xtarget_mm / mm_per_microstep;               // in microsteps

    start_spi_conversation(BCM2835_SPI_CS0);

    // VSTART=0
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTART_MASK,      TMC5041_VSTART_SHIFT,      vstart);
    uint8_t vstart_message[40] = { TMC5041_VSTART(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(vstart_message, spi_status, 5);
    
    // VSTOP=10
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VSTOP_MASK,      TMC5041_VSTOP_SHIFT,      vstop);
    uint8_t vstop_message[40] = { TMC5041_VSTOP(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(vstop_message, spi_status, 5);

    // A1
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_A1_MASK,      TMC5041_A1_SHIFT,      a1);
    uint8_t a1_message[40] = { TMC5041_A1(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(a1_message, spi_status, 5);
    
    // V1
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_V1__MASK,      TMC5041_V1__SHIFT,      v1);
    uint8_t v1_message[40] = { TMC5041_V1(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(v1_message, spi_status, 5);

    // Target velocity [µsteps / t]
    //
    // This is the target velocity in velocity mode. It can be changed any time during a motion
    //
    // VMAX
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_VMAX_MASK,      TMC5041_VMAX_SHIFT,      vmax);
    uint8_t vmax_message[40] = { TMC5041_VMAX(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(vmax_message, spi_status, 5);

    // Maximum acceleration/deceleration [µsteps / ta²]
    //
    // This is the acceleration and deceleration value for velocity mode.
    // In position mode (RAMP=0), must be lower than A1 (???)
    //
    // AMAX
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_AMAX_MASK,      TMC5041_AMAX_SHIFT,      amax);
    uint8_t amax_message[40] = { TMC5041_AMAX(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(amax_message, spi_status, 5);

    // DMAX = AMAX
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_DMAX_MASK,      TMC5041_DMAX_SHIFT,      dmax);
    uint8_t dmax_message[40] = { TMC5041_DMAX(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(dmax_message, spi_status, 5);

    // D1 = A1
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_D1_MASK,      TMC5041_D1_SHIFT,      d1);
    uint8_t d1_message[40] = { TMC5041_D1(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    bcm2835_spi_transfernb(d1_message, spi_status, 5);

    // Reset XACTUAL to 0 so setting XTARGET will cause a move
    // uint8_t xactual_0[40] = { TMC5041_XACTUAL(motor)|TMC_WRITE_BIT, 0x00, 0x00, 0x00, 0x00 };
    // bcm2835_spi_transfernb(xactual_0, spi_status, 5);

    // Target position for RAMPMODE=0 (signed). 
    // Write a new target position to this register in order to activate the ramp generator positioning in RAMPMODE=0. 
    // Initialize all velocity, acceleration and deceleration parameters before.
    //
    // XTARGET
    //
    // Build payload
    write_payload = 0x00;
    write_payload = FIELD_SET(write_payload, TMC5041_XTARGET_MASK,  TMC5041_XTARGET_SHIFT,  xtarget);
    // Build message
    uint8_t xtarget_message[40] = { TMC5041_XTARGET(motor)|TMC_WRITE_BIT, write_payload >> 24, write_payload >> 16, write_payload >> 8, write_payload };
    // Send message
    bcm2835_spi_transfernb(xtarget_message, spi_status, 5);

    end_spi_conversation();

}

// Busy wait for a motor to reach requested position.
// Mostly useful for debugging.
void wait_for_move(uint8_t chip, uint8_t motor) {

    while (true) {

        start_spi_conversation(chip);

        printf("--------------------\n");

        // Log initial requested move params
        // printf("move: xtarget=%d, v1=%d, vmax=%d, a1=%d, amax=%d, d1=%d, dmax=%d\n", xtarget, v1, vmax, a1, amax, d1, dmax);

        // XACTUAL: Actual motor position (signed)
        uint8_t xactual[40] = { TMC5041_XACTUAL(motor), ____, ____, ____, ____ };
        bcm2835_spi_transfernb(xactual, xactual, 5); // query
        bcm2835_spi_transfernb(xactual, xactual, 5); // read
        printf("xactual <- %02x %02x %02x %02x %02x \n", xactual[0], xactual[1], xactual[2], xactual[3], xactual[4]);
        int32_t xactual_value = xactual[1] << 24 | xactual[2] << 16 | xactual[3] << 8 | xactual[4];
        // int32_t xactual_mm = xactual_value * mm_per_microstep;
        // printf("    xactual=%d, xactual_mm=%d\n", xactual_value, xactual_mm);
        printf("    xactual=%d\n", xactual_value);

        // VACTUAL: Actual motor velocity from ramp generator (signed)
        uint8_t vactual_message[40] = { TMC5041_VACTUAL(motor), ____, ____, ____, ____ };
        bcm2835_spi_transfernb(vactual_message, vactual_message, 5); // query
        bcm2835_spi_transfernb(vactual_message, vactual_message, 5); // read
        printf("vactual <- %02x %02x %02x %02x %02x \n", vactual_message[0], vactual_message[1], vactual_message[2], vactual_message[3], vactual_message[4]);
        uint32_t vactual_value = vactual_message[1] << 24 | vactual_message[2] << 16 | vactual_message[3] << 8 | vactual_message[4];
        // float vactual_mm = vactual_value * mm_per_microstep;
        // printf("    vactual=%d, vactual_mm=%f\n", vactual_value, vactual_mm);
        printf("    vactual=%d\n", vactual_value);

        // RAMP_STAT: Ramp and Reference Switch Status Register
        uint8_t rampstat_message[40] = { TMC5041_RAMPSTAT(motor), ____, ____, ____, ____ };
        bcm2835_spi_transfernb(rampstat_message, rampstat_message, 5); // query
        bcm2835_spi_transfernb(rampstat_message, rampstat_message, 5); // read
        printf("rampstat <- %02x %02x %02x %02x %02x \n", rampstat_message[0], rampstat_message[1], rampstat_message[2], rampstat_message[3], rampstat_message[4]);
        uint32_t rampstat_payload = rampstat_message[1] << 24 | rampstat_message[2] << 16 | rampstat_message[3] << 8 | rampstat_message[4];
        bool rampstat_sg_mask           = FIELD_GET(rampstat_payload, TMC5041_STATUS_SG_MASK, TMC5041_STATUS_SG_SHIFT);
        bool rampstat_second_move       = FIELD_GET(rampstat_payload, TMC5041_SECOND_MOVE_MASK, TMC5041_SECOND_MOVE_SHIFT);
        bool rampstat_t_zerowait_active = FIELD_GET(rampstat_payload, TMC5041_T_ZEROWAIT_ACTIVE_MASK, TMC5041_T_ZEROWAIT_ACTIVE_SHIFT);
        bool rampstat_vzero             = FIELD_GET(rampstat_payload, TMC5041_VZERO_MASK, TMC5041_VZERO_SHIFT);
        bool rampstat_position_reached  = FIELD_GET(rampstat_payload, TMC5041_POSITION_REACHED_MASK, TMC5041_POSITION_REACHED_SHIFT);
        bool rampstat_velocity_reached  = FIELD_GET(rampstat_payload, TMC5041_VELOCITY_REACHED_MASK, TMC5041_VELOCITY_REACHED_SHIFT);
        bool event_pos_reached          = FIELD_GET(rampstat_payload, TMC5041_EVENT_POS_REACHED_MASK, TMC5041_EVENT_POS_REACHED_SHIFT);
        bool event_stop_sg              = FIELD_GET(rampstat_payload, TMC5041_EVENT_STOP_SG_MASK, TMC5041_EVENT_STOP_SG_SHIFT);
        bool event_stop_r               = FIELD_GET(rampstat_payload, TMC5041_EVENT_STOP_R_MASK, TMC5041_EVENT_STOP_R_SHIFT);
        bool event_stop_l               = FIELD_GET(rampstat_payload, TMC5041_EVENT_STOP_L_MASK, TMC5041_EVENT_STOP_L_SHIFT);
        bool status_latch_r             = FIELD_GET(rampstat_payload, TMC5041_STATUS_LATCH_R_MASK, TMC5041_STATUS_LATCH_R_SHIFT);
        bool status_latch_l             = FIELD_GET(rampstat_payload, TMC5041_STATUS_LATCH_L_MASK, TMC5041_STATUS_LATCH_L_SHIFT);
        bool status_stop_r              = FIELD_GET(rampstat_payload, TMC5041_STATUS_STOP_R_MASK, TMC5041_STATUS_STOP_R_SHIFT);
        bool status_stop_l              = FIELD_GET(rampstat_payload, TMC5041_STATUS_STOP_L_MASK, TMC5041_STATUS_STOP_L_SHIFT);
        printf("    sg_mask=%d, second_move=%d, t_zerowait_active=%d, vzero=%d, position_reached=%d, velocity_reached=%d\n", 
            rampstat_sg_mask, rampstat_second_move, rampstat_t_zerowait_active, rampstat_vzero, rampstat_position_reached, rampstat_velocity_reached
        );
        printf("    event_pos_reached=%d, event_stop_sg=%d, event_stop_r=%d, event_stop_l=%d, status_latch_r=%d, status_latch_l=%d\n", 
            event_pos_reached, event_stop_sg, event_stop_r, event_stop_l, status_latch_r, status_latch_l
        );
        printf("    status_latch_r=%d, status_latch_l=%d, status_stop_r=%d, status_stop_l=%d\n", 
            status_latch_r, status_latch_l, status_stop_r, status_stop_l
        );

        // Avoid busy loop
        sleep(1);

        // Exit loop when move is done
        if (rampstat_position_reached == 1)
            break;

        end_spi_conversation();
    }

}

int main() {

    // Create a common SPI_STATUS variable
    uint8_t spi_status[40] = { ____, ____, ____, ____, ____ };
    // and a write operation payload variable
    uint32_t write_payload = 0x00;

    //
    // Set up SPI
    //

    setup_spi();

    //
    // Configure driver chip
    //

    config_chip(BCM2835_SPI_CS0);

    //
    // Configure motor
    //

    uint8_t motor = 0; // 0 or 1
    config_motor(BCM2835_SPI_CS0, motor);

    //
    // Move motor
    //

    // Motor free parameters
    uint16_t microsteps         = 256;
    int16_t fullsteps_per_rev   = 200;  // step = 1.8 degree
    uint8_t belt_pitch          = 2;    // GT2 belt
    uint8_t gear_teeth          = 20;   // 20 tooth gear
    // Calculate mm length of a microstep
    float64_t mm_per_rev        = belt_pitch * gear_teeth;
    float64_t mm_per_fullstep   = mm_per_rev / fullsteps_per_rev;
    float64_t mm_per_microstep  = mm_per_fullstep / microsteps;

    int32_t xtarget_mm          = 100;  // absolute position along an axis
    uint16_t vtarget_mm_per_min = 2000; // linuxcnc uses mm/min
    move_motor(BCM2835_SPI_CS0, motor, mm_per_microstep, xtarget_mm, vtarget_mm_per_min);

    //
    // Read moving motor status in a loop
    //

    wait_for_move(BCM2835_SPI_CS0, motor);

    move_motor(BCM2835_SPI_CS0, motor, mm_per_microstep, 10, vtarget_mm_per_min);

    //
    // Close SPI conversation
    //

    // end_spi_conversation();
    motor = TMC_DEFAULT_MOTOR;

    //
    // Shut down SPI
    //

    teardown_spi();

    return 0;
}

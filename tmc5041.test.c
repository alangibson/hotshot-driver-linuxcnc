#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <tmc5041.h>
#include <motor.h>
#include <rpi.h>

// Global variable for the motor so signal handler can access it
static tmc5041_motor_t * MOTOR = NULL;

// Signal handler for Ctrl+C
void handle_sigint(int sig) {
    printf("\nShutting down motor...\n");
    motor_end(MOTOR);
    rpi_spi_unselect();
    rpi_end();
    exit(0);
}

int main(int argc, char *argv[]) {
    // Set up signal handler
    signal(SIGINT, handle_sigint);

    // Check command line arguments
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <chip_number> <motor_number>\n", argv[0]);
        return 1;
    }

    // Convert arguments to integers
    int chip = atoi(argv[1]);
    int motor_num = atoi(argv[2]);
    // TODO use tandem axis mode if there is a second motor_num provided

    MOTOR = tmc5041_motor_create(chip, motor_num);
    // Initialize command values
    // Sets load sensing sensitivity
    *MOTOR->sg_thresh_cmd = 10;
    // TODO can we guess this somehow? Or can it be 0 since we don't use built in stallguard?
    *MOTOR->cs_thresh_cmd = 10000;
    *MOTOR->hold_current_cmd = 8;
    *MOTOR->run_current_cmd = 23;
    // TODO remove from hotshot_joint_init?
    *MOTOR->mres = tmc5041_microsteps_to_mres(256);
    // TODO remove from  hotshot.comp?
    // MOTOR->acceleration_time_ref = tmc5041_acceleration_time_ref(TMC5041_CLOCK_HZ);
    MOTOR->max_acceleration_cmd = 600000;
    // Must be nonzero for motor to power up
    *MOTOR->chop_toff_cmd = 5;

    rpi_init();
    printf("RPi init done\n");

    rpi_spi_select(chip);

    motor_init(MOTOR);
    motor_set_on(MOTOR);
    motor_set_velocity(MOTOR, 100000);

    // Print motor load every second
    while(1) {
        motor_update(MOTOR);
        printf("motor (%d,%d) load: %d, vel: %d, pos: %d\n", chip, motor_num, 
            motor_get_load(MOTOR), motor_get_velocity(MOTOR), motor_get_position(MOTOR));
        sleep(1);
    }

    return 0;
}
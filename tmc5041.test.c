#include <unistd.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <tmc5041.h>
#include <motor.h>

// Global variable for the motor so signal handler can access it
static tmc5041_motor_t * MOTOR = NULL;

// Signal handler for Ctrl+C
void handle_sigint(int sig) {
    if (MOTOR != NULL) {
        printf("\nShutting down motor...\n");
        motor_off(MOTOR);
    }
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
    // TODO can this just be 0?
    *MOTOR->sg_thresh_cmd = 5;
    // TODO can we guess this somehow? Or can it be 0 since we don't use built in stallguard?
    *MOTOR->cs_thresh_cmd = 10000;
    // TODO move into motor_current(run, hold) ?
    *MOTOR->hold_current_cmd = 8;
    *MOTOR->run_current_cmd = 23;
    // TODO init in tmc5041_motor_create
    MOTOR->sg_stop_cmd = 0;
    // Happens in hotshot_joint_init
    MOTOR->mres = tmc5041_microsteps_to_mres(256);
    // Happens in hotshot.comp
    MOTOR->acceleration_time_ref = tmc5041_acceleration_time_ref(TMC5041_CLOCK_HZ);
    // Done in motor_on
    // *tmc5041_motors[0].chop_toff_cmd = 5;
    // There are dedicated functions for these
    // *tmc5041_motors[0].velocity_cmd = 80000;
    // *tmc5041_motors[0].position_cmd = 0;
    // tmc5041_motors[0].acceleration_cmd = 8000;
    // Calculated in motor_init
    // tmc5041_motors[0].velocity_time_ref = tmc5041_velocity_time_ref(TMC5041_CLOCK_HZ);

    motor_init(MOTOR);
    motor_on(MOTOR);
    motor_set_acceleration(6000);
    motor_set_velocity(1000);
    motor_rotate(MOTOR, CW);

    // Print motor load every second
    while(1) {
        printf("motor (%d,%d) load: %d\n", chip, motor_num, motor_load(MOTOR));
        sleep(1);
    }

    return 0;
}
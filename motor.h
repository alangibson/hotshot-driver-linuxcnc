typedef int32_t motor_position_t;
typedef int32_t motor_velocity_t;
typedef int32_t motor_load_t;
typedef uint8_t motor_direction_t; // CW = 1, CCW = 2

// Read and/or write registers over SPI
// Must be kept as fast as possible. Minimize calculations done here.
void motor_update(tmc5041_motor_t * motor);

// Allow driver to configure motor.
// Nothing done here should cause motors to move.
// Happens immediately.
void motor_init(tmc5041_motor_t * motor);

// Allow driver to tear down motor configuration.
// Nothing done here should cause motors to move.
// Happens immediately.
void motor_end(tmc5041_motor_t * motor);

// Set internal motor state to power off.
// Does not take effect until motor_update() is called
void motor_set_on(tmc5041_motor_t * motor);

// Set internal motor state to power on.
// Does not take effect until motor_update() is called
void motor_set_off(tmc5041_motor_t * motor);

// Set target/max velocity.
// Does not take effect until motor_update() is called
void motor_set_velocity(tmc5041_motor_t * motor, uint32_t vel);

// Stop motor motion.
// Does not take effect until motor_update() is called
void motor_set_stop(tmc5041_motor_t * motor);

// Set direction of rotation
// Does not take effect until motor_update() is called
void motor_set_direction(tmc5041_motor_t * motor, motor_direction_t dir);

// Notify motor that it is at homed position.
// Does not take effect until motor_update() is called
void motor_set_homed(tmc5041_motor_t * motor);

motor_position_t motor_get_position(tmc5041_motor_t * motor);

motor_velocity_t motor_get_velocity(tmc5041_motor_t * motor);

motor_load_t motor_get_load(tmc5041_motor_t * motor);


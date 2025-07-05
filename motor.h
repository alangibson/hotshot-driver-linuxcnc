typedef uint8_t motor_dir_t;
typedef int32_t motor_load_t;

typedef int32_t motor_position_t;
typedef int32_t motor_velocity_t;
typedef int32_t motor_acceleration_t;

/** 
 * Allow driver to set up motor.
 * Must not power motor up or start motion.
 * Use motor_on to power motor on.
 */
void motor_init(tmc5041_motor_t * motor);
void motor_end(tmc5041_motor_t * motor);

/** Power motor up */
void motor_on(tmc5041_motor_t * motor);
/** Power motor down */
void motor_off(tmc5041_motor_t * motor);
/**
 * Rotate motor endlessly in given direction.
 *
 * Initialize motor with motor_init before calling.
 * Power motor on with motor_on before calling.
 * Set velocity and acceleration before calling or no motion will happen.
 */
void motor_rotate(tmc5041_motor_t * motor, motor_dir_t dir);
/** 
 * Stop motor motion and hold position.
 * Use motor_off to power motor down.
 */
void motor_stop(tmc5041_motor_t * motor);

/** Read/write to motor driver over SPI.
  * Call as frequently as possible .
  */
void motor_update(tmc5041_motor_t * motor);

motor_load_t motor_get_load(tmc5041_motor_t * motor);

motor_velocity_t motor_get_velocity(tmc5041_motor_t * motor);
void motor_set_velocity(tmc5041_motor_t * motor, motor_velocity_t vel);

motor_position_t motor_get_position(tmc5041_motor_t * motor);

void motor_set_on(tmc5041_motor_t * motor);
void motor_set_off(tmc5041_motor_t * motor);

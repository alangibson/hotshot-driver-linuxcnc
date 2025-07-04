typedef motor_num_t uint32_t
typedef motor_dir_t uint8_t

typedef int32_t motor_position_t;
typedef int32_t motor_velocity_t;
typedef uint32_t motor_acceleration_t;

/** 
 * Allow driver to set up motor.
 * Must not power motor up or start motion.
 * Use motor_on to power motor on.
 */
void motor_init(motor_num_t motor);
void motor_end(motor_num_t motor);

/** Power motor up */
void motor_on(motor_num_t motor);
/** Power motor down */
void motor_off(motor_num_t motor);
/**
 * Rotate motor endlessly in given direction.
 *
 * Initialize motor with motor_init before calling.
 * Power motor on with motor_on before calling.
 * Set velocity and acceleration before calling or no motion will happen.
 */
void motor_rotate(motor_num_t motor, motor_dir_t dir);
/** 
 * Stop motor motion and hold position.
 * Use motor_off to power motor down.
 */
void motor_stop(motor_num_t motor);

/** Read/write to motor driver over SPI.
  * Call as frequently as possible .
  */
void motor_update(tmc5041_motor_t * motor)

void motor_set_velocity(int32_t vel);
void motor_set_acceleration(int32_t acc);

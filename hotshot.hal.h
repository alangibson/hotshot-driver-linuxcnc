void hotshot_init(joint_t * joints, uint8_t motor_count);
void hotshot_spi(joint_t * joints, uint8_t motor_count);
void hotshot_end(joint_t * motors, uint8_t motor_count);
void hotshot_handle_joints(joint_t * motors, uint8_t motor_count);
void hotshot_handle_move(joint_t * motor);
void hotshot_handle_homing(joint_t * joint);
bool hotshot_joint_init(joint_t * joint);



// uint32_t microsteps_per_mm(uint32_t fullsteps_per_rev, float linear_mm_per_rev, uint32_t microsteps);
// drv_status_register_t tmc5041_get_register_DRV_STATUS(joint_t * motor);
// bool get_home_switch_state(joint_t * motor, bool axis_is_homing);
// bool get_neg_limit_switch_state(joint_t * motor, float axis_max_velocity_cmd);
// spi_status_t follow(joint_t * motor);
// float thc_voltage(uint8_t chip, float V_ref);
// void log_move(joint_t * motor, float position_cmd, float max_velocity_cmd, float position_fb);
// ramp_stat_register_t tmc5041_get_register_RAMP_STAT(joint_t * motor);
// bool setup_once();
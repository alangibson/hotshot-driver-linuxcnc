
void calibrate()
{
    float vtarget_mm_per_sec    = axis_x_max_velocity_cmd;
    float xtarget_mm            = 0.0;   // absolute position along an axis
    rpi_start_spi_conversation(motors[0].chip);
    // Start movement
    //
    tmc5041_set_register_RAMPMODE(&motors[0], 1); // velocity mode
    move_motor(motors[0], xtarget_mm);
    // Get calibration info
    //
    int64_t xactual = tmc5041_get_register_XACTUAL(&motors[0]);
    float xactualmm = xactual_mm(&motors[0]);
    float vmax = mm_to_microsteps(motors[0].microstep_per_mm, axis_x_max_velocity_cmd);
    ramp_stat_register_t ramp_stat = tmc5041_get_register_RAMP_STAT(&motors[0]);
    bool get_home_switch = get_home_switch_state(&motors[0], 0);
    bool get_neg_limit_switch = get_neg_limit_switch_state(&motors[0], vtarget_mm_per_sec);
    drv_status_register_t drv_status = tmc5041_get_register_DRV_STATUS(&motors[0]);
    uint16_t sg_result = drv_status.sg_result;
    uint8_t cs_actual = drv_status.cs_actual;
    bool sg_status = drv_status.sg_status;
    axis_x_home_sw = get_home_switch;
    axis_x_neg_limit_sw = get_neg_limit_switch;
    axis_x_position_fb = xactualmm;
    axis_x_load = sg_result;
    axis_x_current = cs_actual;
    axis_x_stall = sg_status;
    // And echo it
    //
    rtapi_print("hotshot (%d:%d): Calibrate\n", motors[0].chip, motors[0].motor);
    rtapi_print("    axis_x_max_velocity_cmd=%f, vmax=%f, axis_x_max_acceleration_cmd=%f\n", 
        axis_x_max_velocity_cmd, vmax, axis_x_max_acceleration_cmd);
    rtapi_print("    xactual_mm=%f, xactual=%d\n", 
        xactualmm, xactual);
    rtapi_print("    axis_x_load=%d, axis_x_current=%d, axis_x_stall=%d\n", 
        sg_result, cs_actual, sg_status);
    rtapi_print("    event_stop_sg=%d\n", ramp_stat.event_stop_sg);
    rtapi_print("    axis_x_home_sw=%d, axis_x_neg_limit_sw=%d\n", 
        get_home_switch, get_neg_limit_switch);
    rpi_end_spi_conversation();
}
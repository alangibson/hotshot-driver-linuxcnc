
void motor_set_on(tmc5041_motor_t * motor)
{
    motor->is_motor_on = TRUE;
}

void motor_set_off(tmc5041_motor_t * motor)
{
    motor->is_motor_on = FALSE;
}
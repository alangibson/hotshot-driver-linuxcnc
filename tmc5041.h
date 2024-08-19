#include "tmc/helpers/API_Header.h"
#include "tmc/ic/TMC5041/TMC5041_Register.h"
#include "tmc/ic/TMC5041/TMC5041_Constants.h"
#include "tmc/ic/TMC5041/TMC5041_Fields.h"

#ifndef GLOBAL_OK
#include "global.h"
#endif

// Taken from TMC5041.h
#define TMC5041_FIELD_READ(tdef, address, mask, shift) \
	FIELD_GET(tmc5041_readInt(tdef, address), mask, shift)
#define TMC5041_FIELD_WRITE(tdef, address, mask, shift, value) \
	(tmc5041_writeInt(tdef, address, FIELD_SET(tmc5041_readInt(tdef, address), mask, shift, value)))


typedef struct {
    bool standstill;
    bool open_load_phase_b;
    bool open_load_phase_a;
    bool ground_short_phase_b;
    bool ground_short_phase_a;
    bool overtemp_warning;
    bool overtemp_alarm;
    bool full_stepping;
    bool sg_status;
    uint16_t sg_result;
    uint8_t cs_actual;
} drv_status_register_t;

typedef struct {
    uint8_t mres;
} chopconf_register_t;

typedef struct {
    bool status_sg;
    bool second_move;
    bool t_zerowait_active;
    bool vzero;
    bool position_reached;
    bool velocity_reached;
    bool event_pos_reached;
    bool event_stop_sg;
    bool event_stop_r;
    bool event_stop_l;
    bool status_latch_r;
    bool status_latch_l;
    bool status_stop_r;
    bool status_stop_l;
} ramp_stat_register_t;

typedef struct {
    bool status_stop_l2;
    bool status_stop_l1;
    bool velocity_reached2;
    bool velocity_reached1;
    bool driver_error2;
    bool driver_error1;
    bool reset_flag;
} spi_status_t;


typedef int32_t     sg_thresh_t;
typedef bool        home_sw_t;
typedef bool        torch_breakaway_t;
typedef float64_t   position_t;
typedef int32_t     tmc_position_t;
typedef int32_t     tmc_velocity_t;
typedef float64_t   velocity_t;

typedef struct {
    volatile uint32_t               *   chip;
	volatile uint32_t               *   motor;
    uint8_t     					    mres;
    volatile tmc_position_t     	*   position_cmd;
    tmc_position_t     	                last_position_cmd;
    volatile tmc_velocity_t         *   velocity_cmd;
    uint32_t                            acceleration_cmd;
    uint32_t                            max_acceleration_cmd;
    bool                                is_motor_on;
    volatile uint32_t               *   cs_thresh_cmd;      // coolStep threshold in ppt for TMC register
    volatile bool                   *   sg_stop_cmd;
    volatile uint32_t               *   run_current_cmd;
    volatile uint32_t               *   hold_current_cmd;
    volatile uint32_t               *   current_hold_delay_cmd;
    volatile uint32_t               *   ramp_mode_cmd;
    volatile uint32_t               *   ramp_a1_cmd;
    volatile uint32_t               *   ramp_d1_cmd;
    volatile uint32_t               *   ramp_dmax_cmd;
    volatile uint32_t               *   ramp_vstart_cmd;
    volatile uint32_t               *   ramp_vstop_cmd;
    volatile uint32_t               *   ramp_v1_cmd;
    volatile uint32_t               *   ramp_tzerowait_cmd;
    volatile uint32_t               *   coolstep_sfilt_cmd;
    volatile uint32_t               *   coolstep_seimin_cmd;
    volatile uint32_t               *   coolstep_sedn_cmd;
    volatile uint32_t               *   coolstep_seup_cmd;
    volatile uint32_t               *   coolstep_semin_cmd;
    volatile uint32_t               *   coolstep_semax_cmd;
    volatile uint32_t               *   chop_mode_cmd;
    volatile uint32_t               *   chop_vhigh_cmd;
    volatile uint32_t               *   chop_vhighchm_cmd;
    volatile uint32_t               *   chop_vhighfs_cmd;
    volatile uint32_t               *   chop_tbl_cmd;
    volatile uint32_t               *   chop_hend_cmd;
    volatile uint32_t               *   chop_hstrt_cmd;
    volatile uint32_t               *   chop_toff_cmd;
    volatile uint32_t               *   chop_vsense_cmd;
    volatile uint32_t               *   sw_en_softstop;
    volatile sg_thresh_t        	*   sg_thresh_cmd;
    volatile tmc_position_t     	*   position_fb;
    volatile tmc_velocity_t     	*   velocity_fb;
    volatile int32_t             	*   acceleration_fb;
    volatile bool                   *   motor_standstill_fb;
    volatile bool                   *   motor_full_stepping_fb;
    volatile bool                   *   motor_overtemp_warning_fb;
    volatile bool                   *   motor_overtemp_alarm_fb;
    volatile int32_t                *   motor_load_fb;
    volatile uint32_t               *   motor_current_fb;
    volatile bool                   *   motor_stall_fb;
    volatile uint32_t               *   microstep_resolution_fb;
    volatile bool                   *   velocity_reached_fb;
    volatile bool                   *   status_sg_fb;
    // volatile bool                   *   second_move_fb;
    // volatile bool                   *   t_zerowait_active_fb;
    // volatile bool                   *   vzero_fb;
    volatile bool                   *   position_reached_fb;
    volatile bool                   *   event_pos_reached_fb;
    volatile bool                   *   event_stop_sg_fb;
    volatile bool                   *   event_stop_r_fb;
    volatile bool                   *   event_stop_l_fb;
    volatile bool                   *   status_latch_r_fb;
    volatile bool                   *   status_latch_l_fb;
    volatile bool                   *   status_stop_r_fb;
    volatile bool                   *   status_stop_l_fb;
} tmc5041_motor_t;

int32_t tmc5041_writeDatagram(tmc5041_motor_t * motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
int32_t tmc5041_writeInt(tmc5041_motor_t * motor, uint8_t address, int32_t value);
int32_t tmc5041_readInt(tmc5041_motor_t * motor, uint8_t address);

//
// Setup and init functions
//
void tmc5041_chip_init();
void tmc5041_motor_init(tmc5041_motor_t * motor);
void tmc5041_motor_set_config_registers(tmc5041_motor_t * motor);
void tmc5041_init(tmc5041_motor_t * motors, size_t motor_count);
void tmc5041_end(tmc5041_motor_t * motors, size_t motor_count);
uint8_t microsteps_to_tmc_mres(uint16_t usteps);

//
// Register access
//
void tmc5041_set_register_XACTUAL(tmc5041_motor_t * motor, int32 xactual);
int32_t tmc5041_get_register_XACTUAL(tmc5041_motor_t * motor);
int32_t tmc5041_get_register_VACTUAL(tmc5041_motor_t * motor);
spi_status_t tmc5041_set_register_XTARGET(tmc5041_motor_t * motor, int32 xtarget);
void tmc5041_pull_register_RAMP_STAT(tmc5041_motor_t * motor);
ramp_stat_register_t tmc5041_get_register_RAMP_STAT(tmc5041_motor_t * motor);
drv_status_register_t tmc5041_get_register_DRV_STATUS(tmc5041_motor_t * motor);
chopconf_register_t tmc5041_get_register_CHOPCONF(tmc5041_motor_t * motor);
void tmc5041_set_register_VMAX(tmc5041_motor_t * motor, int32_t vmax);
void tmc5041_set_register_VSTART(tmc5041_motor_t * motor, int32_t vstart);
void tmc5041_set_register_VSTOP(tmc5041_motor_t * motor, int32_t vstop);
void tmc5041_set_register_AMAX(tmc5041_motor_t * motor, int32_t amax);
void tmc5041_set_register_RAMPMODE(tmc5041_motor_t * motor, int32_t mode);
void tmc5041_set_register_VCOOLTHRS(tmc5041_motor_t * motor, int32_t vcoolthrs);
void tmc5041_push_register_SW_MODE(tmc5041_motor_t * motor);
void tmc5041_pull_register_DRV_STATUS(tmc5041_motor_t * motor);
void tmc5041_push_register_COOLCONF(tmc5041_motor_t * motor);

//
// Higher order and/or complex functions
//
void tmc5041_set_velocity(tmc5041_motor_t * motor, int32_t vmax);
bool tmc5041_motor_clear_stall(tmc5041_motor_t * motor);
bool tmc5041_motor_set_home(tmc5041_motor_t * motor);
void tmc5041_motor_end(tmc5041_motor_t * motor);
void tmc5041_motor_power_on(tmc5041_motor_t * motor);
void tmc5041_motor_power_off(tmc5041_motor_t * motor);
void tmc5041_motor_position_hold(tmc5041_motor_t * motor);

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


typedef uint32_t    sg_thresh_t;
typedef bool        home_sw_t;
typedef bool        torch_breakaway_t;
typedef float64_t   position_t;
typedef int32_t     tmc_position_t;
typedef int32_t     tmc_velocity_t;
typedef float64_t   velocity_t;

typedef struct {
	uint8_t chip;
} tmc5041_chip_t;

typedef struct {
	tmc5041_chip_t					chip;
	uint8_t							motor;
    uint8_t     					mres;
    volatile uint32_t               * cs_thresh_cmd;      // coolStep threshold in ppt for TMC register
    volatile bool                   * sg_stop_cmd;
    volatile uint32_t               * run_current_cmd;
    volatile uint32_t               * hold_current_cmd;
    volatile sg_thresh_t        	* sg_thresh_cmd;
    volatile float64_t              * max_velocity_cmd;
    volatile uint32_t               * max_acceleration_cmd;
    volatile tmc_position_t     	* position_fb;
    volatile tmc_velocity_t     	* velocity_fb;
    volatile bool                   * motor_standstill_fb;
    volatile bool                   * motor_full_stepping_fb;
    volatile bool                   * motor_overtemp_warning_fb;
    volatile bool                   * motor_overtemp_alarm_fb;
    volatile int32_t                * motor_load_fb;
    volatile uint32_t               * motor_current_fb;
    volatile bool                   * motor_stall_fb;
    volatile uint32_t               * microstep_resolution_fb;
} tmc5041_motor_t;

int32_t tmc5041_writeDatagram(tmc5041_motor_t * motor, uint8_t address, uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
int32_t tmc5041_writeInt(tmc5041_motor_t * motor, uint8_t address, int32_t value);
int32_t tmc5041_readInt(tmc5041_motor_t * motor, uint8_t address);

void tmc5041_chip_init();
void tmc5041_motor_init(tmc5041_motor_t * motor);
void tmc5041_init(tmc5041_motor_t * motors, size_t motor_count);
void tmc5041_end(tmc5041_motor_t * motors, size_t motor_count);

void tmc5041_set_register_XACTUAL(tmc5041_motor_t * motor, int32 xactual);
int32_t tmc5041_get_register_XACTUAL(tmc5041_motor_t * motor);
int32_t tmc5041_get_register_VACTUAL(tmc5041_motor_t * motor);
spi_status_t tmc5041_set_register_XTARGET(tmc5041_motor_t * motor, int32 xtarget);
ramp_stat_register_t tmc5041_get_register_RAMP_STAT(tmc5041_motor_t * motor);
drv_status_register_t tmc5041_get_register_DRV_STATUS(tmc5041_motor_t * motor);
chopconf_register_t tmc5041_get_register_CHOPCONF(tmc5041_motor_t * motor);
bool tmc5041_motor_set_home(tmc5041_motor_t * motor);
void tmc5041_motor_end(tmc5041_motor_t * motor);
void tmc5041_motor_halt(tmc5041_motor_t * motor);
#include "bcm2835.h"
#include "rpi.h"
#include "tmc/ic/TMC5041/TMC5041.h"

// RAMPMODE:
// 0: Positioning mode (using all A, D and V parameters)
// 1: Velocity mode to positive VMAX (using AMAX acceleration)
// 2: Velocity mode to negative VMAX (using AMAX acceleration)
// 3: Hold mode (velocity remains unchanged, unless stop event occurs)
#define TMC5041_RAMPMODE_POS 0
#define TMC5041_RAMPMODE_VEL_POS 1
#define TMC5041_RAMPMODE_VEL_NEG 2
#define TMC5041_RAMPMODE_HOLD 3

typedef bool        pin_enable_t;   // hal_bit_t aka int
typedef bool        pin_is_homing_t;
typedef bool        pin_sg_stop_t;
typedef uint32_t    pin_sg_thresh_t;
typedef bool        pin_home_sw_t;
typedef bool        pin_torch_breakaway_t;
typedef float64_t   pin_position_t;
typedef int32_t     pin_tmc_position_t;
typedef int32_t     pin_tmc_velocity_t;
typedef float64_t   pin_velocity_t;
typedef bool        pin_tmc_motor_standstill_t;
typedef bool        pin_tmc_motor_full_stepping_t;
typedef bool        pin_tmc_motor_overtemp_warning_t;
typedef bool        pin_tmc_motor_overtemp_alarm_t;
typedef int32_t     pin_tmc_motor_load_t;
typedef uint32_t    pin_tmc_motor_current_t;
typedef bool        pin_tmc_motor_stall_t;
typedef uint32_t    pin_tmc_microstep_resolution_t;

typedef struct {
    uint32_t                        chip;
    uint32_t                        motor;
    uint32_t                        pitch;
    uint32_t                        teeth;
    bool                            is_on;
    float                           mm_per_rev;
    uint32_t                        microstep_per_mm;
    uint8_t                         tmc_mres_cmd;
    uint32_t                        cs_thresh_cmd;      // coolStep threshold in machine units / time
    uint32_t                        tmc_cs_thresh_cmd;      // coolStep threshold in ppt for TMC register
    // input
    volatile pin_enable_t           * is_enabled;
    bool                            is_homing;
    volatile pin_position_t         * position_cmd;       // position in machine units (mm or inch)
    volatile pin_velocity_t         * max_velocity_cmd;       // maximum velocity
    float                           max_acceleration_cmd;   // maximum acceleration
    uint16_t                        microsteps_cmd;     // desired microsteps (1/microsteps_cmd)
    volatile bool                   * tmc_sg_stop_cmd;
    uint8_t                         tmc_run_current_cmd;
    uint8_t                         tmc_hold_current_cmd;
    volatile pin_sg_thresh_t        * tmc_sg_thresh_cmd;
    volatile float64_t              * tmc_max_velocity_fb;
    volatile uint32_t               * tmc_max_acceleration_fb;
    // output
    bool                            home_sw;
    bool                            neg_limit_sw;
    uint16_t                        load;
    uint8_t                         current;
    bool                            position_reached;
    bool                            t_zerowait_active;
    bool                            velocity_reached;
    volatile pin_torch_breakaway_t  * torch_breakaway_fb;
    volatile pin_sg_stop_t          * sg_stop_fb;
    volatile pin_home_sw_t          * home_sw_fb;
    volatile pin_position_t         * position_fb;
    volatile pin_tmc_position_t     * tmc_position_fb;
    volatile pin_velocity_t         * velocity_fb;
    volatile pin_tmc_velocity_t     * tmc_velocity_fb;
    volatile pin_tmc_motor_standstill_t         * tmc_motor_standstill_fb;
    volatile pin_tmc_motor_full_stepping_t      * tmc_motor_full_stepping_fb;
    volatile pin_tmc_motor_overtemp_warning_t   * tmc_motor_overtemp_warning_fb;
    volatile pin_tmc_motor_overtemp_alarm_t     * tmc_motor_overtemp_alarm_fb;
    volatile pin_tmc_motor_load_t               * tmc_motor_load_fb;
    volatile pin_tmc_motor_current_t            * tmc_motor_current_fb;
    volatile pin_tmc_motor_stall_t              * tmc_motor_stall_fb;
    volatile pin_tmc_microstep_resolution_t     * tmc_microstep_resolution_fb;
} joint_t;

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
    bool diss2g;
    uint8_t mres;
    bool vhighchm;
    bool vhighfs;
    bool vsense;
    uint8_t tbl;
    bool chm;
    bool rndtf;
    bool disfdcc;
    // bool fd3;
    uint8_t hend;
    uint8_t hstrt;
    uint8_t toff;
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
    uint8_t irun;
    uint8_t ihold;
    uint8_t iholddelay;
} ihold_irun_register_t;

typedef struct {
    bool status_stop_l2;
    bool status_stop_l1;
    bool velocity_reached2;
    bool velocity_reached1;
    bool driver_error2;
    bool driver_error1;
    bool reset_flag;
    // Not actually in SPI_STATUS message
    pin_position_t xactual_mm;
} spi_status_t;

void config_chip();
void reset_motor(joint_t * motor);
void teardown_tmc5041(joint_t motors[], size_t motor_count);

// Register accessors
void                    tmc5041_set_register_RAMPMODE(joint_t * motor, uint8_t rampmode);
int32_t                 tmc5041_get_register_XACTUAL(joint_t * motor);
int32_t                 tmc5041_get_register_VACTUAL(joint_t * motor);
chopconf_register_t     tmc5041_get_register_CHOPCONF(joint_t * motor);
spi_status_t            tmc5041_set_register_XTARGET(joint_t * motor, int32 xtarget);
int32_t                 tmc5041_get_register_XTARGET(joint_t * motor);
ihold_irun_register_t   tmc5041_get_register_IHOLD_IRUN(joint_t * motor);

uint8_t     microsteps_to_tmc_mres(uint16_t usteps);
bool        set_home(joint_t * motor);
void        config_tmc5041(joint_t motors[], size_t motor_count);
void        update_tmc5041(joint_t motors[], size_t motor_count);
uint32_t    microsteps_per_mm(uint32_t fullsteps_per_rev, float linear_mm_per_rev, uint32_t microsteps);
int64_t     mm_to_microsteps(uint32_t microstep_per_mm, float mm);

void handle_joint(joint_t * motor);
void report_joint(joint_t * joint);
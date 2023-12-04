#include "tmc/helpers/Types.h"
#include "tmc5041.h"

typedef bool pin_enable_t;   // hal_bit_t aka int
typedef bool pin_is_homing_t;
typedef bool pin_sg_stop_t;
typedef uint32_t    pin_sg_thresh_t;
typedef bool        pin_home_sw_t;
typedef bool        pin_torch_breakaway_t;
typedef float64_t   pin_position_t;
typedef int32_t     pin_tmc_position_t;
typedef int32_t     pin_tmc_velocity_t;
typedef float64_t   pin_velocity_t;

typedef struct {
    uint32_t    chip;
    uint32_t    motor;
    tmc5041_motor_t tmc;
    bool        is_on;
    float       mm_per_rev;
    uint32_t    microstep_per_mm;
    uint8_t     tmc_mres;
    uint32_t    cs_thresh_cmd;      // coolStep threshold in machine units / time
    volatile uint32_t               * tmc_cs_thresh_cmd;      // coolStep threshold in ppt for TMC register
    // input
    volatile uint32_t               * pitch;
    volatile uint32_t               * teeth;
    volatile pin_enable_t           * is_enabled;
    volatile bool                   * is_homing;
    volatile pin_position_t         * position_cmd;       // position in machine units (mm or inch)
    float                           max_velocity_cmd;       // maximum velocity
    float                           max_acceleration_cmd;   // maximum acceleration
    uint16_t                        microsteps_cmd;     // desired microsteps (1/microsteps_cmd)
    volatile bool                   * tmc_sg_stop_cmd;
    volatile uint32_t               * tmc_run_current_cmd;
    volatile uint32_t               * tmc_hold_current_cmd;
    volatile pin_sg_thresh_t        * tmc_sg_thresh_cmd;
    volatile float64_t              * tmc_max_velocity_cmd;
    volatile uint32_t               * tmc_max_acceleration_cmd;
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
    volatile pin_tmc_position_t     * tmc_position_cmd;
    volatile pin_velocity_t         * velocity_fb;
    volatile pin_tmc_velocity_t     * tmc_velocity_cmd;
    volatile bool                   * tmc_motor_standstill_fb;
    volatile bool                   * tmc_motor_full_stepping_fb;
    volatile bool                   * tmc_motor_overtemp_warning_fb;
    volatile bool                   * tmc_motor_overtemp_alarm_fb;
    volatile int32_t                * tmc_motor_load_fb;
    volatile uint32_t               * tmc_motor_current_fb;
    volatile bool                   * tmc_motor_stall_fb;
    volatile uint32_t               * tmc_microstep_resolution_fb;
} joint_t;

#include "tmc/helpers/Types.h"
#include "tmc5041.h"

typedef bool        pin_enable_t;
typedef bool        pin_is_homing_t;
typedef bool        pin_sg_stop_t;
typedef uint32_t    pin_sg_thresh_t;
typedef bool        pin_home_sw_t;
typedef bool        pin_torch_breakaway_t;
typedef float64_t   pin_position_t;
typedef int32_t     pin_tmc_position_t;
typedef int32_t     pin_tmc_velocity_t;
typedef float64_t   pin_velocity_t;

typedef struct {
    tmc5041_motor_t                 tmc;
    bool                            is_setup;
    bool                            is_on;
    float                           mm_per_rev;
    uint32_t                        microstep_per_mm;
    uint32_t                        cs_thresh_cmd;              // coolStep threshold in machine units / time
    volatile uint32_t               * pitch_cmd;
    volatile uint32_t               * teeth_cmd;
    volatile uint32_t               * motor_fullsteps_per_rev_cmd;
    volatile pin_enable_t           * is_enabled_cmd;
    volatile bool                   * is_homing_cmd;
    volatile pin_position_t         * position_cmd;             // position in machine units (mm or inch)
    volatile pin_velocity_t         * velocity_cmd;
    volatile float64_t              * max_velocity_cmd;         // maximum velocity
    volatile float64_t              * max_acceleration_cmd;     // maximum acceleration
    volatile uint32_t               * microsteps_cmd;           // desired microsteps (1/microsteps_cmd)
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
    volatile pin_velocity_t         * velocity_fb;
    volatile bool                   * estop_fb;
    int32_t                         last_xtarget;
    float64_t                       last_position_fb;
    int32_t                         last_move_distance;
} joint_t;

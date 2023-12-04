#include "hotshot.lib.h"

bool hotshot_init(joint_t * motors, uint8_t motor_count);
void hotshot_end(joint_t * motors, uint8_t motor_count);
void handle_joints(joint_t * motors, uint8_t motor_count);
void handle_joint(joint_t * motor);


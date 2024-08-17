#include "hotshot.lib.h"

void hotshot_init();
void hotshot_end(joint_t * motors, uint8_t motor_count);
void hotshot_handle_joints(joint_t * motors, uint8_t motor_count);
void hotshot_handle_move(joint_t * motor);


#ifndef __TASK_CANMOTOR_H__
#define __TASK_CANMOTOR_H__


extern void can_filter_init(void);
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

#endif

#include "cmsis_os.h"
#include "main.h"

#include "task_canMotor.h"
#include "remote_control.h"
#include "pid.h"

extern RC_ctrl_t rc_ctrl;

void entry_motor(void const * argument)
{
    can_filter_init();
    int16_t i = 1000;
    int16_t current = 164;
    while(1)
    {
        // chassis motor x4
        // single motor
        // chassis
        // PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);

        current = rc_ctrl.rc.ch[3];
        // if(toe_is)
        // else{
        CAN_cmd_chassis(current, i, i, i);
        // }
        osDelay(2);

        // gimbal motor x1
        CAN_cmd_gimbal(10000, 10000, 10000, 10000);
        osDelay(2);
    }
}

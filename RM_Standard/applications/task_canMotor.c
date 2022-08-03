#include "cmsis_os.h"
#include "main.h"

#include "task_canMotor.h"
#include "task_SysStatus.h"
#include "remote_control.h"
#include "pid.h"

extern RC_ctrl_t rc_ctrl;
pid_type_def motor_pid;
const motor_measure_t *motor_data;
// uint16_t set_speed = 164;
extern int16_t current;
extern fp32 ref;
extern fp32 set;

void entry_motor(void const * argument)
{
    can_filter_init();
    
    //chassis motor speed PID
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};

    //chassis angle PID
    //底盘角度pid值
    // const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};

    // chassis_motor_measure = get_chassis_motor_measure_point(0);
    PID_init(&motor_pid, PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    CAN_cmd_chassis(0, 164, 164, 164);
    osDelay(200);

    while(1)
    {
        // chassis motor x4
        // single motor
        ref = getSpeedpoint();
        set = rc_ctrl.rc.ch[3];
        if(rc_ctrl.rc.s[1] == 3)
        {
            if(rc_ctrl.rc.s[0] == 3)
                set = 0;
            else if(rc_ctrl.rc.s[0] == 1)
                set = 200;
        }

        current = PID_calc(&motor_pid, ref, set);
        // current = set;
        
        CAN_cmd_chassis(current, 164, 164, 164);
        osDelay(2);

        // gimbal motor x1
        // CAN_cmd_gimbal(10000, 10000, 10000, 10000);
        // osDelay(2);
    }
}

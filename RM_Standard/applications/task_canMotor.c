#include "cmsis_os.h"
#include "main.h"

#include "task_canMotor.h"

void entry_motor(void const * argument)
{
    can_filter_init();
    while(1)
    {
        CAN_cmd_chassis(4000, 4000, 4000, 4000);
        osDelay(2);
        CAN_cmd_gimbal(10000, 10000, 10000, 10000);
        osDelay(2);
    }
}

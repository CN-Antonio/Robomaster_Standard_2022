#include "cmsis_os.h"
#include "main.h"

#include "task_canMotor.h"

void entry_motor(void const * argument)
{
    can_filter_init();
    int16_t i = 1000;
    while(1)
    {
        // chassis motor x4
        CAN_cmd_chassis(165, i, i, i);
        osDelay(2);

        // gimbal motor x1
        CAN_cmd_gimbal(10000, 10000, 10000, 10000);
        osDelay(2);
    }
}

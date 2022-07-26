#include "cmsis_os.h"
#include "main.h"

#include "task_SysStatus.h"

void entry_SysStatus(void const * argument)
{
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
        osDelay(500);
    }
    
}

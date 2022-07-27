#include "cmsis_os.h"
#include "main.h"

#include "task_led.h"

void blue_led_task(void const * argument)
{
    while(1)
    {
        // HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
        osDelay(500);
    }
}

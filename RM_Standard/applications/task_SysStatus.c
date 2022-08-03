#include "cmsis_os.h"
#include "main.h"

#include "task_SysStatus.h"
#include "task_canMotor.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "remote_control.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern UART_HandleTypeDef huart6;

volatile fp32 voltage_vrefint_proportion = 8.0586080586080586080586080586081e-4f;
extern fp32 handware_version, voltage, temperature;
extern fp32 gyro[3], accel[3], temp;

const motor_measure_t *chassis_motor_measure;
const RC_ctrl_t *local_rc_ctrl;

extern void BMI088_read(fp32 gyro[3], fp32 accel[3], fp32 *temperate);
extern uint8_t BMI088_init(void);
extern uint8_t ist8310_init(void);

uint8_t get_hardware_version(void)
{
    uint8_t hardware_version;
    hardware_version = HAL_GPIO_ReadPin(HW0_GPIO_Port, HW0_Pin)
                                | (HAL_GPIO_ReadPin(HW1_GPIO_Port, HW1_Pin)<<1)
                                | (HAL_GPIO_ReadPin(HW2_GPIO_Port, HW2_Pin)<<2);

    return hardware_version;
}
// ADC function begin
static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;//ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}

void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}

fp32 get_battery_voltage(void)
{
    fp32 voltage;
    uint16_t adcx = 0;

    adcx = adcx_get_chx_value(&hadc3, ADC_CHANNEL_8);
    //(22K + 200K) / 22K = 10.090909090909090909090909090909
    voltage =  (fp32)adcx * voltage_vrefint_proportion * 10.090909090909090909090909090909f;

    return voltage;
}

fp32 get_temprate(void)
{
    uint16_t adcx = 0;
    fp32 temperate;

    adcx = adcx_get_chx_value(&hadc1, ADC_CHANNEL_TEMPSENSOR);
    temperate = (fp32)adcx * voltage_vrefint_proportion;
    temperate = (temperate - 0.76f) * 400.0f + 25.0f;

    return temperate;
}
// ADC function End

// 
void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    //返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

    usart6_tx_dma_enable(tx_buf, len);

} //uart_printf

// temp fun
int16_t getSpeedpoint(void)
{
    return chassis_motor_measure->speed_rpm;
}

void entry_SysStatus(void const * argument)
{
	init_vrefint_reciprocal();  // ADC
	while(BMI088_init()){}      // SPI-IMU
	ist8310_init();				// I2C-Magnet
    // CAN motor measure
    chassis_motor_measure = get_chassis_motor_measure_point(0);
    // RC recv
    local_rc_ctrl = get_remote_control_point();
    while (1)
    {
        handware_version = get_hardware_version();
		// ADC
		voltage = get_battery_voltage();
		temperature = get_temprate();

        if(voltage < 12){
			HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
			HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
            osDelay(50);
		}
		else{
			HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
			HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
            osDelay(500);
		}
		// BMI088
        BMI088_read(gyro, accel, &temp);
        
        // Uart Report
        HAL_UART_Transmit(&huart6, "\r\n**********\r\n", 14, 100);

        /*/
        usart_printf(
        "ch0:%d\r\n\
        ch1:%d\r\n\
        ch2:%d\r\n\
        ch3:%d\r\n\
        ch4:%d\r\n\
        s1:%d\r\n\
        s2:%d\r\n\
        mouse_x:%d\r\n\
        mouse_y:%d\r\n\
        press_l:%d\r\n\
        press_r:%d\r\n\
        key:%d\r\n\
        **********\r\n",
        local_rc_ctrl->rc.ch[0], local_rc_ctrl->rc.ch[1], local_rc_ctrl->rc.ch[2], local_rc_ctrl->rc.ch[3], local_rc_ctrl->rc.ch[4],
        local_rc_ctrl->rc.s[0], local_rc_ctrl->rc.s[1],
        local_rc_ctrl->mouse.x, local_rc_ctrl->mouse.y,local_rc_ctrl->mouse.z, local_rc_ctrl->mouse.press_l, local_rc_ctrl->mouse.press_r,
        local_rc_ctrl->key.v);//*/

        //*/
        usart_printf(
        "M0 ecd:%d\r\n\
        M0 rpm:%d\r\n\
        M0 current:%d\r\n\
        M0 temp:%d\r\n\
        M0 last_ecd:%d\r\n\
        **********\r\n",
        chassis_motor_measure->ecd, chassis_motor_measure->speed_rpm, chassis_motor_measure->given_current, chassis_motor_measure->temperate, chassis_motor_measure->last_ecd
        );//*/
        

        osDelay(100);
    }
    
}

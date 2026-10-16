#include "./BSP/LED/led.h"

/**
 * @brief       Initialize LED related I/O ports and enable clocks
 * @param       None
 * @retval      None
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    LED0_GPIO_CLK_ENABLE();                                 /* LED0 clock enable */
    LED1_GPIO_CLK_ENABLE();                                 /* LED1 clock enable */
    
    gpio_init_struct.Pin = LED0_GPIO_PIN;                   /* LED0 pin */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* Push-pull output */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* Pull-up */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* High speed */
    HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);       /* Initialize LED0 pin */
    
    gpio_init_struct.Pin = LED1_GPIO_PIN;                   /* LED1 pin */
    HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);       /* Initialize LED1 pin */
    
    LED0(1);                                                /* Turn off LED0 */
    LED1(1);                                                /* Turn off LED1 */
}

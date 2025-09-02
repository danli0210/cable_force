/**
 ****************************************************************************************************
 * @file        key.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       Key input driver code
 * @copyright   Copyright (c) 2025 Southeast University. All rights reserved.
 * @license     This software is provided for academic and research purposes only.
 *              Commercial use requires explicit permission from the author.
 ****************************************************************************************************
 * @attention
 *
 * Development Platform: STM32F407 Discovery/Explorer Board
 * Target MCU: STM32F407VGT6
 * System Core: ARM Cortex-M4 @ 168MHz
 *
 * Revision History
 * V1.0 20250601
 *
 ****************************************************************************************************
 */

#include "./BSP/KEY/key.h"
#include "./SYSTEM/delay/delay.h"

/**
 * @brief       Key initialization function
 * @param       None
 * @retval      None
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;                          /* GPIO configuration parameter storage variable */
    
    KEY0_GPIO_CLK_ENABLE();                                     /* KEY0 clock enable */
    KEY1_GPIO_CLK_ENABLE();                                     /* KEY1 clock enable */
    KEY2_GPIO_CLK_ENABLE();                                     /* KEY2 clock enable */
    WKUP_GPIO_CLK_ENABLE();                                     /* WKUP clock enable */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0 pin */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* Input mode */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* Pull-up */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* High speed */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0 pin mode setting, pull-up input */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;                       /* KEY1 pin */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* Input mode */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* Pull-up */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* High speed */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);           /* KEY1 pin mode setting, pull-up input */

    gpio_init_struct.Pin = KEY2_GPIO_PIN;                       /* KEY2 pin */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* Input mode */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* Pull-up */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* High speed */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);           /* KEY2 pin mode setting, pull-up input */

    gpio_init_struct.Pin = WKUP_GPIO_PIN;                       /* WKUP pin */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* Input mode */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                      /* Pull-down */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* High speed */
    HAL_GPIO_Init(WKUP_GPIO_PORT, &gpio_init_struct);           /* WKUP pin mode setting, pull-down input */
}

/**
 * @brief       Key scanning function
 * @note        This function has response priority (when multiple keys are pressed simultaneously): WK_UP > KEY2 > KEY1 > KEY0!!
 * @param       mode: 0 / 1, specific meanings as follows:
 *   @arg       0:  Does not support continuous press (when key is pressed and held, only the first call will return key value,
 *                   must be released and pressed again to return other key values)
 *   @arg       1:  Supports continuous press (when key is pressed and held, each call to this function will return key value)
 * @retval      Key value, defined as follows:
 *              KEY0_PRES, 1, KEY0 pressed
 *              KEY1_PRES, 2, KEY1 pressed
 *              KEY2_PRES, 3, KEY2 pressed
 *              WKUP_PRES, 4, WKUP pressed
 */
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* Key release flag */
    uint8_t keyval = 0;

    if (mode) key_up = 1;       /* Support continuous press */

    if (key_up && (KEY0 == 0 || KEY1 == 0 || KEY2 == 0 || WK_UP == 1))  /* Key release flag is 1, and any key is pressed */
    {
        delay_ms(10);           /* Debouncing */
        key_up = 0;

        if (KEY0 == 0)  keyval = KEY0_PRES;

        if (KEY1 == 0)  keyval = KEY1_PRES;

        if (KEY2 == 0)  keyval = KEY2_PRES;

        if (WK_UP == 1) keyval = WKUP_PRES;
    }
    else if (KEY0 == 1 && KEY1 == 1 && KEY2 == 1 && WK_UP == 0)         /* No key pressed, mark key as released */
    {
        key_up = 1;
    }

    return keyval;              /* Return key value */
}

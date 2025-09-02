/**
 ****************************************************************************************************
 * @file        rng.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       RNG driver implementation
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

#include "./BSP/RNG/rng.h"
#include "./SYSTEM/delay/delay.h"


RNG_HandleTypeDef g_rng_handle;     /* RNG handle */

/**
 * @brief       Initialize RNG
 * @param       None
 * @retval      0 if success, 1 if failure
 */
uint8_t rng_init(void)
{
    uint16_t retry = 0;

    g_rng_handle.Instance = RNG;
    HAL_RNG_DeInit(&g_rng_handle);
    HAL_RNG_Init(&g_rng_handle);    /* Initialize RNG */

    while (__HAL_RNG_GET_FLAG(&g_rng_handle, RNG_FLAG_DRDY) == RESET && retry < 10000)   /* Wait until RNG is ready */
    {
        retry++;
        delay_us(10);
    }
    
    if (retry >= 10000)
    {
        return 1;       /* RNG is not working properly */
    }
    return 0;
}

/**
 * @brief       RNG low-level initialization, clock source setup and enable
 * @note        This function is called by HAL_RNG_Init()
 * @param       hrng : RNG handle
 * @retval      None
 */
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
    __HAL_RCC_RNG_CLK_ENABLE();     /* Enable RNG clock */
}

/**
 * @brief       Get a random number
 * @param       None
 * @retval      Random number (32-bit)
 */
uint32_t rng_get_random_num(void)
{
    uint32_t randomnum;
    HAL_RNG_GenerateRandomNumber(&g_rng_handle, &randomnum);
    return randomnum;
}

/**
 * @brief       Get a random number within a range
 * @param       min,max : minimum and maximum values
 * @retval      Random number rval, satisfying min <= rval <= max
 */
int rng_get_random_range(int min, int max)
{ 
    uint32_t randomnum;
    HAL_RNG_GenerateRandomNumber(&g_rng_handle, &randomnum);
    return randomnum % (max - min + 1) + min;
}

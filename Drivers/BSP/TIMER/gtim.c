/**
 ****************************************************************************************************
 * @file        gtim.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.1
 * @date        2025-06-01
 * @brief       General-purpose timer driver code - with real-time analysis support
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
 * Initial release
 * V1.1 20250615
 * Added real-time tension analysis trigger function
 *
 ****************************************************************************************************
 */

#include "./BSP/TIMER/gtim.h"
#include "./BSP/LED/led.h"

/******************************************************************************************/
/* Timer configuration handle definition */
TIM_HandleTypeDef g_timx_handle; /* Timer x handle */

/******************************************************************************************/
/* HAL general callback interface functions */
// External function declaration
extern void trigger_realtime_analysis(void);

/**
 * @brief       Timer update interrupt callback function
 * @param       htim: Timer handle pointer
 * @note        This function is called by timer interrupt functions
 * @retval      None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&g_timx_handle)) /* General-purpose timer interrupt actual execution content */
    {
        // Keep original LED toggle function
        LED1_TOGGLE();
        
        // Trigger real-time tension analysis
        trigger_realtime_analysis();
    }
}

/**
 * @brief       General-purpose timer TIMX interrupt initialization function
 * @note
 *              General-purpose timer clock source is APB1. When PPRE1 ¡Ý 2 prescaler,
 *              general-purpose timer clock is 2x APB1 clock. APB1 is 42MHz, so timer clock = 84MHz
 *              Timer overflow time calculation: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft = timer working frequency, unit: MHz
 *
 * @param       arr: Auto-reload value
 * @param       psc: Clock prescaler
 * @retval      None
 */
void gtim_timx_int_init(uint16_t arr, uint16_t psc)
{
    GTIM_TIMX_INT_CLK_ENABLE(); /* Enable TIMx clock */
    g_timx_handle.Instance = GTIM_TIMX_INT;                     /* General-purpose timer x */
    g_timx_handle.Init.Prescaler = psc;                         /* Prescaler */
    g_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;        /* Up counter */
    g_timx_handle.Init.Period = arr;                            /* Auto-reload value */
    g_timx_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;  /* Clock division factor */
    HAL_TIM_Base_Init(&g_timx_handle);
    HAL_NVIC_SetPriority(GTIM_TIMX_INT_IRQn, 1, 3);             /* Set interrupt priority, preemption priority 1, sub-priority 3 */
    HAL_NVIC_EnableIRQ(GTIM_TIMX_INT_IRQn);                     /* Enable TIMx interrupt */
    HAL_TIM_Base_Start_IT(&g_timx_handle);                      /* Enable timer x and timer x update interrupt */
}

/**
 * @brief       Timer interrupt service routine
 * @param       None
 * @retval      None
 */
void GTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_handle);
}

/**
 * @brief       Stop timer
 * @param       None
 * @retval      None
 */
void gtim_timx_stop(void)
{
    HAL_TIM_Base_Stop_IT(&g_timx_handle);
}

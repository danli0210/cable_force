/**
 ****************************************************************************************************
 * @file        gtim.h
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
 * Added timer stop function
 *
 ****************************************************************************************************
 */

#ifndef __GTIM_H
#define __GTIM_H
#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* General-purpose timer definitions */
/* TIMX interrupt definitions 
 * Default configuration is for TIM2~TIM5.
 * Note: By modifying these 4 macro definitions, any timer from TIM1~TIM8 can be supported.
 */
 
#define GTIM_TIMX_INT                       TIM3
#define GTIM_TIMX_INT_IRQn                  TIM3_IRQn
#define GTIM_TIMX_INT_IRQHandler            TIM3_IRQHandler
#define GTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM3_CLK_ENABLE(); }while(0)  /* TIM3 clock enable */

/******************************************************************************************/

/* Function declarations */
void gtim_timx_int_init(uint16_t arr, uint16_t psc);        /* General-purpose timer interrupt initialization function */
void gtim_timx_stop(void);                                  /* Stop timer */

#endif

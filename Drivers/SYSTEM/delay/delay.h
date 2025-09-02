/**
 ****************************************************************************************************
 * @file        delay.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       SysTick normal counting mode for delay management
 *              Provides delay_init initialization function, delay_us and delay_ms delay functions
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
 
#ifndef __DELAY_H
#define __DELAY_H

#include "./SYSTEM/sys/sys.h"

/* Function declarations */
void delay_init(uint16_t sysclk);           /* Initialize delay function */
void delay_ms(uint16_t nms);                /* Delay nms milliseconds */
void delay_us(uint32_t nus);                /* Delay nus microseconds */

#if (!SYS_SUPPORT_OS)                       /* Not using SysTick interrupt */
    void HAL_Delay(uint32_t Delay);         /* HAL library delay function, needed by SDIO etc. */
#endif

#endif /* __DELAY_H */

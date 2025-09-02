/**
 ****************************************************************************************************
 * @file        sys.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       System initialization code (including clock config/interrupt management/GPIO setup)
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
 * First Release
 * V1.0 20250601
 ****************************************************************************************************
 */

#ifndef __SYS_H
#define __SYS_H

#include "stm32f4xx.h"
#include "core_cm4.h" 
#include "stm32f4xx_hal.h"

/**
 * SYS_SUPPORT_OS used to define whether system folder supports OS
 * 0: Does not support OS
 * 1: Supports OS
 */
#define SYS_SUPPORT_OS         0

/* Function Declarations ********************************************************************************/

void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);                         /* Set interrupt vector offset */
void sys_standby(void);                                                                     /* Enter standby mode */
void sys_soft_reset(void);                                                                  /* System software reset */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);   /* Configure system clock */

/* Assembly functions below */
void sys_intx_disable(void);        /* Disable all interrupts */
void sys_intx_enable(void);         /* Enable all interrupts */
void sys_msr_msp(uint32_t addr);    /* Set stack pointer address */

#endif /* __SYS_H */

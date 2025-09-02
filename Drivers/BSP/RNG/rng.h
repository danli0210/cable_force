/**
 ****************************************************************************************************
 * @file        rng.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       RNG driver code
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

#ifndef __RNG_H
#define __RNG_H 

#include "./SYSTEM/sys/sys.h"

uint8_t rng_init(void);                         /* Initialize RNG */
uint32_t rng_get_random_num(void);              /* Get a random number */
int rng_get_random_range(int min, int max);     /* Get a random number within a specified range */

#endif

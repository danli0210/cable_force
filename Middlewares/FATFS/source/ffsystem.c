/**
 ****************************************************************************************************
 * @file        ffsystem.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       FATFS low-level (ffsystem) driver code
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
 *
 ****************************************************************************************************
 */

#include "./MALLOC/malloc.h"
#include "./SYSTEM/sys/sys.h"
#include "./FATFS/source/ff.h"

/**
 * @brief       Get current time
 * @param       None
 * @retval      Time value
 *   @note      Time encoding rules:
 *              User defined function to give a current time to fatfs module 
 *              31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31)
 *              15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) 
 */
DWORD get_fattime (void)
{
    return 0;
}

/**
 * @brief       Dynamic memory allocation
 * @param       size : Memory size to allocate (bytes)
 * @retval      Start address of allocated memory
 */
void *ff_memalloc (UINT size)
{
    return (void*)mymalloc(SRAMIN, size);
}

/**
 * @brief       Free memory
 * @param       mf  : Memory start address
 * @retval      None
 */
void ff_memfree (void* mf)
{
    myfree(SRAMIN, mf);
}

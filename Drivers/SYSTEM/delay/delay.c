/**
 ****************************************************************************************************
 * @file        delay.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       SysTick normal counting mode for delay management (supports RTOS)
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

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/delay/delay.h"

static uint32_t g_fac_us = 0;       /* us delay multiplier */

/* If SYS_SUPPORT_OS is defined, means OS support is required (not limited to UCOS) */
#if SYS_SUPPORT_OS

/* Add common header files (needed by UCOS) */
#include "os.h"

/* Define g_fac_ms variable, represents ms delay multiplier, indicates ms per tick (only needed when OS is enabled) */
static uint16_t g_fac_ms = 0;

/*
 *  When delay_us/delay_ms need to support OS, three OS-related macro definitions and functions are needed for support
 *  First, 3 macro definitions:
 *      delay_osrunning    : Used to indicate whether OS is currently running, to decide whether related functions can be used
 *      delay_ostickspersec: Used to indicate OS clock tick rate, delay_init will initialize systick based on this parameter
 *      delay_osintnesting : Used to indicate OS interrupt nesting level, because scheduling is not allowed in interrupts, delay_ms uses this parameter to decide how to run
 *  Then 3 functions:
 *      delay_osschedlock  : Used to lock OS task scheduling, disable scheduling
 *      delay_osschedunlock: Used to unlock OS task scheduling, re-enable scheduling
 *      delay_ostimedly    : Used for OS delay, can cause task scheduling.
 *
 *  This example only supports UCOSII, for other OS, please port by reference
 */

/* Support UCOSII */
#define delay_osrunning     OSRunning           /* OS running flag, 0=not running; 1=running */
#define delay_ostickspersec OS_TICKS_PER_SEC    /* OS clock tick rate, i.e., scheduling times per second */
#define delay_osintnesting  OSIntNesting        /* Interrupt nesting level, i.e., number of interrupt nesting */

/**
 * @brief     Lock task scheduling during us-level delay (prevent interruption of us-level delay)
 * @param     None
 * @retval    None
 */
void delay_osschedlock(void)
{
    OSSchedLock();                      /* UCOSII method, disable scheduling to prevent us delay interruption */
}

/**
 * @brief     Restore task scheduling during us-level delay
 * @param     None
 * @retval    None
 */
void delay_osschedunlock(void)
{
    OSSchedUnlock();                    /* UCOSII method, restore scheduling */
}

/**
 * @brief     OS time delay function
 * @param     ticks: Number of ticks to delay
 * @retval    None
 */
void delay_ostimedly(uint32_t ticks)
{
    OSTimeDly(ticks);                               /* UCOSII delay */
}

/**
 * @brief     SysTick interrupt service function, used when OS is running
 * @param     None  
 * @retval    None
 */  
void SysTick_Handler(void)
{
    /* Only execute normal scheduling when OS starts running */
    if (delay_osrunning == OS_TRUE)
    {
        /* Call uC/OS-II SysTick interrupt service function */
        OS_CPU_SysTickHandler();
    }
    HAL_IncTick();
}
#endif

/**
 * @brief     Initialize delay function
 * @param     sysclk: System clock frequency, i.e., CPU frequency (rcc_c_ck), 168MHz
 * @retval    None
 */  
void delay_init(uint16_t sysclk)
{
#if SYS_SUPPORT_OS                                      /* If OS support is needed */
    uint32_t reload;
#endif
    g_fac_us = sysclk;                                  /* Since systick has been configured in HAL_Init, no need to reconfigure here */
#if SYS_SUPPORT_OS                                      /* If OS support is needed */
    reload = sysclk;                                    /* Count times per second, unit is M */
    reload *= 1000000 / delay_ostickspersec;            /* Set overflow time according to delay_ostickspersec, reload is 24-bit
                                                         * register, maximum value: 16777216, at 168M, approximately 0.09986s
                                                         */
    g_fac_ms = 1000 / delay_ostickspersec;              /* Represents minimum delay unit that OS can provide */
    SysTick->CTRL |= 1 << 1;                            /* Enable SYSTICK interrupt */
    SysTick->LOAD = reload;                             /* Interrupt every 1/delay_ostickspersec seconds */
    SysTick->CTRL |= 1 << 0;                            /* Enable SYSTICK */
#endif 
}

/**
 * @brief     Delay nus microseconds
 * @note      Whether using OS or not, clock extraction method is used for us delay
 * @param     nus: Number of microseconds to delay
 * @note      nus range: 0 ~ (2^32 / fac_us) (fac_us generally equals system main frequency, calculate yourself)
 * @retval    None
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD value */
    ticks = nus * g_fac_us;                 /* Required number of ticks */
    
#if SYS_SUPPORT_OS                          /* If OS support is needed */
    delay_osschedlock();                    /* Lock OS task scheduler */
#endif

    told = SysTick->VAL;                    /* Counter value when entering */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* Note that SYSTICK is a down counter */
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) 
            {
                break;                      /* Time exceeds/equals delay time, exit */
            }
        }
    }

#if SYS_SUPPORT_OS                          /* If OS support is needed */
    delay_osschedunlock();                  /* Restore OS task scheduler */
#endif 
}

/**
 * @brief     Delay nms milliseconds
 * @param     nms: Number of milliseconds to delay (0< nms <= (2^32 / fac_us / 1000))(fac_us generally equals system main frequency, calculate yourself)
 * @retval    None
 */
void delay_ms(uint16_t nms)
{
#if SYS_SUPPORT_OS  /* If OS support is needed, call OS delay according to situation to release CPU */
    if (delay_osrunning && delay_osintnesting == 0)     /* If OS is already running, and not in interrupt (task scheduling not allowed in interrupt) */
    {
        if (nms >= g_fac_ms)                            /* Delay time is greater than OS minimum time period */
        {
            delay_ostimedly(nms / g_fac_ms);            /* OS delay */
        }

        nms %= g_fac_ms;                                /* OS can no longer provide such small delay, use normal delay method */
    }
#endif

    delay_us((uint32_t)(nms * 1000));                   /* Normal delay method */
}

/**
 * @brief       HAL library internal delay function
 * @note        HAL library delay uses SysTick by default, if we don't enable SysTick interrupt, calling this delay will cause infinite loop
 * @param       Delay : Number of milliseconds to delay
 * @retval      None
 */
void HAL_Delay(uint32_t Delay)
{
     delay_ms(Delay);
}

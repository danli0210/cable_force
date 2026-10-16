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

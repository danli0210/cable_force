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

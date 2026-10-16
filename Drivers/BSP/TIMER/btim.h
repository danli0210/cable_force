#ifndef __BTIM_H
#define __BTIM_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* Basic timer definitions */

/* TIMX interrupt definitions
 * Default configuration is for TIM6/TIM7.
 * Note: By modifying these 4 macro definitions, any timer from TIM1~TIM8 can be supported.
 */
 
#define BTIM_TIMX_INT                       TIM6
#define BTIM_TIMX_INT_IRQn                  TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)  /* TIM6 clock enable */

/******************************************************************************************/

void btim_timx_int_init(uint16_t arr, uint16_t psc);    /* Basic timer interrupt initialization function */

#endif


















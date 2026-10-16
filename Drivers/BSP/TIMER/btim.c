#include "./BSP/LED/led.h"
#include "./BSP/TIMER/btim.h"


extern uint8_t g_timeout ;              /* Defined in main.c */

TIM_HandleTypeDef g_timx_handle;         /* Timer configuration handle */


/**
 * @brief       Basic timer TIMX interrupt initialization function
 *    @note
 *              Basic timer clock source is APB1. When the APB1 prescaler is >= 2,
 *              the timer clock is 2x the APB1 clock. APB1 is 42MHz, so timer clock = 84MHz
 *              Timer overflow time calculation: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft = timer working frequency, unit: MHz
 *
 * @param       arr : Auto-reload value
 * @param       psc : Clock prescaler
 * @retval      None
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    g_timx_handle.Instance = BTIM_TIMX_INT;                      /* Basic timer x */
    g_timx_handle.Init.Prescaler = psc;                          /* Set prescaler */
    g_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;         /* Up counter */
    g_timx_handle.Init.Period = arr;                             /* Auto-reload value */
    HAL_TIM_Base_Init(&g_timx_handle);
    
    HAL_TIM_Base_Start_IT(&g_timx_handle);                       /* Enable timer x and its update interrupt: TIM_IT_UPDATE */
}

/**
 * @brief       Timer low-level initialization: enable clock and set interrupt priority
                This function is called by HAL_TIM_Base_Init()
 * @param       None
 * @retval      None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                     /* Enable TIMx clock */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 3); /* Preemption priority 1, sub-priority 3, priority group 2 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);         /* Enable TIM6 interrupt */
    }
}

/**
 * @brief       Basic timer TIMX interrupt service routine
 * @param       None
 * @retval      None
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_handle);  /* Handle timer interrupt and dispatch callbacks */
}

/**
 * @brief       Timer update interrupt callback function
 * @param       None
 * @retval      None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&g_timx_handle))
    {
        g_timeout++;
//        LED1_TOGGLE(); /* Toggle LED1 */
    }
}


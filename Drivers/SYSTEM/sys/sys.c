/**
 ****************************************************************************************************
 * @file        sys.c
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
 * Revision History
 * V1.0 20250601
 * First Release
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"

/**
 * @brief       Set interrupt vector table offset address
 * @param       baseaddr: Base address
 * @param       offset: Offset value
 * @retval      None
 */
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset)
{
    /* Set NVIC vector table offset register, VTOR low 9 bits reserved, i.e. [8:0] reserved */
    SCB->VTOR = baseaddr | (offset & (uint32_t)0xFFFFFE00);
}

/**
 * @brief       Disable all interrupts (except fault and NMI interrupts)
 * @param       None
 * @retval      None
 */
void sys_intx_disable(void)
{
    __ASM volatile("cpsid i");
}

/**
 * @brief       Enable all interrupts
 * @param       None
 * @retval      None
 */
void sys_intx_enable(void)
{
    __ASM volatile("cpsie i");
}

/**
 * @brief       Set stack pointer address
 * @note        Red X on the left is an MDK false alarm, it's actually fine
 * @param       addr: Stack pointer address
 * @retval      None
 */
void sys_msr_msp(uint32_t addr)
{
    __set_MSP(addr);    /* Set stack pointer address */
}

/**
 * @brief       Enter standby mode
 * @param       None
 * @retval      None
 */
void sys_standby(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();    /* Enable power clock */
    SET_BIT(PWR->CR, PWR_CR_PDDS); /* Enter standby mode */
}

/**
 * @brief       System software reset
 * @param       None
 * @retval      None
 */
void sys_soft_reset(void)
{
    NVIC_SystemReset();
}

/**
 * @brief       Clock configuration function
 * @param       plln: Main PLL multiplication factor (PLL multiplier), range: 64~432.
 * @param       pllm: Main PLL and audio PLL pre-divider (divider before PLL), range: 2~63.
 * @param       pllp: Main PLL p divider (divider after PLL), divided output as system clock, range: 2, 4, 6, 8 (only these 4 values).
 * @param       pllq: Main PLL q divider (divider after PLL), range: 2~15.
 * @note
 *
 *              Fvco: VCO frequency
 *              Fsys: System clock frequency, also main PLL p divider output clock frequency
 *              Fq:   Main PLL q divider output clock frequency
 *              Fs:   Main PLL input clock frequency, can be HSI, HSE, etc.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllp = Fs * (plln / (pllm * pllp));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              When external crystal is 8MHz, recommended values: plln = 336, pllm = 8, pllp = 2, pllq = 7.
 *              Results: Fvco = 8 * (336 / 8) = 336MHz
 *                      Fsys = pll_p_ck = 336 / 2 = 168MHz
 *                      Fq   = pll_q_ck = 336 / 7 = 48MHz
 *
 *              F407 default required frequencies:
 *              CPU frequency (HCLK) = pll_p_ck = 168MHz
 *              AHB1/2/3 (rcc_hclk1/2/3) = 168MHz
 *              APB1 (rcc_pclk1) = pll_p_ck / 4 = 42MHz
 *              APB2 (rcc_pclk2) = pll_p_ck / 2 = 84MHz
 *
 * @retval      Error code: 0, success; 1, error;
 */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_OscInitTypeDef rcc_osc_init = {0};
    RCC_ClkInitTypeDef rcc_clk_init = {0};

    __HAL_RCC_PWR_CLK_ENABLE();                                         /* Enable PWR clock */

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);      /* Set regulator output voltage level for operation when device is not at maximum frequency */

    /* Enable HSE, select HSE as PLL clock source, configure PLL1, enable USB clock */
    rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;        /* Clock source is HSE */
    rcc_osc_init.HSEState = RCC_HSE_ON;                          /* Turn on HSE */
    rcc_osc_init.PLL.PLLState = RCC_PLL_ON;                      /* Turn on PLL */
    rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;              /* PLL clock source select HSE */
    rcc_osc_init.PLL.PLLN = plln;
    rcc_osc_init.PLL.PLLM = pllm;
    rcc_osc_init.PLL.PLLP = pllp;
    rcc_osc_init.PLL.PLLQ = pllq;
    ret = HAL_RCC_OscConfig(&rcc_osc_init);                      /* Initialize RCC */
    if(ret != HAL_OK)
    {
        return 1;                                                /* Clock initialization failed, you can add your own handling here */
    }

    /* Select PLL as system clock source and configure HCLK, PCLK1 and PCLK2 */
    rcc_clk_init.ClockType = ( RCC_CLOCKTYPE_SYSCLK \
                                    | RCC_CLOCKTYPE_HCLK \
                                    | RCC_CLOCKTYPE_PCLK1 \
                                    | RCC_CLOCKTYPE_PCLK2);

    rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;         /* Set system clock source to PLL */
    rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;                /* AHB divider factor is 1 */
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV4;                 /* APB1 divider factor is 4 */
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV2;                 /* APB2 divider factor is 2 */
    ret = HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_5);   /* Set FLASH latency cycles to 5WS, which is 6 CPU cycles */
    if(ret != HAL_OK)
    {
        return 1;                                                /* Clock initialization failed */
    }
    
    /* STM32F405x/407x/415x/417x Z version devices support prefetch feature */
    if (HAL_GetREVID() == 0x1001)
    {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();                    /* Enable flash prefetch */
    }
    return 0;
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief       This function is used to report the file name and line number when compilation error occurs
 * @param       file: Pointer to source file
 *              line: Line number in the file
 * @retval      None
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    while (1)
    {
        /* User can add their own implementation to report the file name and line number,
           ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    }
}
#endif

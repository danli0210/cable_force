/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       USART initialization code (typically USART1), supports printf functionality
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
 ****************************************************************************************************
 */

#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"

/* If using OS, include the following header file */
#if SYS_SUPPORT_OS
#include "os.h"                               /* OS usage */
#endif

/******************************************************************************************/
/* Add the following code to support printf function without selecting use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* When using AC6 compiler */
__asm(".global __use_no_semihosting\n\t");          /* Declare not using semihosting mode */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6 requires declaring main function as no-parameter format, otherwise some routines may enter semihosting mode */

#else
/* When using AC5 compiler, define __FILE here and not use semihosting mode */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* Not using semihosting mode, at least need to redefine _ttywrch\_sys_exit\_sys_command_string functions to be compatible with both AC6 and AC5 modes */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* Define _sys_exit() to avoid using semihosting mode */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE is defined in stdio.h */
FILE __stdout;

/* Redefine fputc function, printf function will eventually output characters to USART by calling fputc */
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0);               /* Wait for the previous character to finish sending */

    USART1->DR = (uint8_t)ch;                       /* Write the character ch to be sent into DR register */
    return ch;
}
#endif
/***********************************************END*******************************************/
    
#if USART_EN_RX                                     /* If receive is enabled */

/* Receive buffer, maximum USART_REC_LEN bytes */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  Receive status
 *  bit15:       Receive complete flag
 *  bit14:       Received 0x0d
 *  bit13~0:     Number of valid bytes received
*/
uint16_t g_usart_rx_sta = 0;

uint8_t g_rx_buffer[RXBUFFERSIZE];                  /* USART receive buffer used by HAL library */

UART_HandleTypeDef g_uart1_handle;                  /* UART handle */

/**
 * @brief       USART X initialization function
 * @param       baudrate: Baud rate, set the baud rate value according to your needs
 * @note        Note: Must set the correct clock source, otherwise the USART baud rate will be set abnormally.
 *              The clock source of USART here has been set in the sys_stm32_clock_init() function.
 * @retval      None
 */
void usart_init(uint32_t baudrate)
{
    g_uart1_handle.Instance = USART_UX;                         /* USART1 */
    g_uart1_handle.Init.BaudRate = baudrate;                    /* Baud rate */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* Word length is 8-bit data format */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;             /* One stop bit */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;              /* No parity bit */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* No hardware flow control */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;                 /* Transmit and receive mode */
    HAL_UART_Init(&g_uart1_handle);                             /* HAL_UART_Init() will enable UART1 */
    
    /* This function will enable receive interrupt: flag UART_IT_RXNE, and set receive buffer and maximum data amount for receive buffer */
    HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}

/**
 * @brief       UART low-level initialization function
 * @param       huart: UART handle type pointer
 * @note        This function will be called by HAL_UART_Init()
 *              Complete clock enable, pin configuration, interrupt configuration
 * @retval      None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    if(huart->Instance == USART_UX)                             /* If it's USART1, perform USART1 MSP initialization */
    {
        USART_UX_CLK_ENABLE();                                  /* USART1 clock enable */
        USART_TX_GPIO_CLK_ENABLE();                             /* TX pin clock enable */
        USART_RX_GPIO_CLK_ENABLE();                             /* RX pin clock enable */

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;               /* TX pin */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* Alternate function push-pull output */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* Pull-up */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* High speed */
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;          /* Alternate function as USART1 */
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);   /* Initialize TX pin */

        gpio_init_struct.Pin = USART_RX_GPIO_PIN;               /* RX pin */
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;          /* Alternate function as USART1 */
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);   /* Initialize RX pin */

#if USART_EN_RX
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                      /* Enable USART1 interrupt channel */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);              /* Preemption priority 3, sub-priority 3 */
#endif
    }
}

/**
 * @brief       RX transmission callback function
 * @param       huart: UART handle type pointer
 * @retval      None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART_UX)             /* If it's USART1 */
    {
        if((g_usart_rx_sta & 0x8000) == 0)      /* Reception not complete */
        {
            if(g_usart_rx_sta & 0x4000)         /* Received 0x0d */
            {
                if(g_rx_buffer[0] != 0x0a) 
                {
                    g_usart_rx_sta = 0;         /* Reception error, restart */
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;   /* Reception complete */
                }
            }
            else                                /* Haven't received 0x0D yet */
            {
                if(g_rx_buffer[0] == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;
                }
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0] ;
                    g_usart_rx_sta++;
                    if(g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;     /* Reception data error, restart reception */
                    }
                }
            }
        }
        
        HAL_UART_Receive_IT(&g_uart1_handle, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
    }
}

/**
 * @brief       USART1 interrupt service function
 * @param       None
 * @retval      None
 */
void USART_UX_IRQHandler(void)
{ 
#if SYS_SUPPORT_OS                              /* Using OS */
    OSIntEnter();    
#endif

    HAL_UART_IRQHandler(&g_uart1_handle);       /* Call HAL library interrupt handling common function */

#if SYS_SUPPORT_OS                              /* Using OS */
    OSIntExit();
#endif
}

#endif

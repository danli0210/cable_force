/**
 ****************************************************************************************************
 * @file        usart.h
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
************************************************************************************
 */

#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "./SYSTEM/sys/sys.h"

/*******************************************************************************************************/
/* Pin and USART Definitions
 * Default configuration is for USART1.
 * Note: By modifying these 12 macro definitions, any USART1~UART7 can be supported.
 */

#define USART_TX_GPIO_PORT              GPIOA
#define USART_TX_GPIO_PIN               GPIO_PIN_9
#define USART_TX_GPIO_AF                GPIO_AF7_USART1
#define USART_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* TX pin clock enable */

#define USART_RX_GPIO_PORT              GPIOA
#define USART_RX_GPIO_PIN               GPIO_PIN_10
#define USART_RX_GPIO_AF                GPIO_AF7_USART1
#define USART_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* RX pin clock enable */

#define USART_UX                        USART1
#define USART_UX_IRQn                   USART1_IRQn
#define USART_UX_IRQHandler             USART1_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ __HAL_RCC_USART1_CLK_ENABLE(); }while(0)  /* USART1 clock enable */

/*******************************************************************************************************/

#define USART_REC_LEN   200                     /* Define maximum receive bytes: 200 */
#define USART_EN_RX     1                       /* Enable (1) / Disable (0) USART1 receive */
#define RXBUFFERSIZE    1                       /* Buffer size */

/* External variable declarations */
extern UART_HandleTypeDef g_uart1_handle;       /* UART handle */
extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* Receive buffer, max USART_REC_LEN bytes. Last byte is newline */
extern uint16_t g_usart_rx_sta;                 /* Receive status flag */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL library USART receive buffer */

/* Function declarations */
void usart_init(uint32_t baudrate);             /* USART initialization function */

#endif /* __USART_H */

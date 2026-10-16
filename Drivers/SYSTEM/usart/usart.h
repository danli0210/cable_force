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

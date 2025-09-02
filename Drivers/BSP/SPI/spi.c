/**
 ****************************************************************************************************
 * @file        spi.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       SPI driver code
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

#include "./BSP/SPI/spi.h"


SPI_HandleTypeDef g_spi1_handler; /* SPI1 handle */

/**
 * @brief       SPI initialization code
 *   @note      Master mode, 8-bit data, hardware chip select disabled
 * @param       None
 * @retval      None
 */
void spi1_init(void)
{
    SPI1_SPI_CLK_ENABLE(); /* Enable SPI1 clock */

    g_spi1_handler.Instance = SPI1_SPI;                                /* SPI1 */
    g_spi1_handler.Init.Mode = SPI_MODE_MASTER;                        /* Set SPI working mode to master mode */
    g_spi1_handler.Init.Direction = SPI_DIRECTION_2LINES;              /* Set SPI data mode: SPI set to full duplex mode */
    g_spi1_handler.Init.DataSize = SPI_DATASIZE_8BIT;                  /* Set SPI data size: SPI transmit/receive 8-bit frame structure */
    g_spi1_handler.Init.CLKPolarity = SPI_POLARITY_HIGH;               /* Serial synchronous clock idle state is high level */
    g_spi1_handler.Init.CLKPhase = SPI_PHASE_2EDGE;                    /* Data is sampled on the second transition edge (rising or falling) of serial synchronous clock */
    g_spi1_handler.Init.NSS = SPI_NSS_SOFT;                            /* NSS signal managed by software (using SSI bit) */
    g_spi1_handler.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256; /* Baud rate prescaler value: 256 */
    g_spi1_handler.Init.FirstBit = SPI_FIRSTBIT_MSB;                   /* Data transmission starts from MSB bit */
    g_spi1_handler.Init.TIMode = SPI_TIMODE_DISABLE;                   /* Disable TI mode */
    g_spi1_handler.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;   /* Disable hardware CRC calculation */
    g_spi1_handler.Init.CRCPolynomial = 7;                             /* CRC calculation polynomial */
    HAL_SPI_Init(&g_spi1_handler);                                     /* Initialize SPI1 */

    __HAL_SPI_ENABLE(&g_spi1_handler); /* Enable SPI1 */

    spi1_read_write_byte(0xFF); /* Start transmission, generate 8 clock pulses to clear DR, not mandatory */
}

/**
 * @brief       SPI1 low-level driver, clock enable, pin configuration
 *   @note      This function will be called by HAL_SPI_Init()
 * @param       hspi: SPI handle
 * @retval      None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    GPIO_InitTypeDef GPIO_Initure;
    if (hspi->Instance == SPI1_SPI)
    {
        SPI1_SCK_GPIO_CLK_ENABLE();  /* Enable SPI1_SCK pin clock */
        SPI1_MISO_GPIO_CLK_ENABLE(); /* Enable SPI1_MISO pin clock */
        SPI1_MOSI_GPIO_CLK_ENABLE(); /* Enable SPI1_MOSI pin clock */

        /* SCK pin mode configuration (alternate function output) */
        GPIO_Initure.Pin = SPI1_SCK_GPIO_PIN;
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_Initure.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_Initure);

        /* MISO pin mode configuration (alternate function output) */
        GPIO_Initure.Pin = SPI1_MISO_GPIO_PIN;
        HAL_GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_Initure);

        /* MOSI pin mode configuration (alternate function output) */
        GPIO_Initure.Pin = SPI1_MOSI_GPIO_PIN;
        HAL_GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_Initure);
    }
}

/**
 * @brief       SPI1 speed setting function
 *   @note      SPI1 clock source is APB2, i.e., PCLK2, which is 84MHz
 *              SPI speed = PCLK2 / 2^(speed + 1)
 * @param       speed   : SPI1 clock divider coefficient
 *                       Range: SPI_BAUDRATEPRESCALER_2 ~ SPI_BAUDRATEPRESCALER_256
 * @retval      None
 */
void spi1_set_speed(uint8_t speed)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(speed)); /* Check validity */
    __HAL_SPI_DISABLE(&g_spi1_handler);             /* Disable SPI */
    g_spi1_handler.Instance->CR1 &= 0xFFC7;         /* Clear bits 3-5, used to set baud rate */
    g_spi1_handler.Instance->CR1 |= speed << 3;     /* Set SPI speed */
    __HAL_SPI_ENABLE(&g_spi1_handler);              /* Enable SPI */
}

/**
 * @brief       SPI1 read/write one byte of data
 * @param       txdata  : Data to send (1 byte)
 * @retval      Received data (1 byte)
 */
uint8_t spi1_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata;
    HAL_SPI_TransmitReceive(&g_spi1_handler, &txdata, &rxdata, 1, 1000);
    return rxdata; /* Return received data */
}

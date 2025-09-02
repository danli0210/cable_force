/**
 ****************************************************************************************************
 * @file        sdio_sdcard.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       SD card driver code via SDIO interface
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

#include "string.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/SDIO/sdio_sdcard.h"


SD_HandleTypeDef g_sdcard_handler;            /* SD card handle */
HAL_SD_CardInfoTypeDef g_sd_card_info_handle; /* SD card information structure */

/**
 * @brief       Initialize SD card
 * @param       None
 * @retval      Return value: 0 for successful initialization; other values indicate initialization error
 */
uint8_t sd_init(void)
{
    uint8_t SD_Error;

    /* Clock frequency cannot exceed 400KHz during initialization */
    g_sdcard_handler.Instance = SDIO;
    g_sdcard_handler.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;                       /* Rising edge */
    g_sdcard_handler.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;                  /* Disable bypass mode, use HCLK division to get SDIO_CK */
    g_sdcard_handler.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;           /* Do not turn off clock power when idle */
    g_sdcard_handler.Init.BusWide = SDIO_BUS_WIDE_1B;                               /* 1-bit data line */
    g_sdcard_handler.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE; /* Disable hardware flow control */
    g_sdcard_handler.Init.ClockDiv = SDIO_TRANSF_CLK_DIV;                           /* SD transfer clock frequency max 25MHz */

    SD_Error = HAL_SD_Init(&g_sdcard_handler);
    if (SD_Error != HAL_OK)
    {
        return 1;
    }
    
    HAL_SD_GetCardInfo(&g_sdcard_handler, &g_sd_card_info_handle);                  /* Get SD card information */

    SD_Error = HAL_SD_ConfigWideBusOperation(&g_sdcard_handler, SDIO_BUS_WIDE_4B);  /* Enable 4-bit wide bus mode */
    if (SD_Error != HAL_OK)
    {
        return 2;
    }
    
    return 0;
}

/**
 * @brief       SDIO low-level driver, clock enable, pin configuration
 *              This function will be called by HAL_SD_Init()
 * @param       hsd: SD card handle
 * @retval      None
 */
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
    GPIO_InitTypeDef gpio_init_struct;

    __HAL_RCC_SDIO_CLK_ENABLE();    /* Enable SDIO clock */
    SD_D0_GPIO_CLK_ENABLE();        /* Enable D0 pin GPIO clock */
    SD_D1_GPIO_CLK_ENABLE();        /* Enable D1 pin GPIO clock */
    SD_D2_GPIO_CLK_ENABLE();        /* Enable D2 pin GPIO clock */
    SD_D3_GPIO_CLK_ENABLE();        /* Enable D3 pin GPIO clock */
    SD_CLK_GPIO_CLK_ENABLE();       /* Enable CLK pin GPIO clock */
    SD_CMD_GPIO_CLK_ENABLE();       /* Enable CMD pin GPIO clock */

    gpio_init_struct.Pin = SD_D0_GPIO_PIN;              /* SD_D0 pin mode configuration */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;            /* Push-pull alternate function */
    gpio_init_struct.Pull = GPIO_PULLUP;                /* Pull-up */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;      /* High speed */
    gpio_init_struct.Alternate = GPIO_AF12_SDIO;        /* Alternate function as SDIO */
    HAL_GPIO_Init(SD_D0_GPIO_PORT, &gpio_init_struct);  /* Initialize */

    gpio_init_struct.Pin = SD_D1_GPIO_PIN;              /* SD_D1 pin mode configuration */
    HAL_GPIO_Init(SD_D1_GPIO_PORT, &gpio_init_struct);  /* Initialize */

    gpio_init_struct.Pin = SD_D2_GPIO_PIN;              /* SD_D2 pin mode configuration */
    HAL_GPIO_Init(SD_D2_GPIO_PORT, &gpio_init_struct);  /* Initialize */

    gpio_init_struct.Pin = SD_D3_GPIO_PIN;              /* SD_D3 pin mode configuration */
    HAL_GPIO_Init(SD_D3_GPIO_PORT, &gpio_init_struct);  /* Initialize */

    gpio_init_struct.Pin = SD_CLK_GPIO_PIN;             /* SD_CLK pin mode configuration */
    HAL_GPIO_Init(SD_CLK_GPIO_PORT, &gpio_init_struct); /* Initialize */

    gpio_init_struct.Pin = SD_CMD_GPIO_PIN;             /* SD_CMD pin mode configuration */
    HAL_GPIO_Init(SD_CMD_GPIO_PORT, &gpio_init_struct); /* Initialize */
}

/**
 * @brief       Get card information function
 * @param       cardinfo: SD card information handle
 * @retval      Return value: Status of reading card information
 */
uint8_t get_sd_card_info(HAL_SD_CardInfoTypeDef *cardinfo)
{
    uint8_t sta;
    
    sta = HAL_SD_GetCardInfo(&g_sdcard_handler, cardinfo);
    
    return sta;
}

/**
 * @brief       Check if SD card can transfer (read/write) data
 * @param       None
 * @retval      Return value: SD_TRANSFER_OK      Transfer complete, ready for next transfer
 *                           SD_TRANSFER_BUSY    SD card is busy, cannot proceed with next transfer
 */
uint8_t get_sd_card_state(void)
{
    return ((HAL_SD_GetCardState(&g_sdcard_handler) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

/**
 * @brief       Read SD card (called by fatfs/usb)
 * @param       pbuf  : Data buffer
 * @param       saddr : Sector address
 * @param       cnt   : Number of sectors
 * @retval      0: Normal; Other: Error code (see SD_Error definition)
 */
uint8_t sd_read_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint8_t sta = HAL_OK;
    uint32_t timeout = SD_TIMEOUT;
    long long lsector = saddr;
    
    __disable_irq();                                                                       /* Disable global interrupts (POLLING mode, interrupts must not interfere with SDIO read/write operations!) */
    sta = HAL_SD_ReadBlocks(&g_sdcard_handler, (uint8_t *)pbuf, lsector, cnt, SD_TIMEOUT); /* Multi-sector read operation */

    /* Wait for SD card read completion */
    while (get_sd_card_state() != SD_TRANSFER_OK)
    {
        if (timeout-- == 0)
        {
            sta = SD_TRANSFER_BUSY;
        }
    }
    __enable_irq(); /* Enable global interrupts */
    
    return sta;
}

/**
 * @brief       Write SD card (called by fatfs/usb)
 * @param       pbuf  : Data buffer
 * @param       saddr : Sector address
 * @param       cnt   : Number of sectors
 * @retval      0: Normal; Other: Error code (see SD_Error definition)
 */
uint8_t sd_write_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt)
{
    uint8_t sta = HAL_OK;
    uint32_t timeout = SD_TIMEOUT;
    long long lsector = saddr;
    
    __disable_irq();                                                                        /* Disable global interrupts (POLLING mode, interrupts must not interfere with SDIO read/write operations!) */
    sta = HAL_SD_WriteBlocks(&g_sdcard_handler, (uint8_t *)pbuf, lsector, cnt, SD_TIMEOUT); /* Multi-sector write operation */

    /* Wait for SD card write completion */
    while (get_sd_card_state() != SD_TRANSFER_OK)
    {
        if (timeout-- == 0)
        {
            sta = SD_TRANSFER_BUSY;
        }
    }
    __enable_irq();     /* Enable global interrupts */
    
    return sta;
}

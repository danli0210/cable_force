/**
 ****************************************************************************************************
 * @file        sdio_sdcard.h
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

#ifndef __SDIO_SDCARD_H
#define __SDIO_SDCARD_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* SDIO signal pins: SD_D0 ~ SD_D3/SD_CLK/SD_CMD pin definitions
 * If you use different pins for SDIO signals, modify these definitions accordingly.
 */

#define SD_D0_GPIO_PORT                GPIOC
#define SD_D0_GPIO_PIN                 GPIO_PIN_8
#define SD_D0_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)    /* GPIO port clock enable */

#define SD_D1_GPIO_PORT                GPIOC
#define SD_D1_GPIO_PIN                 GPIO_PIN_9
#define SD_D1_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)    /* GPIO port clock enable */

#define SD_D2_GPIO_PORT                GPIOC
#define SD_D2_GPIO_PIN                 GPIO_PIN_10
#define SD_D2_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)    /* GPIO port clock enable */

#define SD_D3_GPIO_PORT                GPIOC
#define SD_D3_GPIO_PIN                 GPIO_PIN_11
#define SD_D3_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)    /* GPIO port clock enable */

#define SD_CLK_GPIO_PORT               GPIOC
#define SD_CLK_GPIO_PIN                GPIO_PIN_12
#define SD_CLK_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)    /* GPIO port clock enable */

#define SD_CMD_GPIO_PORT               GPIOD
#define SD_CMD_GPIO_PIN                GPIO_PIN_2
#define SD_CMD_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* GPIO port clock enable */

/******************************************************************************************/

#define SD_TIMEOUT             ((uint32_t)100000000)    /* Timeout value */
#define SD_TRANSFER_OK         ((uint8_t)0x00)
#define SD_TRANSFER_BUSY       ((uint8_t)0x01)

/* Macros for quick capacity calculation based on SD_HandleTypeDef */
#define SD_TOTAL_SIZE_BYTE(__Handle__)  (((uint64_t)((__Handle__)->SdCard.LogBlockNbr) * ((__Handle__)->SdCard.LogBlockSize)) >> 0)
#define SD_TOTAL_SIZE_KB(__Handle__)    (((uint64_t)((__Handle__)->SdCard.LogBlockNbr) * ((__Handle__)->SdCard.LogBlockSize)) >> 10)
#define SD_TOTAL_SIZE_MB(__Handle__)    (((uint64_t)((__Handle__)->SdCard.LogBlockNbr) * ((__Handle__)->SdCard.LogBlockSize)) >> 20)
#define SD_TOTAL_SIZE_GB(__Handle__)    (((uint64_t)((__Handle__)->SdCard.LogBlockNbr) * ((__Handle__)->SdCard.LogBlockSize)) >> 30)

/*  
 *  SD transfer clock divider. Due to low HAL library efficiency, overflow (SD read) / 
 *  underflow (SD write) errors can easily occur.
 *  For 4-bit mode: Set this macro to 1, SDIO clock frequency: 48/(SDIO_TRANSF_CLK_DIV + 2) = 16M * 4bit = 64Mbps
 *  For 1-bit mode: Set this macro to 0, SDIO clock frequency: 48/(SDIO_TRANSF_CLK_DIV + 2) = 24M * 1bit = 24Mbps
 */
#define  SDIO_TRANSF_CLK_DIV        1   
/******************************************************************************************/

extern SD_HandleTypeDef        g_sdcard_handler;        /* SD card handle */
extern HAL_SD_CardInfoTypeDef  g_sd_card_info_handle;   /* SD card info structure */

/* Function declarations */
uint8_t sd_init(void);                                              /* Initialize SD card */
uint8_t get_sd_card_info(HAL_SD_CardInfoTypeDef *cardinfo);         /* Get card information */
uint8_t get_sd_card_state(void);                                    /* Get card state */
uint8_t sd_read_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt);  /* Read from SD card */
uint8_t sd_write_disk(uint8_t *pbuf, uint32_t saddr, uint32_t cnt); /* Write to SD card */

#endif

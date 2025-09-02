/**
 ****************************************************************************************************
 * @file        sram.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       External SRAM driver code
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

#ifndef __SRAM_H
#define __SRAM_H
#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* SRAM WR/RD/CS pin definitions 
 * SRAM_D0~D15 and address lines are not defined here due to too many pins. They are 
 * directly modified in sram_init(). When porting, besides changing these 3 IO pins, 
 * you also need to modify the data lines and address lines IO pins in sram_init().
 */

#define SRAM_WR_GPIO_PORT               GPIOD
#define SRAM_WR_GPIO_PIN                GPIO_PIN_5
#define SRAM_WR_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* GPIO port clock enable */

#define SRAM_RD_GPIO_PORT               GPIOD
#define SRAM_RD_GPIO_PIN                GPIO_PIN_4
#define SRAM_RD_GPIO_CLK_ENABLE()       do{ __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)   /* GPIO port clock enable */

/* SRAM_CS (must be set according to SRAM_FSMC_NEX) pin definition */
#define SRAM_CS_GPIO_PORT                GPIOG
#define SRAM_CS_GPIO_PIN                 GPIO_PIN_10
#define SRAM_CS_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOG_CLK_ENABLE(); }while(0)   /* GPIO port clock enable */

/* FSMC related parameter definitions 
 * Note: We default to connecting SRAM through FSMC Bank1 Block 3, Bank1 has 4 chip selects: FSMC_NE1~4
 *
 * When modifying SRAM_FSMC_NEX, the corresponding SRAM_CS_GPIO related settings must also be changed
 */
#define SRAM_FSMC_NEX           3         /* Use FSMC_NE3 to connect SRAM_CS, valid range: 1~4 */
#define SRAM_FSMC_BCRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2]       /* BCR register, auto-calculated based on SRAM_FSMC_NEX */
#define SRAM_FSMC_BTRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2 + 1]   /* BTR register, auto-calculated based on SRAM_FSMC_NEX */
#define SRAM_FSMC_BWTRX         FSMC_Bank1E->BWTR[(SRAM_FSMC_NEX - 1) * 2]      /* BWTR register, auto-calculated based on SRAM_FSMC_NEX */

/******************************************************************************************/
/* SRAM base address, determined by SRAM_FSMC_NEX setting
 * We generally use FSMC Bank1 to drive SRAM, Bank1 address range total size is 256MB, divided into 4 blocks:
 * Memory Block 1 (FSMC_NE1) address range: 0x6000_0000 ~ 0x63FF_FFFF
 * Memory Block 2 (FSMC_NE2) address range: 0x6400_0000 ~ 0x67FF_FFFF
 * Memory Block 3 (FSMC_NE3) address range: 0x6800_0000 ~ 0x6BFF_FFFF
 * Memory Block 4 (FSMC_NE4) address range: 0x6C00_0000 ~ 0x6FFF_FFFF
 */
#define SRAM_BASE_ADDR         (0x60000000 + (0x4000000 * (SRAM_FSMC_NEX - 1)))

extern SRAM_HandleTypeDef g_sram_handler;    /* SRAM handle */

/* Function declarations */
void sram_init(void);                                                   /* Initialize SRAM */
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen);       /* Write data to SRAM */
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen);        /* Read data from SRAM */
uint8_t sram_test_read(uint32_t addr);                                  /* Test read single byte */
void sram_test_write(uint32_t addr, uint8_t data);                     /* Test write single byte */

#endif

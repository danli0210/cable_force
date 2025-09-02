/**
 ****************************************************************************************************
 * @file        norflash.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       NOR FLASH (25QXX series) driver code
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

#ifndef __NORFLASH_H
#define __NORFLASH_H

#include "./SYSTEM/sys/sys.h"


/******************************************************************************************/
/* NORFLASH chip select pin definitions */

#define NORFLASH_CS_GPIO_PORT           GPIOB
#define NORFLASH_CS_GPIO_PIN            GPIO_PIN_14
#define NORFLASH_CS_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* GPIO port clock enable */

/******************************************************************************************/

/* NORFLASH chip select signal */
#define NORFLASH_CS(x)      do{ x ? \
                                  HAL_GPIO_WritePin(NORFLASH_CS_GPIO_PORT, NORFLASH_CS_GPIO_PIN, GPIO_PIN_SET) : \
                                  HAL_GPIO_WritePin(NORFLASH_CS_GPIO_PORT, NORFLASH_CS_GPIO_PIN, GPIO_PIN_RESET); \
                            }while(0)

/* Supported FLASH chip list */
#define W25Q80      0xEF13          /* W25Q80   chip ID */
#define W25Q16      0xEF14          /* W25Q16   chip ID */
#define W25Q32      0xEF15          /* W25Q32   chip ID */
#define W25Q64      0xEF16          /* W25Q64   chip ID */
#define W25Q128     0xEF17          /* W25Q128  chip ID */
#define W25Q256     0xEF18          /* W25Q256  chip ID */
#define BY25Q64     0x6816          /* BY25Q64  chip ID */
#define BY25Q128    0x6817          /* BY25Q128 chip ID */
#define NM25Q64     0x5216          /* NM25Q64  chip ID */
#define NM25Q128    0x5217          /* NM25Q128 chip ID */

/* Command instruction table */
#define FLASH_WriteEnable           0x06    /* Write enable command */
#define FLASH_WriteDisable          0x04    /* Write disable command */
#define FLASH_ReadStatusReg1        0x05    /* Read status register 1 */
#define FLASH_ReadStatusReg2        0x35    /* Read status register 2 */
#define FLASH_ReadStatusReg3        0x15    /* Read status register 3 */
#define FLASH_WriteStatusReg1       0x01    /* Write status register 1 */
#define FLASH_WriteStatusReg2       0x31    /* Write status register 2 */
#define FLASH_WriteStatusReg3       0x11    /* Write status register 3 */
#define FLASH_ReadData              0x03    /* Read data command */
#define FLASH_FastReadData          0x0B    /* Fast read data command */
#define FLASH_FastReadDual          0x3B    /* Fast read dual command */
#define FLASH_FastReadQuad          0xEB    /* Fast read quad command */
#define FLASH_PageProgram           0x02    /* Page program command */
#define FLASH_PageProgramQuad       0x32    /* Quad page program command */
#define FLASH_BlockErase            0xD8    /* Block erase command */
#define FLASH_SectorErase           0x20    /* Sector erase command */
#define FLASH_ChipErase             0xC7    /* Chip erase command */
#define FLASH_PowerDown             0xB9    /* Power down command */
#define FLASH_ReleasePowerDown      0xAB    /* Release power down command */
#define FLASH_DeviceID              0xAB    /* Device ID command */
#define FLASH_ManufactDeviceID      0x90    /* Manufacturer device ID command */
#define FLASH_JedecDeviceID         0x9F    /* JEDEC device ID command */
#define FLASH_Enable4ByteAddr       0xB7    /* Enable 4-byte address mode */
#define FLASH_Exit4ByteAddr         0xE9    /* Exit 4-byte address mode */
#define FLASH_SetReadParam          0xC0    /* Set read parameters */
#define FLASH_EnterQPIMode          0x38    /* Enter QPI mode */
#define FLASH_ExitQPIMode           0xFF    /* Exit QPI mode */


extern uint16_t norflash_TYPE;      /* Define FLASH chip model */

/* Static function declarations */
static void norflash_wait_busy(void);               /* Wait for device ready */
static void norflash_send_address(uint32_t address);/* Send address */
static void norflash_write_page(uint8_t *pbuf, uint32_t addr, uint16_t datalen);    /* Write page */
static void norflash_write_nocheck(uint8_t *pbuf, uint32_t addr, uint16_t datalen); /* Write flash without erase */

/* Public function declarations */
void norflash_init(void);                   /* Initialize 25QXX flash */
uint16_t norflash_read_id(void);            /* Read FLASH ID */
void norflash_write_enable(void);           /* Write enable */
uint8_t norflash_read_sr(uint8_t regno);    /* Read status register */
void norflash_write_sr(uint8_t regno, uint8_t sr);   /* Write status register */

void norflash_erase_chip(void);             /* Chip erase */
void norflash_erase_sector(uint32_t saddr); /* Sector erase */
void norflash_read(uint8_t *pbuf, uint32_t addr, uint16_t datalen);     /* Read from flash */
void norflash_write(uint8_t *pbuf, uint32_t addr, uint16_t datalen);    /* Write to flash */

#endif

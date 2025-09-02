/**
 ****************************************************************************************************
 * @file        diskio.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       FATFS low-level (diskio) driver code
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

#include "./MALLOC/malloc.h"
#include "./SYSTEM/usart/usart.h"
#include "./FATFS/source/diskio.h"
#include "./BSP/SDIO/sdio_sdcard.h"
#include "./BSP/NORFLASH/norflash.h"


#define SD_CARD     0       /* SD card, volume label 0 */
#define EX_FLASH    1       /* External SPI flash, volume label 1 */

/**
 * For 25Q128 FLASH chip, we allocate the first 12MB for FATFS use, after 12MB
 * follows the font library, 3 font libraries + UNIGBK.BIN, total size 3.09MB, occupying 15.09MB total
 * Storage space after 15.09MB can be used freely.
 */

#define SPI_FLASH_SECTOR_SIZE   512
#define SPI_FLASH_SECTOR_COUNT  12 * 1024 * 2   /* 25Q128, first 12MB for FATFS use */
#define SPI_FLASH_BLOCK_SIZE    8               /* Each BLOCK has 8 sectors */
#define SPI_FLASH_FATFS_BASE    0               /* FATFS start address in external FLASH, starts from 0 */


/**
 * @brief       Get disk status
 * @param       pdrv : Physical drive number 0~9
 * @retval      Execution result (see FATFS, DSTATUS definition)
 */
DSTATUS disk_status (
    BYTE pdrv       /* Physical drive number to identify the drive */
)
{
    return RES_OK;
}

/**
 * @brief       Initialize disk
 * @param       pdrv : Physical drive number 0~9
 * @retval      Execution result (see FATFS, DSTATUS definition)
 */
DSTATUS disk_initialize (
    BYTE pdrv       /* Physical drive number to identify the drive */
)
{
    uint8_t res = 0;

    switch (pdrv)
    {
        case SD_CARD:           /* SD card */
            res = sd_init();    /* SD card initialization */
            break;

        case EX_FLASH:          /* External flash */
            norflash_init(); 
            break;

        default:
            res = 1;
            break;
    }

    if (res)
    {
        return STA_NOINIT;
    }
    else
    {
        return 0; /* Initialization successful */
    }
}

/**
 * @brief       Read sectors
 * @param       pdrv   : Physical drive number 0~9
 * @param       buff   : Data buffer to store read data
 * @param       sector : Sector address
 * @param       count  : Number of sectors to read
 * @retval      Execution result (see FATFS, DRESULT definition)
 */
DRESULT disk_read (
    BYTE pdrv,      /* Physical drive number to identify the drive */
    BYTE *buff,     /* Data buffer to store read data */
    DWORD sector,   /* Sector address in LBA */
    UINT count      /* Number of sectors to read */
)
{
    uint8_t res = 0;

    if (!count) return RES_PARERR;   /* count cannot be 0, otherwise return parameter error */

    switch (pdrv)
    {
        case SD_CARD:       /* SD card */
            res = sd_read_disk(buff, sector, count);

            while (res)     /* Read error */
            {
                //printf("sd rd error:%d\r\n", res);
                sd_init();  /* Reinitialize SD card */
                res = sd_read_disk(buff, sector, count);
            }

            break;

        case EX_FLASH:      /* External flash */
            for (; count > 0; count--)
            {
                norflash_read(buff, SPI_FLASH_FATFS_BASE + sector * SPI_FLASH_SECTOR_SIZE, SPI_FLASH_SECTOR_SIZE);
                sector++;
                buff += SPI_FLASH_SECTOR_SIZE;
            }

            res = 0;
            break;

        default:
            res = 1;
    }

    /* Process return value, convert to ff.c return value */
    if (res == 0x00)
    {
        return RES_OK;
    }
    else
    {
        return RES_ERROR; 
    }
}

/**
 * @brief       Write sectors
 * @param       pdrv   : Physical drive number 0~9
 * @param       buff   : Data buffer to be written
 * @param       sector : Sector address
 * @param       count  : Number of sectors to write
 * @retval      Execution result (see FATFS, DRESULT definition)
 */
DRESULT disk_write (
    BYTE pdrv,          /* Physical drive number to identify the drive */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Sector address in LBA */
    UINT count          /* Number of sectors to write */
)
{
    uint8_t res = 0;

    if (!count) return RES_PARERR;  /* count cannot be 0, otherwise return parameter error */

    switch (pdrv)
    {
        case SD_CARD:       /* SD card */
            res = sd_write_disk((uint8_t *)buff, sector, count);

            while (res)     /* Write error */
            {
                //printf("sd wr error:%d\r\n", res);
                sd_init();  /* Reinitialize SD card */
                res = sd_write_disk((uint8_t *)buff, sector, count);
            }

            break;

        case EX_FLASH:      /* External flash */
            for (; count > 0; count--)
            {
                norflash_write((uint8_t *)buff, SPI_FLASH_FATFS_BASE + sector * SPI_FLASH_SECTOR_SIZE, SPI_FLASH_SECTOR_SIZE);
                sector++;
                buff += SPI_FLASH_SECTOR_SIZE;
            }

            res = 0;
            break;

        default:
            res = 1;
    }

    /* Process return value, convert to ff.c return value */
    if (res == 0x00)
    {
        return RES_OK;
    }
    else
    {
        return RES_ERROR; 
    }
}

/**
 * @brief       Get other control parameters
 * @param       pdrv   : Physical drive number 0~9
 * @param       cmd    : Control code
 * @param       buff   : Buffer to send/receive control data
 * @retval      Execution result (see FATFS, DRESULT definition)
 */
DRESULT disk_ioctl (
    BYTE pdrv,      /* Physical drive number (0..) */
    BYTE cmd,       /* Control code */
    void *buff      /* Buffer to send/receive control data */
)
{
    DRESULT res;

    if (pdrv == SD_CARD)    /* SD card */
    {
        switch (cmd)
        {
            case CTRL_SYNC:
                res = RES_OK;
                break;

            case GET_SECTOR_SIZE:
                *(DWORD *)buff = 512;
                res = RES_OK;
                break;

            case GET_BLOCK_SIZE:
                *(WORD *)buff = g_sd_card_info_handle.LogBlockSize;
                res = RES_OK;
                break;

            case GET_SECTOR_COUNT:
                *(DWORD *)buff = g_sd_card_info_handle.LogBlockNbr;
                res = RES_OK;
                break;

            default:
                res = RES_PARERR;
                break;
        }
    }
    else if (pdrv == EX_FLASH)  /* External FLASH */
    {
        switch (cmd)
        {
            case CTRL_SYNC:
                res = RES_OK;
                break;

            case GET_SECTOR_SIZE:
                *(WORD *)buff = SPI_FLASH_SECTOR_SIZE;
                res = RES_OK;
                break;

            case GET_BLOCK_SIZE:
                *(WORD *)buff = SPI_FLASH_BLOCK_SIZE;
                res = RES_OK;
                break;

            case GET_SECTOR_COUNT:
                *(DWORD *)buff = SPI_FLASH_SECTOR_COUNT;
                res = RES_OK;
                break;

            default:
                res = RES_PARERR;
                break;
        }
    }
    else
    {
        res = RES_ERROR;    /* Others not supported */
    }
    
    return res;
}

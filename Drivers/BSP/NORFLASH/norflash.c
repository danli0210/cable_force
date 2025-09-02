/**
 ****************************************************************************************************
 * @file        norflash.c
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

#include "./BSP/SPI/spi.h"
#include "./SYSTEM/delay/delay.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/NORFLASH/norflash.h"


uint16_t g_norflash_type = NM25Q128;     /* Default is NM25Q128 */

/**
 * @brief       Initialize SPI NOR FLASH
 * @param       None
 * @retval      None
 */
void norflash_init(void)
{
    uint8_t temp;

    NORFLASH_CS_GPIO_CLK_ENABLE();      /* Enable NORFLASH CS pin clock */

    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = NORFLASH_CS_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NORFLASH_CS_GPIO_PORT, &gpio_init_struct); /* CS pin mode configuration (alternate output) */

    NORFLASH_CS(1);                         /* Deselect chip */

    spi1_init();                            /* Initialize SPI1 */
    spi1_set_speed(SPI_SPEED_4);            /* Switch SPI1 to high speed state 21MHz */
    
    g_norflash_type = norflash_read_id();   /* Read FLASH ID */
    
    if (g_norflash_type == W25Q256)         /* SPI FLASH is W25Q256, must enable 4-byte address mode */
    {
        temp = norflash_read_sr(3);         /* Read status register 3, check address mode */

        if ((temp & 0x01) == 0)             /* If not in 4-byte address mode, enter 4-byte address mode */
        {
            norflash_write_enable();        /* Write enable */
            temp |= 1 << 1;                 /* ADP=1, power-on 4-byte address mode */
            norflash_write_sr(3, temp);     /* Write SR3 */
            
            NORFLASH_CS(0);
            spi1_read_write_byte(FLASH_Enable4ByteAddr);    /* Enable 4-byte address instruction */
            NORFLASH_CS(1);
        }
    }

    //printf("ID:%x\r\n", g_norflash_type);
}

/**
 * @brief       Wait for device ready
 * @param       None
 * @retval      None
 */
static void norflash_wait_busy(void)
{
    while ((norflash_read_sr(1) & 0x01) == 0x01);   /* Wait for BUSY bit to clear */
}

/**
 * @brief       25QXX write enable
 *   @note      Set WEL bit in S1 register
 * @param       None
 * @retval      None
 */
void norflash_write_enable(void)
{
    NORFLASH_CS(0);
    spi1_read_write_byte(FLASH_WriteEnable);   /* Send write enable command */
    NORFLASH_CS(1);
}

/**
 * @brief       25QXX send address
 *   @note      Send 24-bit/32-bit address depending on chip type
 * @param       address : Address to send
 * @retval      None
 */
static void norflash_send_address(uint32_t address)
{
    if (g_norflash_type == W25Q256)                     /* Only W25Q256 supports 4-byte address mode */
    {
        spi1_read_write_byte((uint8_t)((address)>>24)); /* Send bit31 ~ bit24 address */
    } 
    spi1_read_write_byte((uint8_t)((address)>>16));     /* Send bit23 ~ bit16 address */
    spi1_read_write_byte((uint8_t)((address)>>8));      /* Send bit15 ~ bit8  address */
    spi1_read_write_byte((uint8_t)address);             /* Send bit7  ~ bit0  address */
}

/**
 * @brief       Read status register of 25QXX, 25QXX has 3 status registers
 *   @note      Status Register 1:
 *              BIT7  6   5   4   3   2   1   0
 *              SPR   RV  TB BP2 BP1 BP0 WEL BUSY
 *              SPR:Default 0, status register protection bit, used with WP
 *              TB,BP2,BP1,BP0:FLASH region write protection settings
 *              WEL:Write enable latch
 *              BUSY:Busy flag bit (1=busy; 0=idle)
 *              Default:0x00
 *
 *              Status Register 2:
 *              BIT7  6   5   4   3   2   1   0
 *              SUS   CMP LB3 LB2 LB1 (R) QE  SRP1
 *
 *              Status Register 3:
 *              BIT7      6    5    4   3   2   1   0
 *              HOLD/RST  DRV1 DRV0 (R) (R) WPS ADP ADS
 *
 * @param       regno: Status register number, range: 1~3
 * @retval      Status register value
 */
uint8_t norflash_read_sr(uint8_t regno)
{
    uint8_t byte = 0, command = 0;

    switch (regno)
    {
        case 1:
            command = FLASH_ReadStatusReg1;  /* Read status register 1 command */
            break;

        case 2:
            command = FLASH_ReadStatusReg2;  /* Read status register 2 command */
            break;

        case 3:
            command = FLASH_ReadStatusReg3;  /* Read status register 3 command */
            break;

        default:
            command = FLASH_ReadStatusReg1;
            break;
    }

    NORFLASH_CS(0);
    spi1_read_write_byte(command);      /* Send read register command */
    byte = spi1_read_write_byte(0xFF);  /* Read one byte */
    NORFLASH_CS(1);
    
    return byte;
}

/**
 * @brief       Write 25QXX status register
 *   @note      Register description see norflash_read_sr function description
 * @param       regno: Status register number, range: 1~3
 * @param       sr   : Value to write to status register
 * @retval      None
 */
void norflash_write_sr(uint8_t regno, uint8_t sr)
{
    uint8_t command = 0;

    switch (regno)
    {
        case 1:
            command = FLASH_WriteStatusReg1;  /* Write status register 1 command */
            break;

        case 2:
            command = FLASH_WriteStatusReg2;  /* Write status register 2 command */
            break;

        case 3:
            command = FLASH_WriteStatusReg3;  /* Write status register 3 command */
            break;

        default:
            command = FLASH_WriteStatusReg1;
            break;
    }

    NORFLASH_CS(0);
    spi1_read_write_byte(command);  /* Send write register command */
    spi1_read_write_byte(sr);       /* Write one byte */
    NORFLASH_CS(1);
}

/**
 * @brief       Read chip ID
 * @param       None
 * @retval      FLASH chip ID
 *   @note      Chip ID list see: norflash.h, chip list section
 */
uint16_t norflash_read_id(void)
{
    uint16_t deviceid;

    NORFLASH_CS(0);
    spi1_read_write_byte(FLASH_ManufactDeviceID);   /* Send read ID command */
    spi1_read_write_byte(0);                        /* Write one byte */
    spi1_read_write_byte(0);
    spi1_read_write_byte(0);
    deviceid = spi1_read_write_byte(0xFF) << 8;     /* Read high 8 bits */
    deviceid |= spi1_read_write_byte(0xFF);         /* Read low 8 bits */
    NORFLASH_CS(1);

    return deviceid;
}

/**
 * @brief       Read SPI FLASH
 *   @note      Start reading specified length of data from specified address
 * @param       pbuf    : Data buffer
 * @param       addr    : Starting read address (max 32-bit)
 * @param       datalen : Number of bytes to read (max 65535)
 * @retval      None
 */
void norflash_read(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t i;

    NORFLASH_CS(0);
    spi1_read_write_byte(FLASH_ReadData);       /* Send read command */
    norflash_send_address(addr);                /* Send address */
    
    for (i = 0; i < datalen; i++)
    {
        pbuf[i] = spi1_read_write_byte(0xFF);   /* Read data in loop */
    }
    
    NORFLASH_CS(1);
}

/**
 * @brief       Write less than 256 bytes of data within one page (0~65535) via SPI
 *   @note      Start writing maximum 256 bytes of data from specified address
 * @param       pbuf    : Data buffer
 * @param       addr    : Starting write address (max 32-bit)
 * @param       datalen : Number of bytes to write (max 256), should not exceed remaining bytes in the page!!!
 * @retval      None
 */
static void norflash_write_page(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t i;

    norflash_write_enable();                    /* Write enable */

    NORFLASH_CS(0);
    spi1_read_write_byte(FLASH_PageProgram);    /* Send page program command */
    norflash_send_address(addr);                /* Send address */

    for (i = 0; i < datalen; i++)
    {
        spi1_read_write_byte(pbuf[i]);          /* Write data in loop */
    }
    
    NORFLASH_CS(1);
    norflash_wait_busy();       /* Wait for write completion */
}

/**
 * @brief       Write SPI FLASH without verification
 *   @note      Must ensure all data in the address range to be written is 0xFF, otherwise data written at non-0xFF locations will fail!
 *              Has automatic page wrap feature
 *              Start writing specified length of data from specified address, but ensure address doesn't exceed bounds!
 *
 * @param       pbuf    : Data buffer
 * @param       addr    : Starting write address (max 32-bit)
 * @param       datalen : Number of bytes to write (max 65535)
 * @retval      None
 */
static void norflash_write_nocheck(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t pageremain;
    pageremain = 256 - addr % 256;  /* Remaining bytes in single page */

    if (datalen <= pageremain)      /* No more than 256 bytes */
    {
        pageremain = datalen;
    }

    while (1)
    {
        /* When bytes to write is less than remaining page addresses, write all at once
         * When bytes to write is more than remaining page addresses, write remaining page first, then handle remaining length differently
         */
        norflash_write_page(pbuf, addr, pageremain);

        if (datalen == pageremain)      /* Write complete */
        {
            break;
        }
        else                            /* datalen > pageremain */
        {
            pbuf += pageremain;         /* pbuf pointer offset, pageremain bytes already written */
            addr += pageremain;         /* Write address offset, pageremain bytes already written */
            datalen -= pageremain;      /* Total write length minus bytes already written */

            if (datalen > 256)          /* Remaining data still larger than one page, can write one page at once */
            {
                pageremain = 256;       /* Can write 256 bytes at once */
            }
            else                        /* Remaining data less than one page, can write all at once */
            {
                pageremain = datalen;   /* Less than 256 bytes remaining */
            }
        }
    }
}

/**
 * @brief       Write SPI FLASH
 *   @note      Start writing specified length of data from specified address, this function includes erase operation!
 *              SPI FLASH generally: 256 bytes per Page, 4Kbytes per Sector, 16 sectors per Block
 *              Minimum erase unit is Sector.
 *
 * @param       pbuf    : Data buffer
 * @param       addr    : Starting write address (max 32-bit)
 * @param       datalen : Number of bytes to write (max 65535)
 * @retval      None
 */
uint8_t g_norflash_buf[4096];   /* Sector buffer */

void norflash_write(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t *norflash_buf;

    norflash_buf = g_norflash_buf;
    secpos = addr / 4096;       /* Sector address */
    secoff = addr % 4096;       /* Offset within sector */
    secremain = 4096 - secoff;  /* Remaining space in sector */

    //printf("ad:%X,nb:%X\r\n", addr, datalen); /* For testing */
    if (datalen <= secremain)
    {
        secremain = datalen;    /* No more than 4096 bytes */
    }

    while (1)
    {
        norflash_read(norflash_buf, secpos * 4096, 4096);   /* Read entire sector content */

        for (i = 0; i < secremain; i++)     /* Verify data */
        {
            if (norflash_buf[secoff + i] != 0xFF)
            {
                break;                      /* Need erase, exit for loop directly */
            }
        }

        if (i < secremain)                  /* Need erase */
        {
            norflash_erase_sector(secpos);  /* Erase this sector */

            for (i = 0; i < secremain; i++) /* Copy data */
            {
                norflash_buf[i + secoff] = pbuf[i];
            }

            norflash_write_nocheck(norflash_buf, secpos * 4096, 4096);  /* Write entire sector */
        }
        else    /* Already erased, directly write to remaining sector space */
        {
            norflash_write_nocheck(pbuf, addr, secremain);              /* Write sector directly */
        }

        if (datalen == secremain)
        {
            break;  /* Write complete */
        }
        else        /* Write not complete */
        {
            secpos++;               /* Sector address increment by 1 */
            secoff = 0;             /* Offset position is 0 */

            pbuf += secremain;      /* Pointer offset */
            addr += secremain;      /* Write address offset */
            datalen -= secremain;   /* Byte count decrement */

            if (datalen > 4096)
            {
                secremain = 4096;   /* Next sector still cannot be written completely */
            }
            else
            {
                secremain = datalen;/* Next sector can be written completely */
            }
        }
    }
}

/**
 * @brief       Erase entire chip
 *   @note      Wait time is very long...
 * @param       None
 * @retval      None
 */
void norflash_erase_chip(void)
{
    norflash_write_enable();    /* Write enable */
    norflash_wait_busy();       /* Wait for ready */
    NORFLASH_CS(0);
    spi1_read_write_byte(FLASH_ChipErase);  /* Send chip erase command */ 
    NORFLASH_CS(1);
    norflash_wait_busy();       /* Wait for chip erase completion */
}

/**
 * @brief       Erase one sector
 *   @note      Note: This is sector address, not byte address!!
 *              Minimum time to erase one sector: 150ms
 *
 * @param       saddr : Sector address, set according to actual capacity
 * @retval      None
 */
void norflash_erase_sector(uint32_t saddr)
{
    //printf("fe:%x\r\n", saddr);   /* Monitor flash erase status, for testing */
    saddr *= 4096;
    norflash_write_enable();        /* Write enable */
    norflash_wait_busy();           /* Wait for ready */

    NORFLASH_CS(0);
    spi1_read_write_byte(FLASH_SectorErase);    /* Send sector erase command */
    norflash_send_address(saddr);   /* Send address */
    NORFLASH_CS(1);
    norflash_wait_busy();           /* Wait for sector erase completion */
}

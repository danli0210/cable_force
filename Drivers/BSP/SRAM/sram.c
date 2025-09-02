/**
 ****************************************************************************************************
 * @file        sram.c
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

#include "./BSP/SRAM/sram.h"
#include "./SYSTEM/usart/usart.h"


SRAM_HandleTypeDef g_sram_handler; /* SRAM handle */

/**
 * @brief       Initialize external SRAM
 * @param       None
 * @retval      None
 */
void sram_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    FSMC_NORSRAM_TimingTypeDef fsmc_readwritetim;

    SRAM_CS_GPIO_CLK_ENABLE();    /* Enable SRAM_CS pin clock */
    SRAM_WR_GPIO_CLK_ENABLE();    /* Enable SRAM_WR pin clock */
    SRAM_RD_GPIO_CLK_ENABLE();    /* Enable SRAM_RD pin clock */
    __HAL_RCC_FSMC_CLK_ENABLE();  /* Enable FSMC clock */
    __HAL_RCC_GPIOD_CLK_ENABLE(); /* Enable GPIOD clock */
    __HAL_RCC_GPIOE_CLK_ENABLE(); /* Enable GPIOE clock */
    __HAL_RCC_GPIOF_CLK_ENABLE(); /* Enable GPIOF clock */
    __HAL_RCC_GPIOG_CLK_ENABLE(); /* Enable GPIOG clock */

    gpio_init_struct.Pin = SRAM_CS_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(SRAM_CS_GPIO_PORT, &gpio_init_struct); /* SRAM_CS pin mode configuration */

    gpio_init_struct.Pin = SRAM_WR_GPIO_PIN;
    HAL_GPIO_Init(SRAM_WR_GPIO_PORT, &gpio_init_struct); /* SRAM_WR pin mode configuration */

    gpio_init_struct.Pin = SRAM_RD_GPIO_PIN;
    HAL_GPIO_Init(SRAM_RD_GPIO_PORT, &gpio_init_struct); /* SRAM_RD pin mode configuration */

    /* PD0,1,4,5,8~15 - Configure data and address lines */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | 
                       GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
                       GPIO_PIN_14 | GPIO_PIN_15;
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;       /* Push-pull alternate function */
    gpio_init_struct.Pull = GPIO_PULLUP;           /* Pull-up */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH; /* High speed */
    HAL_GPIO_Init(GPIOD, &gpio_init_struct);

    /* PE0,1,7~15 - Configure data and address lines */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
                       GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |
                       GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio_init_struct);

    /* PF0~5,12~15 - Configure data and address lines */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                       GPIO_PIN_5 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);

    /* PG0~5,10 - Configure data and address lines */
    gpio_init_struct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOG, &gpio_init_struct);

    g_sram_handler.Instance = FSMC_NORSRAM_DEVICE;
    g_sram_handler.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;

    g_sram_handler.Init.NSBank = (SRAM_FSMC_NEX == 1) ? FSMC_NORSRAM_BANK1 : \
                                 (SRAM_FSMC_NEX == 2) ? FSMC_NORSRAM_BANK2 : \
                                 (SRAM_FSMC_NEX == 3) ? FSMC_NORSRAM_BANK3 : 
                                                        FSMC_NORSRAM_BANK4; /* Select FSMC_NE1~4 based on configuration */
    g_sram_handler.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;     /* Address/data lines are not multiplexed */
    g_sram_handler.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;                 /* SRAM memory type */
    g_sram_handler.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;    /* 16-bit data width */
    g_sram_handler.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;   /* Disable burst access mode (only valid for synchronous burst memory) */
    g_sram_handler.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW; /* Wait signal polarity (only used in burst mode) */
    g_sram_handler.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;      /* Memory enables NWAIT one clock cycle before wait state or during wait state */
    g_sram_handler.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;       /* Enable memory write operation */
    g_sram_handler.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;              /* Wait signal enable (not used here) */
    g_sram_handler.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;          /* Read and write use the same timing */
    g_sram_handler.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;  /* Disable wait signal in asynchronous mode (not used here) */
    g_sram_handler.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;              /* Disable burst write */
    
    /* FSMC read timing control register */
    fsmc_readwritetim.AddressSetupTime = 0x02;                              /* Address setup time (ADDSET) = 2 HCLK cycles = 1/168M*2 = 12ns */
    fsmc_readwritetim.AddressHoldTime = 0x00;                               /* Address hold time (ADDHLD) not used in mode A */
    fsmc_readwritetim.DataSetupTime = 0x08;                                 /* Data setup time = 8 HCLK cycles = 6*8 = 48ns */
    fsmc_readwritetim.BusTurnAroundDuration = 0x00;
    fsmc_readwritetim.AccessMode = FSMC_ACCESS_MODE_A;                      /* Access mode A */
    HAL_SRAM_Init(&g_sram_handler, &fsmc_readwritetim, &fsmc_readwritetim);
}

/**
 * @brief       Write specified length of data to SRAM at specified address
 * @param       pbuf    : Data buffer
 * @param       addr    : Starting write address (max 32-bit)
 * @param       datalen : Number of bytes to write (max 32-bit)
 * @retval      None
 */
void sram_write(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *(volatile uint8_t *)(SRAM_BASE_ADDR + addr) = *pbuf;
        addr++;
        pbuf++;
    }
}

/**
 * @brief       Read specified length of data from SRAM at specified address
 * @param       pbuf    : Data buffer
 * @param       addr    : Starting read address (max 32-bit)
 * @param       datalen : Number of bytes to read (max 32-bit)
 * @retval      None
 */
void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *pbuf++ = *(volatile uint8_t *)(SRAM_BASE_ADDR + addr);
        addr++;
    }
}

/******************** Test Functions **********************************/

/**
 * @brief       Test function - Write 1 byte to SRAM at specified address
 * @param       addr    : Starting write address (max 32-bit)
 * @param       data    : Byte to write
 * @retval      None
 */
void sram_test_write(uint32_t addr, uint8_t data)
{
    sram_write(&data, addr, 1); /* Write 1 byte */
}

/**
 * @brief       Test function - Read 1 byte from SRAM at specified address
 * @param       addr    : Starting read address (max 32-bit)
 * @retval      Read data (1 byte)
 */
uint8_t sram_test_read(uint32_t addr)
{
    uint8_t data;
    sram_read(&data, addr, 1); /* Read 1 byte */
    return data;
}

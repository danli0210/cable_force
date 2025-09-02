/**
 ****************************************************************************************************
 * @file        malloc.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       Memory management driver
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

#ifndef __MALLOC_H
#define __MALLOC_H
#include "./SYSTEM/sys/sys.h"

/* Define 3 memory pools */
#define SRAMIN                  0                               /* Internal memory pool */
#define SRAMCCM                 1                               /* CCM memory pool (this SRAM section can only be accessed by CPU!!!) */
#define SRAMEX                  2                               /* External memory pool */
#define SRAMBANK                3                               /* Define number of supported SRAM blocks */

/* Define memory management table type. When external SDRAM is used, uint32_t type must be used, 
 * otherwise uint16_t can be defined to save memory usage */
#define MT_TYPE     uint16_t

/* For single block memory, the total space size occupied by memory management is calculated as follows:
 * size = MEM1_MAX_SIZE + (MEM1_MAX_SIZE / MEM1_BLOCK_SIZE) * sizeof(MT_TYPE)
 * Taking SRAMEX as example, size = 963 * 1024 + (963 * 1024 / 32) * 2 = 1047744 ¡Ö 1023KB
 * Given total memory capacity (size), the maximum memory pool calculation formula is:
 * MEM1_MAX_SIZE = (MEM1_BLOCK_SIZE * size) / (MEM1_BLOCK_SIZE + sizeof(MT_TYPE))
 * Taking CCM as example, MEM2_MAX_SIZE = (32 * 64) / (32 + 2) = 60.24KB ¡Ö 60KB
 */
 
/* mem1 memory parameter settings. mem1 is completely within internal SRAM */
#define MEM1_BLOCK_SIZE         32                              /* Memory block size is 32 bytes */
#define MEM1_MAX_SIZE           100 * 1024                      /* Maximum managed memory 100K */
#define MEM1_ALLOC_TABLE_SIZE   MEM1_MAX_SIZE/MEM1_BLOCK_SIZE   /* Memory table size */

/* mem2 memory parameter settings. mem2 is in CCM, used to manage CCM (special note: this SRAM section can only be accessed by CPU!!) */
#define MEM2_BLOCK_SIZE         32                              /* Memory block size is 32 bytes */
#define MEM2_MAX_SIZE           60 * 1024                       /* Maximum managed memory 60K */
#define MEM2_ALLOC_TABLE_SIZE   MEM2_MAX_SIZE/MEM2_BLOCK_SIZE   /* Memory table size */

/* mem3 memory parameter settings. mem3 is external SRAM */
#define MEM3_BLOCK_SIZE         32                              /* Memory block size is 32 bytes */
#define MEM3_MAX_SIZE           963 * 1024                      /* Maximum managed memory 963K */
#define MEM3_ALLOC_TABLE_SIZE   MEM3_MAX_SIZE/MEM3_BLOCK_SIZE   /* Memory table size */

/* If NULL is not defined, define NULL */
#ifndef NULL
#define NULL 0
#endif

/* Memory management controller */
struct _m_mallco_dev
{
    void (*init)(uint8_t);              /* Initialize */
    uint16_t (*perused)(uint8_t);       /* Memory usage rate */
    uint8_t *membase[SRAMBANK];         /* Memory pool - manages memory for SRAMBANK regions */
    MT_TYPE *memmap[SRAMBANK];          /* Memory management status table */
    uint8_t  memrdy[SRAMBANK];          /* Memory management ready status */
};

extern struct _m_mallco_dev mallco_dev; /* Defined in malloc.c */

/* Function declarations */
void my_mem_init(uint8_t memx);                             /* Memory management initialization function (external/internal call) */
uint16_t my_mem_perused(uint8_t memx);                      /* Get memory usage rate (external/internal call) */
void my_mem_set(void *s, uint8_t c, uint32_t count);        /* Memory set function */
void my_mem_copy(void *des, void *src, uint32_t n);         /* Memory copy function */
void myfree(uint8_t memx, void *ptr);                       /* Memory release (external call) */
void *mymalloc(uint8_t memx, uint32_t size);                /* Memory allocation (external call) */
void *myrealloc(uint8_t memx, void *ptr, uint32_t size);    /* Memory reallocation (external call) */

#endif

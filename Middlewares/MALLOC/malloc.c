/**
 ****************************************************************************************************
 * @file        malloc.c
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

#include "./MALLOC/malloc.h"


#if !(__ARMCC_VERSION >= 6010050)   /* Not using AC6 compiler, i.e., using AC5 compiler */
/* Memory pools (64-byte aligned) */
static __align(64) uint8_t mem1base[MEM1_MAX_SIZE];                                     /* Internal SRAM memory pool */
static __align(64) uint8_t mem2base[MEM2_MAX_SIZE] __attribute__((at(0x10000000)));     /* Internal CCM memory pool */
static __align(64) uint8_t mem3base[MEM3_MAX_SIZE] __attribute__((at(0x68000000)));     /* External SRAM memory pool */

/* Memory management tables */
static MT_TYPE mem1mapbase[MEM1_ALLOC_TABLE_SIZE];                                                  /* Internal SRAM memory pool MAP */
static MT_TYPE mem2mapbase[MEM2_ALLOC_TABLE_SIZE] __attribute__((at(0x10000000 + MEM2_MAX_SIZE)));  /* Internal CCM memory pool MAP */
static MT_TYPE mem3mapbase[MEM3_ALLOC_TABLE_SIZE] __attribute__((at(0x68000000 + MEM3_MAX_SIZE)));  /* External SRAM memory pool MAP */
#else      /* Using AC6 compiler */
/* Memory pools (64-byte aligned) */
static __ALIGNED(64) uint8_t mem1base[MEM1_MAX_SIZE];                                                           /* Internal SRAM memory pool */
static __ALIGNED(64) uint8_t mem2base[MEM2_MAX_SIZE] __attribute__((section(".bss.ARM.__at_0x10000000")));      /* Internal CCM memory pool */
static __ALIGNED(64) uint8_t mem3base[MEM3_MAX_SIZE] __attribute__((section(".bss.ARM.__at_0x68000000")));      /* External SRAM memory pool */ 

/* Memory management tables */
static MT_TYPE mem1mapbase[MEM1_ALLOC_TABLE_SIZE];                                                              /* Internal SRAM memory pool MAP */
static MT_TYPE mem2mapbase[MEM2_ALLOC_TABLE_SIZE] __attribute__((section(".bss.ARM.__at_0x1000F000")));         /* Internal CCM memory pool MAP */
static MT_TYPE mem3mapbase[MEM3_ALLOC_TABLE_SIZE] __attribute__((section(".bss.ARM.__at_0x680F0C00")));         /* External SRAM memory pool MAP */
#endif

/* Memory management parameters */
const uint32_t memtblsize[SRAMBANK] = {MEM1_ALLOC_TABLE_SIZE, MEM2_ALLOC_TABLE_SIZE, MEM3_ALLOC_TABLE_SIZE};    /* Memory table sizes */
const uint32_t memblksize[SRAMBANK] = {MEM1_BLOCK_SIZE, MEM2_BLOCK_SIZE, MEM3_BLOCK_SIZE};                      /* Memory block sizes */
const uint32_t memsize[SRAMBANK] = {MEM1_MAX_SIZE, MEM2_MAX_SIZE, MEM3_MAX_SIZE};                               /* Total memory sizes */

/* Memory management controller */
struct _m_mallco_dev mallco_dev =
{
    my_mem_init,                                /* Memory initialization */
    my_mem_perused,                             /* Memory usage rate */
    mem1base, mem2base, mem3base,               /* Memory pools */
    mem1mapbase, mem2mapbase, mem3mapbase,      /* Memory management status tables */
    0, 0, 0,                                    /* Memory management not ready */
};

/**
 * @brief       Copy memory
 * @param       *des : Destination address
 * @param       *src : Source address
 * @param       n    : Memory length to copy (in bytes)
 * @retval      None
 */
void my_mem_copy(void *des, void *src, uint32_t n)
{
    uint8_t *xdes = des;
    uint8_t *xsrc = src;

    while (n--) *xdes++ = *xsrc++;
}

/**
 * @brief       Set memory value
 * @param       *s    : Memory start address
 * @param       c     : Value to set
 * @param       count : Memory size to set (in bytes)
 * @retval      None
 */
void my_mem_set(void *s, uint8_t c, uint32_t count)
{
    uint8_t *xs = s;

    while (count--) *xs++ = c;
}

/**
 * @brief       Memory management initialization
 * @param       memx : Memory block identifier
 * @retval      None
 */
void my_mem_init(uint8_t memx)
{
    uint8_t mttsize = sizeof(MT_TYPE);  /* Get memmap array type length (uint16_t/uint32_t) */
    my_mem_set(mallco_dev.memmap[memx], 0, memtblsize[memx] * mttsize); /* Clear memory status table data */
    mallco_dev.memrdy[memx] = 1;        /* Memory management initialization OK */
}

/**
 * @brief       Get memory usage rate
 * @param       memx : Memory block identifier
 * @retval      Usage rate (scaled by 10x, 0~1000, representing 0.0%~100.0%)
 */
uint16_t my_mem_perused(uint8_t memx)
{
    uint32_t used = 0;
    uint32_t i;

    for (i = 0; i < memtblsize[memx]; i++)
    {
        if (mallco_dev.memmap[memx][i]) used++;
    }

    return (used * 1000) / (memtblsize[memx]);
}

/**
 * @brief       Memory allocation (internal call)
 * @param       memx : Memory block identifier
 * @param       size : Memory size to allocate (bytes)
 * @retval      Memory offset address
 *   @arg       0 ~ 0xFFFFFFFE : Valid memory offset address
 *   @arg       0xFFFFFFFF     : Invalid memory offset address
 */
static uint32_t my_mem_malloc(uint8_t memx, uint32_t size)
{
    signed long offset = 0;
    uint32_t nmemb;         /* Required number of memory blocks */
    uint32_t cmemb = 0;     /* Consecutive free memory blocks */
    uint32_t i;

    if (!mallco_dev.memrdy[memx])
    {
        mallco_dev.init(memx);          /* Not initialized, execute initialization first */
    }
    
    if (size == 0) return 0xFFFFFFFF;   /* No allocation needed */

    nmemb = size / memblksize[memx];    /* Get number of consecutive memory blocks needed */

    if (size % memblksize[memx]) nmemb++;

    for (offset = memtblsize[memx] - 1; offset >= 0; offset--)  /* Search entire memory control area */
    {
        if (!mallco_dev.memmap[memx][offset])
        {
            cmemb++;                    /* Increase consecutive free memory blocks */
        }
        else 
        {
            cmemb = 0;                  /* Reset consecutive memory blocks */
        }
        
        if (cmemb == nmemb)             /* Found consecutive nmemb free memory blocks */
        {
            for (i = 0; i < nmemb; i++) /* Mark memory blocks as occupied */
            {
                mallco_dev.memmap[memx][offset + i] = nmemb;
            }

            return (offset * memblksize[memx]); /* Return offset address */
        }
    }

    return 0xFFFFFFFF;                  /* No suitable memory blocks found for allocation */
}

/**
 * @brief       Free memory (internal call)
 * @param       memx   : Memory block identifier
 * @param       offset : Memory address offset
 * @retval      Free result
 *   @arg       0, Free successful;
 *   @arg       1, Free failed;
 *   @arg       2, Out of range (failed);
 */
static uint8_t my_mem_free(uint8_t memx, uint32_t offset)
{
    int i;

    if (!mallco_dev.memrdy[memx])   /* Not initialized, execute initialization first */
    {
        mallco_dev.init(memx);
        return 1;                   /* Not initialized */
    }

    if (offset < memsize[memx])     /* Offset within memory pool */
    {
        int index = offset / memblksize[memx];      /* Memory block number for offset */
        int nmemb = mallco_dev.memmap[memx][index]; /* Number of memory blocks */

        for (i = 0; i < nmemb; i++)                 /* Clear memory blocks */
        {
            mallco_dev.memmap[memx][index + i] = 0;
        }

        return 0;
    }
    else
    {
        return 2;   /* Offset out of range */
    }
}

/**
 * @brief       Free memory (external call)
 * @param       memx : Memory block identifier
 * @param       ptr  : Memory start address
 * @retval      None
 */
void myfree(uint8_t memx, void *ptr)
{
    uint32_t offset;

    if (ptr == NULL) return;    /* Address is 0 */

    offset = (uint32_t)ptr - (uint32_t)mallco_dev.membase[memx];
    my_mem_free(memx, offset);  /* Free memory */
}

/**
 * @brief       Allocate memory (external call)
 * @param       memx : Memory block identifier
 * @param       size : Memory size to allocate (bytes)
 * @retval      Start address of allocated memory
 */
void *mymalloc(uint8_t memx, uint32_t size)
{
    uint32_t offset;
    
    offset = my_mem_malloc(memx, size);
    if (offset == 0xFFFFFFFF)   /* Allocation error */
    {
        return NULL;            /* Return NULL (0) */
    }
    else                        /* Allocation successful, return start address */
    {
        return (void *)((uint32_t)mallco_dev.membase[memx] + offset);
    }
}

/**
 * @brief       Reallocate memory (external call)
 * @param       memx : Memory block identifier
 * @param       *ptr : Old memory start address
 * @param       size : Memory size to allocate (bytes)
 * @retval      Start address of newly allocated memory
 */
void *myrealloc(uint8_t memx, void *ptr, uint32_t size)
{
    uint32_t offset;
    
    offset = my_mem_malloc(memx, size);
    if (offset == 0xFFFFFFFF)   /* Allocation error */
    {
        return NULL;            /* Return NULL (0) */
    }
    else                        /* Allocation successful, return start address */
    {
        my_mem_copy((void *)((uint32_t)mallco_dev.membase[memx] + offset), ptr, size);  /* Copy old memory content to new memory */
        myfree(memx, ptr);      /* Free old memory */
        return (void *)((uint32_t)mallco_dev.membase[memx] + offset);                   /* Return new memory start address */
    }
}

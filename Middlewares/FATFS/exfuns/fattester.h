/**
 ****************************************************************************************************
 * @file        fattester.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       FATFS test code
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

#ifndef __FATTESTER_H
#define __FATTESTER_H
#include "./SYSTEM/sys/sys.h"
#include "./FATFS/source/ff.h"

/* Define whether to support file system test functionality
 * 1: Support (enable);
 * 0: Not support (disable);
 */
#define USE_FATTESTER         1

/* If file system testing is supported, enable the following code */
#if USE_FATTESTER == 1
 
/* FATFS test structure */
typedef struct
{
    FIL *file;          /* File structure pointer 1 */
    FILINFO fileinfo;   /* File information */
    DIR dir;            /* Directory */
    uint8_t *fatbuf;    /* Read/write buffer */
    uint8_t initflag;   /* Initialization flag */
} _m_fattester;

extern _m_fattester fattester;      /* FATFS test structure */

/* Function declarations */
uint8_t mf_init(void);                                      /* Initialize file system test */
void mf_free(void);                                         /* Free memory allocated for file system test */
uint8_t mf_mount(uint8_t* path, uint8_t mt);                /* Mount file system */
uint8_t mf_open(uint8_t* path, uint8_t mode);               /* Open file */
uint8_t mf_close(void);                                     /* Close file */
uint8_t mf_read(uint16_t len);                              /* Read file */
uint8_t mf_write(uint8_t* dat, uint16_t len);               /* Write file */
uint8_t mf_opendir(uint8_t* path);                          /* Open directory */
uint8_t mf_closedir(void);                                  /* Close directory */
uint8_t mf_readdir(void);                                   /* Read directory */
uint8_t mf_scan_files(uint8_t* path);                       /* Scan files */
uint32_t mf_showfree(uint8_t* drv);                         /* Show free space */
uint8_t mf_lseek(uint32_t offset);                          /* Seek file position */
uint32_t mf_tell(void);                                     /* Get file position */
uint32_t mf_size(void);                                     /* Get file size */
uint8_t mf_mkdir(uint8_t* pname);                           /* Create directory */
uint8_t mf_fmkfs(uint8_t* path, uint8_t opt, uint16_t au);  /* Format file system */
uint8_t mf_unlink(uint8_t* pname);                          /* Delete file/directory */
uint8_t mf_rename(uint8_t* oldname, uint8_t* newname);      /* Rename file/directory */
void mf_getlabel(uint8_t* path);                            /* Get volume label */
void mf_setlabel(uint8_t* path);                            /* Set volume label */
void mf_gets(uint16_t size);                                /* Get string from file */
uint8_t mf_putc(uint8_t c);                                 /* Put character to file */
uint8_t mf_puts(uint8_t* c);                                /* Put string to file */

#endif
 
#endif

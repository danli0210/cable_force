/**
 ****************************************************************************************************
 * @file        exfuns.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       FATFS extension code
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

#ifndef __EXFUNS_H
#define __EXFUNS_H
#include "./SYSTEM/sys/sys.h"
#include "./FATFS/source/ff.h"

extern FATFS *fs[FF_VOLUMES];
extern FIL *file;
extern FIL *ftemp;
extern UINT br, bw;
extern FILINFO fileinfo;
extern DIR dir;
extern uint8_t *fatbuf;     /* SD card data buffer */

/* File type definitions returned by exfuns_file_type
 * Based on FILE_TYPE_TBL table, defined in exfuns.c
 */
#define T_BIN       0x00    /* Binary file */
#define T_LRC       0x10    /* LRC lyrics file */
#define T_NES       0x20    /* NES game file */
#define T_SMS       0x21    /* SMS game file */
#define T_TEXT      0x30    /* Text file (.txt) */
#define T_C         0x31    /* C source file (.c) */
#define T_H         0x32    /* Header file (.h) */
#define T_WAV       0x40    /* WAV audio file */
#define T_MP3       0x41    /* MP3 audio file */
#define T_OGG       0x42    /* OGG audio file */
#define T_FLAC      0x43    /* FLAC audio file */
#define T_AAC       0x44    /* AAC audio file */
#define T_WMA       0x45    /* WMA audio file */
#define T_MID       0x46    /* MIDI file */
#define T_BMP       0x50    /* BMP image file */
#define T_JPG       0x51    /* JPG image file */
#define T_JPEG      0x52    /* JPEG image file */
#define T_GIF       0x53    /* GIF image file */
#define T_AVI       0x60    /* AVI video file */

/* Function declarations */
uint8_t exfuns_init(void);                                                      /* Initialize and allocate memory */
uint8_t exfuns_file_type(char *fname);                                          /* Identify file type */
uint8_t exfuns_get_free(uint8_t *pdrv, uint32_t *total, uint32_t *free);       /* Get disk total and free capacity */
uint32_t exfuns_get_folder_size(uint8_t *fdname);                              /* Get folder size */
uint8_t *exfuns_get_src_dname(uint8_t *dpfn);                                  /* Get source directory name */
uint8_t exfuns_file_copy(uint8_t(*fcpymsg)(uint8_t *pname, uint8_t pct, uint8_t mode), uint8_t *psrc, uint8_t *pdst, uint32_t totsize, uint32_t cpdsize, uint8_t fwmode);       /* File copy operation */
uint8_t exfuns_folder_copy(uint8_t(*fcpymsg)(uint8_t *pname, uint8_t pct, uint8_t mode), uint8_t *psrc, uint8_t *pdst, uint32_t *totsize, uint32_t *cpdsize, uint8_t fwmode);   /* Folder copy operation */

#endif

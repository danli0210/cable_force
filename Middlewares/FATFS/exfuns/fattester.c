/**
 ****************************************************************************************************
 * @file        fattester.c
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

#include "string.h"
#include "./MALLOC/malloc.h"
#include "./SYSTEM/usart/usart.h"
#include "./FATFS/exfuns/exfuns.h"
#include "./FATFS/exfuns/fattester.h"
#include "./BSP/SDIO/sdio_sdcard.h"


/* If file system testing is supported, enable the following code */
#if USE_FATTESTER == 1

/* FATFS test structure
 * Contains file pointer/file information/directory/read-write buffer etc., convenient for 
 * test functions in fattester.c. When file system test functionality is not needed
 */
_m_fattester fattester;



/**
 * @brief       Initialize file system test (allocate memory)
 *   @note      This function must be called once before executing any file system tests.
 *              This function only needs to be called successfully once, no need to call repeatedly!!
 * @param       None
 * @retval      Execution result: 0: Success; 1: Failure;
 */
uint8_t mf_init(void)
{
    fattester.file = (FIL *)mymalloc(SRAMIN, sizeof(FIL));      /* Allocate memory for file */
    fattester.fatbuf = (uint8_t *)mymalloc(SRAMIN, 512);        /* Allocate memory for fattester.fatbuf */

    if (fattester.file && fattester.fatbuf)
    {
        return 0;   /* Allocation successful */
    }
    else
    {
        mf_free();  /* Free memory */
        return 1;   /* Allocation failed */
    }
}

/**
 * @brief       Free memory allocated for file system test
 *   @note      After calling this function, file system test functionality will be disabled.
 * @param       None
 * @retval      None
 */
void mf_free(void)
{
    myfree(SRAMIN, fattester.file);
    myfree(SRAMIN, fattester.fatbuf);
}

/**
 * @brief       Register work area for disk
 * @param       path : Disk path, such as "0:", "1:"
 * @param       mt   : 0: Do not register immediately (register later); 1: Register immediately
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_mount(uint8_t *path, uint8_t mt)
{
    return f_mount(fs[1], (const TCHAR *)path, mt);
}

/**
 * @brief       Open file
 * @param       path : Path + filename
 * @param       mode : Open mode
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_open(uint8_t *path, uint8_t mode)
{
    uint8_t res;
    res = f_open(fattester.file, (const TCHAR *)path, mode);    /* Open file */
    return res;
}

/**
 * @brief       Close file
 * @param       None
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_close(void)
{
    f_close(fattester.file);
    return 0;
}

/**
 * @brief       Read data
 * @param       len : Length to read
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_read(uint16_t len)
{
    uint16_t i, t;
    uint8_t res = 0;
    uint16_t tlen = 0;
    uint32_t br = 0;
    printf("\r\nRead fattester.file data is:\r\n");

    for (i = 0; i < len / 512; i++)
    {
        res = f_read(fattester.file, fattester.fatbuf, 512, &br);

        if (res)
        {
            printf("Read Error:%d\r\n", res);
            break;
        }
        else
        {
            tlen += br;

            for (t = 0; t < br; t++)printf("%c", fattester.fatbuf[t]);
        }
    }

    if (len % 512)
    {
        res = f_read(fattester.file, fattester.fatbuf, len % 512, &br);

        if (res)    /* Read data error */
        {
            printf("\r\nRead Error:%d\r\n", res);
        }
        else
        {
            tlen += br;

            for (t = 0; t < br; t++)printf("%c", fattester.fatbuf[t]);
        }
    }

    if (tlen)printf("\r\nReaded data len:%d\r\n", tlen);    /* Length of data read */

    printf("Read data over\r\n");
    return res;
}

/**
 * @brief       Write data
 * @param       pdata : Data buffer
 * @param       len   : Write length
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_write(uint8_t *pdata, uint16_t len)
{
    uint8_t res;
    uint32_t bw = 0;

    printf("\r\nBegin Write fattester.file...\r\n");
    printf("Write data len:%d\r\n", len);
    res = f_write(fattester.file, pdata, len, &bw);

    if (res)
    {
        printf("Write Error:%d\r\n", res);
    }
    else
    {
        printf("Writed data len:%d\r\n", bw);
    }

    printf("Write data over.\r\n");
    return res;
}

/**
 * @brief       Open directory
 * @param       path : Path
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_opendir(uint8_t *path)
{
    return f_opendir(&fattester.dir, (const TCHAR *)path);
}

/**
 * @brief       Close directory
 * @param       None
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_closedir(void)
{
    return f_closedir(&fattester.dir);
}

/**
 * @brief       Read directory
 * @param       None
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_readdir(void)
{
    uint8_t res;
    res = f_readdir(&fattester.dir, &fattester.fileinfo);   /* Read information of a file */

    if (res != FR_OK)return res;    /* Error occurred */

    printf("\r\n fattester.dir info:\r\n");

    printf("fattester.dir.dptr:%d\r\n", fattester.dir.dptr);
    printf("fattester.dir.obj.id:%d\r\n", fattester.dir.obj.id);
    printf("fattester.dir.obj.sclust:%d\r\n", fattester.dir.obj.sclust);
    printf("fattester.dir.obj.objsize:%lld\r\n", fattester.dir.obj.objsize);
    printf("fattester.dir.obj.c_ofs:%d\r\n", fattester.dir.obj.c_ofs);
    printf("fattester.dir.clust:%d\r\n", fattester.dir.clust);
    printf("fattester.dir.sect:%d\r\n", fattester.dir.sect);
    printf("fattester.dir.blk_ofs:%d\r\n", fattester.dir.blk_ofs);

    printf("\r\n");
    printf("fattester.file Name is:%s\r\n", fattester.fileinfo.fname);
    printf("fattester.file Size is:%lld\r\n", fattester.fileinfo.fsize);
    printf("fattester.file data is:%d\r\n", fattester.fileinfo.fdate);
    printf("fattester.file time is:%d\r\n", fattester.fileinfo.ftime);
    printf("fattester.file Attr is:%d\r\n", fattester.fileinfo.fattrib);
    printf("\r\n");
    return 0;
}

/**
 * @brief       Scan files
 * @param       path : Path
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_scan_files(uint8_t *path)
{
    FRESULT res;
    res = f_opendir(&fattester.dir, (const TCHAR *)path); /* Open a directory */

    if (res == FR_OK)
    {
        printf("\r\n");

        while (1)
        {
            res = f_readdir(&fattester.dir, &fattester.fileinfo);   /* Read a file from directory */

            if (res != FR_OK || fattester.fileinfo.fname[0] == 0)
            {
                break;  /* Error or end reached, exit */
            }
            
            // if (fattester.fileinfo.fname[0] == '.') continue;    /* Ignore parent directory */
            printf("%s/", path);    /* Print path */
            printf("%s\r\n", fattester.fileinfo.fname); /* Print filename */
        }
    }

    return res;
}

/**
 * @brief       Show free space
 * @param       path : Path (drive letter)
 * @retval      Free space (bytes)
 */
uint32_t mf_showfree(uint8_t *path)
{
    FATFS *fs1;
    uint8_t res;
    uint32_t fre_clust = 0, fre_sect = 0, tot_sect = 0;
    /* Get disk information and free cluster count */
    res = f_getfree((const TCHAR *)path, (DWORD *)&fre_clust, &fs1);

    if (res == 0)
    {
        tot_sect = (fs1->n_fatent - 2) * fs1->csize;/* Get total sector count */
        fre_sect = fre_clust * fs1->csize;          /* Get free sector count */
#if FF_MAX_SS!=512
        tot_sect *= fs1->ssize / 512;
        fre_sect *= fs1->ssize / 512;
#endif

        if (tot_sect < 20480)   /* Total capacity less than 10MB */
        {
            /* Print free space in unit of KB (assuming 512 bytes/sector) */
            printf("\r\nDisk total capacity:%d KB\r\n"
                   "Available space:%d KB\r\n",
                   tot_sect >> 1, fre_sect >> 1);
        }
        else
        {
            /* Print free space in unit of MB (assuming 512 bytes/sector) */
            printf("\r\nDisk total capacity:%d MB\r\n"
                   "Available space:%d MB\r\n",
                   tot_sect >> 11, fre_sect >> 11);
        }
    }

    return fre_sect;
}

/**
 * @brief       File read/write pointer offset
 * @param       offset : Offset relative to start address
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_lseek(uint32_t offset)
{
    return f_lseek(fattester.file, offset);
}

/**
 * @brief       Read current position of file read/write pointer
 * @param       None
 * @retval      Current position
 */
uint32_t mf_tell(void)
{
    return f_tell(fattester.file);
}

/**
 * @brief       Read file size
 * @param       None
 * @retval      File size
 */
uint32_t mf_size(void)
{
    return f_size(fattester.file);
}

/**
 * @brief       Create directory
 * @param       path : Directory path + name
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_mkdir(uint8_t *path)
{
    return f_mkdir((const TCHAR *)path);
}

/**
 * @brief       Format disk
 * @param       path : Disk path, such as "0:", "1:"
 * @param       opt  : Mode, FM_FAT, FM_FAT32, FM_EXFAT, FM_ANY etc...
 * @param       au   : Cluster size
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_fmkfs(uint8_t *path, uint8_t opt, uint16_t au)
{
    MKFS_PARM temp = {FM_ANY, 0, 0, 0, 0};
    temp.fmt = opt;     /* File system format, 1: FM_FAT; 2: FM_FAT32; 4: FM_EXFAT; */
    temp.au_size = au;  /* Cluster size definition, 0 uses default cluster size */
    return f_mkfs((const TCHAR *)path, &temp, 0, FF_MAX_SS);    /* Format with default parameters, workbuf, minimum _MAX_SS size */
}

/**
 * @brief       Delete file/directory
 * @param       path : File/directory path + name
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_unlink(uint8_t *path)
{
    return  f_unlink((const TCHAR *)path);
}

/**
 * @brief       Rename file/directory (can also move files if directories are different!)
 * @param       oldname : Previous name
 * @param       newname : New name
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_rename(uint8_t *oldname, uint8_t *newname)
{
    return  f_rename((const TCHAR *)oldname, (const TCHAR *)newname);
}

/**
 * @brief       Get volume label (disk name)
 * @param       path : Disk path, such as "0:", "1:"
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
void mf_getlabel(uint8_t *path)
{
    uint8_t buf[20];
    uint32_t sn = 0;
    uint8_t res;
    res = f_getlabel ((const TCHAR *)path, (TCHAR *)buf, (DWORD *)&sn);

    if (res == FR_OK)
    {
        printf("\r\nDisk %s volume label is:%s\r\n", path, buf);
        printf("Disk %s serial number:%X\r\n\r\n", path, sn);
    }
    else
    {
        printf("\r\nGet failed, error code:%X\r\n", res);
    }
}

/**
 * @brief       Set volume label (disk name), maximum 11 characters!!, supports numbers, uppercase letters, and Chinese characters etc.
 * @param       path : Disk number + name, such as "0:ALIENTEK", "1:OPENEDV"
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
void mf_setlabel(uint8_t *path)
{
    uint8_t res;
    res = f_setlabel ((const TCHAR *)path);

    if (res == FR_OK)
    {
        printf("\r\nDisk volume label set successfully:%s\r\n", path);
    }
    else printf("\r\nDisk volume label set failed, error code:%X\r\n", res);
}

/**
 * @brief       Read a string from file
 * @param       size : Length to read
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
void mf_gets(uint16_t size)
{
    TCHAR *rbuf;
    rbuf = f_gets((TCHAR *)fattester.fatbuf, size, fattester.file);

    if (*rbuf == 0)return  ; /* No data read */
    else
    {
        printf("\r\nThe String Readed Is:%s\r\n", rbuf);
    }
}

/**
 * @brief       Write a character to file (requires FF_USE_STRFUNC >= 1)
 * @param       c : Character to write
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_putc(uint8_t c)
{
    return f_putc((TCHAR)c, fattester.file);
}

/**
 * @brief       Write string to file (requires FF_USE_STRFUNC >= 1)
 * @param       str : String to write
 * @retval      Execution result (see FATFS, FRESULT definition)
 */
uint8_t mf_puts(uint8_t *str)
{
    return f_puts((TCHAR *)str, fattester.file);
}

#endif

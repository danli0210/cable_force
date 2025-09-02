/**
 ****************************************************************************************************
 * @file        exfuns.c
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

#include "string.h"
#include "./MALLOC/malloc.h"
#include "./SYSTEM/usart/usart.h"
#include "./FATFS/exfuns/exfuns.h"
#include "./FATFS/exfuns/fattester.h"


#define FILE_MAX_TYPE_NUM       7       /* Maximum FILE_MAX_TYPE_NUM major categories */
#define FILE_MAX_SUBT_NUM       7       /* Maximum FILE_MAX_SUBT_NUM subcategories */

/* File type table */
char *const FILE_TYPE_TBL[FILE_MAX_TYPE_NUM][FILE_MAX_SUBT_NUM] =
{
    {"BIN"},            /* Binary files */
    {"LRC"},            /* LRC files */
    {"NES", "SMS"},     /* NES/SMS files */
    {"TXT", "C", "H"},  /* Text files */
    {"WAV", "MP3", "OGG", "FLAC", "AAC", "WMA", "MID"},   /* Supported audio files */
    {"BMP", "JPG", "JPEG", "GIF"},  /* Image files */
    {"AVI"},            /* Video files */
};
    
/******************************************************************************************/
/* Public file area, used when using malloc */

/* Logical disk work area (must allocate memory for fs before calling any FATFS related functions) */
FATFS *fs[FF_VOLUMES];  

/******************************************************************************************/


/**
 * @brief       Allocate memory for exfuns
 * @param       None
 * @retval      0: Success; 1: Failure
 */
uint8_t exfuns_init(void)
{
    uint8_t i;
    uint8_t res = 0;

    for (i = 0; i < FF_VOLUMES; i++)
    {
        fs[i] = (FATFS *)mymalloc(SRAMIN, sizeof(FATFS));   /* Allocate memory for disk i work area */

        if (!fs[i])break;
    }
    
#if USE_FATTESTER == 1  /* If file system testing is enabled */
    res = mf_init();    /* Initialize file system testing (allocate memory) */
#endif
    
    if (i == FF_VOLUMES && res == 0)
    {
        return 0;   /* All successful */
    }
    else 
    {
        return 1;   /* If any allocation fails, return failure */
    }
}

/**
 * @brief       Convert lowercase letter to uppercase, keep numbers unchanged
 * @param       c : Letter to convert
 * @retval      Converted letter (uppercase)
 */
uint8_t exfuns_char_upper(uint8_t c)
{
    if (c < 'A')return c;   /* Number, keep unchanged */

    if (c >= 'a')
    {
        return c - 0x20;    /* Convert to uppercase */
    }
    else
    {
        return c;           /* Already uppercase, keep unchanged */
    }
}

/**
 * @brief       Report file type
 * @param       fname : File name
 * @retval      File type
 *   @arg       0xFF : Unrecognizable file type
 *   @arg       Other: High 4 bits indicate major category, low 4 bits indicate subcategory
 */
uint8_t exfuns_file_type(char *fname)
{
    uint8_t tbuf[5];
    char *attr = 0;   /* File extension */
    uint8_t i = 0, j;

    while (i < 250)
    {
        i++;

        if (*fname == '\0')break;   /* Reached the end */

        fname++;
    }

    if (i == 250)return 0xFF;   /* Invalid string */

    for (i = 0; i < 5; i++)     /* Get file extension */
    {
        fname--;

        if (*fname == '.')
        {
            fname++;
            attr = fname;
            break;
        }
    }

    if (attr == 0)return 0xFF;

    strcpy((char *)tbuf, (const char *)attr);       /* Copy */

    for (i = 0; i < 4; i++)tbuf[i] = exfuns_char_upper(tbuf[i]);    /* Convert all to uppercase */

    for (i = 0; i < FILE_MAX_TYPE_NUM; i++)         /* Major category comparison */
    {
        for (j = 0; j < FILE_MAX_SUBT_NUM; j++)     /* Subcategory comparison */
        {
            if (*FILE_TYPE_TBL[i][j] == 0)break;    /* No more members to compare in this group */

            if (strcmp((const char *)FILE_TYPE_TBL[i][j], (const char *)tbuf) == 0) /* Found */
            {
                return (i << 4) | j;
            }
        }
    }

    return 0xFF;    /* Not found */
}

/**
 * @brief       Get disk free capacity
 * @param       pdrv : Disk number ("0:"~"9:")
 * @param       total: Total capacity (KB)
 * @param       free : Free capacity (KB)
 * @retval      0: Normal; Other: Error code
 */
uint8_t exfuns_get_free(uint8_t *pdrv, uint32_t *total, uint32_t *free)
{
    FATFS *fs1;
    uint8_t res;
    uint32_t fre_clust = 0, fre_sect = 0, tot_sect = 0;
    
    /* Get disk information and free cluster count */
    res = (uint32_t)f_getfree((const TCHAR *)pdrv, (DWORD *)&fre_clust, &fs1);

    if (res == 0)
    {
        tot_sect = (fs1->n_fatent - 2) * fs1->csize;    /* Get total sector count */
        fre_sect = fre_clust * fs1->csize;              /* Get free sector count */
#if FF_MAX_SS!=512  /* If sector size is not 512 bytes, convert to 512 bytes */
        tot_sect *= fs1->ssize / 512;
        fre_sect *= fs1->ssize / 512;
#endif
        *total = tot_sect >> 1;     /* Unit: KB */
        *free = fre_sect >> 1;      /* Unit: KB */
    }

    return res;
}

/**
 * @brief       File copy
 *   @note      Copy psrc file to pdst.
 *              Note: File size should not exceed 4GB.
 *
 * @param       fcpymsg : Function pointer for copy progress display
 *                  pname: File/folder name
 *                  pct: Percentage
 *                  mode:
 *                      bit0 : Update file name
 *                      bit1 : Update percentage pct
 *                      bit2 : Update folder
 *                      Other: Reserved
 *                  Return value: 0: Normal; 1: Force exit;
 *
 * @param       psrc    : Source file
 * @param       pdst    : Destination file
 * @param       totsize : Total size (when totsize is 0, indicates single file copy only)
 * @param       cpdsize : Already copied size
 * @param       fwmode  : File write mode
 *   @arg       0: Do not overwrite existing files
 *   @arg       1: Overwrite existing files
 *
 * @retval      Execution result
 *   @arg       0   : Normal
 *   @arg       0xFF: Force exit
 *   @arg       Other: Error code
 */
uint8_t exfuns_file_copy(uint8_t(*fcpymsg)(uint8_t *pname, uint8_t pct, uint8_t mode), uint8_t *psrc, uint8_t *pdst, 
                                      uint32_t totsize, uint32_t cpdsize, uint8_t fwmode)
{
    uint8_t res;
    uint16_t br = 0;
    uint16_t bw = 0;
    FIL *fsrc = 0;
    FIL *fdst = 0;
    uint8_t *fbuf = 0;
    uint8_t curpct = 0;
    unsigned long long lcpdsize = cpdsize;
    
    fsrc = (FIL *)mymalloc(SRAMIN, sizeof(FIL));    /* Allocate memory */
    fdst = (FIL *)mymalloc(SRAMIN, sizeof(FIL));
    fbuf = (uint8_t *)mymalloc(SRAMIN, 8192);

    if (fsrc == NULL || fdst == NULL || fbuf == NULL)
    {
        res = 100;  /* Reserve previous values for FATFS */
    }
    else
    {
        if (fwmode == 0)
        {
            fwmode = FA_CREATE_NEW;     /* Do not overwrite */
        }
        else 
        {
            fwmode = FA_CREATE_ALWAYS;  /* Overwrite existing files */
        }
        
        res = f_open(fsrc, (const TCHAR *)psrc, FA_READ | FA_OPEN_EXISTING);        /* Open read-only file */

        if (res == 0)res = f_open(fdst, (const TCHAR *)pdst, FA_WRITE | fwmode);    /* Open second file only if first opens successfully */

        if (res == 0)           /* Both files opened successfully */
        {
            if (totsize == 0)   /* Single file copy only */
            {
                totsize = fsrc->obj.objsize;
                lcpdsize = 0;
                curpct = 0;
            }
            else
            {
                curpct = (lcpdsize * 100) / totsize;            /* Get new percentage */
            }
            
            fcpymsg(psrc, curpct, 0x02);                        /* Update percentage */

            while (res == 0)    /* Start copying */
            {
                res = f_read(fsrc, fbuf, 8192, (UINT *)&br);    /* Read 8192 bytes from source */

                if (res || br == 0)break;

                res = f_write(fdst, fbuf, (UINT)br, (UINT *)&bw);/* Write to destination file */
                lcpdsize += bw;

                if (curpct != (lcpdsize * 100) / totsize)       /* Check if percentage needs update */
                {
                    curpct = (lcpdsize * 100) / totsize;

                    if (fcpymsg(psrc, curpct, 0x02))            /* Update percentage */
                    {
                        res = 0xFF;                             /* Force exit */
                        break;
                    }
                }

                if (res || bw < br)break;
            }

            f_close(fsrc);
            f_close(fdst);
        }
    }

    myfree(SRAMIN, fsrc); /* Free memory */
    myfree(SRAMIN, fdst);
    myfree(SRAMIN, fbuf);
    return res;
}

/**
 * @brief       Get folder from path
 *   @note      Remove all path components, leaving only folder name.
 * @param       pname : Full path 
 * @retval      0   : Path is just a volume label
 *              Other: Folder name start address
 */
uint8_t *exfuns_get_src_dname(uint8_t *pname)
{
    uint16_t temp = 0;

    while (*pname != 0)
    {
        pname++;
        temp++;
    }

    if (temp < 4)return 0;

    while ((*pname != 0x5c) && (*pname != 0x2f))pname--;    /* Trace back to last "\" or "/" */

    return ++pname;
}

/**
 * @brief       Get folder size
 *   @note      Note: Folder size should not exceed 4GB.
 * @param       fdname : Full path 
 * @retval      0   : Folder size is 0, or error occurred during reading
 *              Other: Folder size
 */
uint32_t exfuns_get_folder_size(uint8_t *fdname)
{
#define MAX_PATHNAME_DEPTH  512 + 1     /* Maximum target file path + filename depth */
    uint8_t res = 0;
    DIR *fddir = 0;         /* Directory */
    FILINFO *finfo = 0;     /* File information */
    uint8_t *pathname = 0;  /* Target folder path + filename */
    uint16_t pathlen = 0;   /* Target path length */
    uint32_t fdsize = 0;

    fddir = (DIR *)mymalloc(SRAMIN, sizeof(DIR));   /* Allocate memory */
    finfo = (FILINFO *)mymalloc(SRAMIN, sizeof(FILINFO));

    if (fddir == NULL || finfo == NULL)res = 100;

    if (res == 0)
    {
        pathname = mymalloc(SRAMIN, MAX_PATHNAME_DEPTH);

        if (pathname == NULL)res = 101;

        if (res == 0)
        {
            pathname[0] = 0;
            strcat((char *)pathname, (const char *)fdname);     /* Copy path */
            res = f_opendir(fddir, (const TCHAR *)fdname);      /* Open source directory */

            if (res == 0)   /* Directory opened successfully */
            {
                while (res == 0)   /* Start copying folder contents */
                {
                    res = f_readdir(fddir, finfo);                  /* Read a file from directory */

                    if (res != FR_OK || finfo->fname[0] == 0)break; /* Error or end reached, exit */

                    if (finfo->fname[0] == '.')continue;            /* Ignore parent directory */

                    if (finfo->fattrib & 0x10)   /* Is subdirectory (file attribute, 0x20: archive file; 0x10: subdirectory) */
                    {
                        pathlen = strlen((const char *)pathname);   /* Get current path length */
                        strcat((char *)pathname, (const char *)"/");/* Add slash */
                        strcat((char *)pathname, (const char *)finfo->fname);   /* Add subdirectory name to source path */
                        //printf("\r\nsub folder:%s\r\n",pathname);      /* Print subdirectory name */
                        fdsize += exfuns_get_folder_size(pathname);     /* Get subdirectory size, recursive call */
                        pathname[pathlen] = 0;                          /* Add terminator */
                    }
                    else
                    {
                        fdsize += finfo->fsize; /* Not a directory, directly add file size */
                    }
                }
            }

            myfree(SRAMIN, pathname);
        }
    }

    myfree(SRAMIN, fddir);
    myfree(SRAMIN, finfo);

    if (res)
    {
        return 0;
    }
    else 
    {
        return fdsize;
    }
}

/**
 * @brief       Folder copy
 *   @note      Copy psrc folder to pdst folder.
 *              Note: File size should not exceed 4GB.
 *
 * @param       fcpymsg : Function pointer for copy progress display
 *                  pname: File/folder name
 *                  pct: Percentage
 *                  mode:
 *                      bit0 : Update file name
 *                      bit1 : Update percentage pct
 *                      bit2 : Update folder
 *                      Other: Reserved
 *                  Return value: 0: Normal; 1: Force exit;
 *
 * @param       psrc    : Source folder
 * @param       pdst    : Destination folder
 *   @note      Must be in format like "X:"/"X:XX"/"X:XX/XX" etc. and ensure parent folder exists
 *
 * @param       totsize : Total size (when totsize is 0, indicates single file copy only)
 * @param       cpdsize : Already copied size
 * @param       fwmode  : File write mode
 *   @arg       0: Do not overwrite existing files
 *   @arg       1: Overwrite existing files
 *
 * @retval      Execution result
 *   @arg       0   : Normal
 *   @arg       0xFF: Force exit
 *   @arg       Other: Error code
 */
uint8_t exfuns_folder_copy(uint8_t(*fcpymsg)(uint8_t *pname, uint8_t pct, uint8_t mode), uint8_t *psrc, uint8_t *pdst, 
                           uint32_t *totsize, uint32_t *cpdsize, uint8_t fwmode)
{
#define MAX_PATHNAME_DEPTH 512 + 1  /* Maximum target file path + filename depth */
    uint8_t res = 0;
    DIR *srcdir = 0;    /* Source directory */
    DIR *dstdir = 0;    /* Destination directory */
    FILINFO *finfo = 0; /* File information */
    uint8_t *fn = 0;    /* Long filename */

    uint8_t *dstpathname = 0;   /* Destination folder path + filename */
    uint8_t *srcpathname = 0;   /* Source folder path + filename */

    uint16_t dstpathlen = 0;    /* Destination path length */
    uint16_t srcpathlen = 0;    /* Source path length */


    srcdir = (DIR *)mymalloc(SRAMIN, sizeof(DIR));  /* Allocate memory */
    dstdir = (DIR *)mymalloc(SRAMIN, sizeof(DIR));
    finfo = (FILINFO *)mymalloc(SRAMIN, sizeof(FILINFO));

    if (srcdir == NULL || dstdir == NULL || finfo == NULL)res = 100;

    if (res == 0)
    {
        dstpathname = mymalloc(SRAMIN, MAX_PATHNAME_DEPTH);
        srcpathname = mymalloc(SRAMIN, MAX_PATHNAME_DEPTH);

        if (dstpathname == NULL || srcpathname == NULL)res = 101;

        if (res == 0)
        {
            dstpathname[0] = 0;
            srcpathname[0] = 0;
            strcat((char *)srcpathname, (const char *)psrc);    /* Copy original source file path */
            strcat((char *)dstpathname, (const char *)pdst);    /* Copy original destination file path */
            res = f_opendir(srcdir, (const TCHAR *)psrc);       /* Open source directory */

            if (res == 0)   /* Directory opened successfully */
            {
                strcat((char *)dstpathname, (const char *)"/"); /* Add slash */
                fn = exfuns_get_src_dname(psrc);

                if (fn == 0)   /* Volume label copy */
                {
                    dstpathlen = strlen((const char *)dstpathname);
                    dstpathname[dstpathlen] = psrc[0];  /* Record volume label */
                    dstpathname[dstpathlen + 1] = 0;    /* Terminator */
                }
                else strcat((char *)dstpathname, (const char *)fn); /* Add filename */

                fcpymsg(fn, 0, 0x04);   /* Update folder name */
                res = f_mkdir((const TCHAR *)dstpathname);  /* Create folder if it doesn't exist */

                if (res == FR_EXIST)res = 0;

                while (res == 0)   /* Start copying folder contents */
                {
                    res = f_readdir(srcdir, finfo);         /* Read a file from directory */

                    if (res != FR_OK || finfo->fname[0] == 0)break; /* Error or end reached, exit */

                    if (finfo->fname[0] == '.')continue;    /* Ignore parent directory */

                    fn = (uint8_t *)finfo->fname;           /* Get filename */
                    dstpathlen = strlen((const char *)dstpathname); /* Get current destination path length */
                    srcpathlen = strlen((const char *)srcpathname); /* Get source path length */

                    strcat((char *)srcpathname, (const char *)"/"); /* Add slash to source path */

                    if (finfo->fattrib & 0x10)  /* Is subdirectory (file attribute, 0x20: archive file; 0x10: subdirectory) */
                    {
                        strcat((char *)srcpathname, (const char *)fn);  /* Add subdirectory name to source path */
                        res = exfuns_folder_copy(fcpymsg, srcpathname, dstpathname, totsize, cpdsize, fwmode);   /* Copy folder */
                    }
                    else     /* Not a directory */
                    {
                        strcat((char *)dstpathname, (const char *)"/"); /* Add slash to destination path */
                        strcat((char *)dstpathname, (const char *)fn);  /* Add filename to destination path */
                        strcat((char *)srcpathname, (const char *)fn);  /* Add filename to source path */
                        fcpymsg(fn, 0, 0x01);       /* Update filename */
                        res = exfuns_file_copy(fcpymsg, srcpathname, dstpathname, *totsize, *cpdsize, fwmode);  /* Copy file */
                        *cpdsize += finfo->fsize;   /* Add file size */
                    }

                    srcpathname[srcpathlen] = 0;    /* Add terminator */
                    dstpathname[dstpathlen] = 0;    /* Add terminator */
                }
            }

            myfree(SRAMIN, dstpathname);
            myfree(SRAMIN, srcpathname);
        }
    }

    myfree(SRAMIN, srcdir);
    myfree(SRAMIN, dstdir);
    myfree(SRAMIN, finfo);
    return res;
}

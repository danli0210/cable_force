/**
 ****************************************************************************************************
 * @file        data_logger.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       Data Logging Module Implementation
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

#include "./DATALOG/data_logger.h"
#include "./BSP/SDIO/sdio_sdcard.h"
#include "./FATFS/exfuns/exfuns.h"
#include "./MALLOC/malloc.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Forward declarations */
static int flush_signal_cache(void);
static int flush_result_cache(void);
static int open_data_files(void);

/* Global data logger */
static data_logger_t *g_logger = NULL;

/* File handle cache - Reduce frequent file open/close operations */
static FIL original_file;
static FIL reconstructed_file;
static FIL tension_file;
static bool files_opened = false;

/* ================ Initialization Functions ================ */

int data_logger_init(void) {
    /* Allocate logger memory - Use external SRAM */
    g_logger = (data_logger_t*)mymalloc(SRAMEX, sizeof(data_logger_t));
    if (!g_logger) {
        printf("Data logger memory allocation failed!\r\n");
        return -1;
    }
    
    /* Initialize logger */
    memset(g_logger, 0, sizeof(data_logger_t));
    g_logger->logging_enabled = false;
    g_logger->signal_cache_count = 0;
    g_logger->result_cache_count = 0;
    g_logger->total_records_written = 0;
    
    /* Create analysis header file */
    if (create_analysis_header() != 0) {
        printf("Failed to create analysis header file\r\n");
        return -1;
    }
    
    printf("Data logger initialized successfully\r\n");
    return 0;
}

int data_logger_cleanup(void) {
    if (g_logger) {
        /* Flush all caches */
        flush_all_caches();
        
        /* Close files */
        if (files_opened) {
            f_close(&original_file);
            f_close(&reconstructed_file);
            f_close(&tension_file);
            files_opened = false;
        }
        
        /* Free memory */
        myfree(SRAMEX, g_logger);
        g_logger = NULL;
        
        printf("Data logger cleaned up\r\n");
    }
    return 0;
}

void data_logger_enable(bool enable) {
    if (g_logger) {
        g_logger->logging_enabled = enable;
        printf("Data logging: %s\r\n", enable ? "Enabled" : "Disabled");
        
        /* If logging is enabled, open files immediately */
        if (enable && !files_opened) {
            open_data_files();
        }
    }
}

/* ================ File Management ================ */

static int open_data_files(void) {
    if (files_opened) return 0;
    
    FRESULT res;
    
    printf("Creating data log files...\r\n");
    
    /* Open original data file */
    res = f_open(&original_file, ORIGINAL_DATA_FILE, 
                 FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Cannot create original data file: %d\r\n", res);
        return -1;
    }
    printf("Original data file created successfully: %s\r\n", ORIGINAL_DATA_FILE);
    
    /* Open reconstructed data file */
    res = f_open(&reconstructed_file, RECONSTRUCTED_DATA_FILE, 
                 FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Cannot create reconstructed data file: %d\r\n", res);
        f_close(&original_file);
        return -1;
    }
    printf("Reconstructed data file created successfully: %s\r\n", RECONSTRUCTED_DATA_FILE);
    
    /* Open tension results file */
    res = f_open(&tension_file, TENSION_RESULTS_FILE, 
                 FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Cannot create tension results file: %d\r\n", res);
        f_close(&original_file);
        f_close(&reconstructed_file);
        return -1;
    }
    printf("Tension results file created successfully: %s\r\n", TENSION_RESULTS_FILE);
    
    /* Write CSV header */
    const char* csv_header = "WindowID,Timestamp,Tension_kN,Frequency_Hz,"
                            "DominantMode,Confidence,ProcessingTime_ms,"
                            "ReconstructionUsed,PacketLossRate\r\n";
    UINT bw;
    res = f_write(&tension_file, csv_header, strlen(csv_header), &bw);
    if (res != FR_OK) {
        printf("CSV header write failed: %d\r\n", res);
    } else {
        printf("CSV header written successfully\r\n");
    }
    f_sync(&tension_file);
    
    files_opened = true;
    printf("All data files created and opened\r\n");
    return 0;
}

/* ================ Signal Data Logging ================ */

int log_signal_data(const float* original_signal, 
                   const float* reconstructed_signal,
                   const uint8_t* mask,
                   uint32_t window_id,
                   bool has_packet_loss,
                   float packet_loss_rate) {
    
    if (!g_logger || !g_logger->logging_enabled) return 0;
    if (!original_signal || !reconstructed_signal || !mask) return -1;
    
    /* Check cache space */
    if (g_logger->signal_cache_count >= SIGNAL_CACHE_SIZE) {
        /* Flush signal cache */
        if (flush_signal_cache() != 0) {
            printf("Signal cache flush failed\r\n");
            return -1;
        }
    }
    
    /* Add to cache */
    signal_record_t* record = &g_logger->signal_cache[g_logger->signal_cache_count];
    
    memcpy(record->original_signal, original_signal, SIGNAL_LEN * sizeof(float));
    memcpy(record->reconstructed_signal, reconstructed_signal, SIGNAL_LEN * sizeof(float));
    memcpy(record->signal_mask, mask, SIGNAL_LEN * sizeof(uint8_t));
    record->window_id = window_id;
    record->timestamp = HAL_GetTick();
    record->has_packet_loss = has_packet_loss;
    record->packet_loss_rate = packet_loss_rate;
    
    g_logger->signal_cache_count++;
    
    return 0;
}

static int flush_signal_cache(void) {
    if (!g_logger || g_logger->signal_cache_count == 0) return 0;
    
    /* Ensure files are opened */
    if (!files_opened && open_data_files() != 0) {
        return -1;
    }
    
    UINT bw;
    FRESULT res;
    
    printf("Flushing signal cache (%d windows)...\r\n", g_logger->signal_cache_count);
    
    /* Write signal data one by one with detailed logging */
    for (uint32_t i = 0; i < g_logger->signal_cache_count; i++) {
        signal_record_t* record = &g_logger->signal_cache[i];
        
        printf("Writing window %d - Original signal...", i);
        
        /* Write original data - Batch write to avoid writing too much data at once */
        uint32_t bytes_per_batch = 64 * sizeof(float); /* Write 64 floats per batch (256 bytes) */
        uint32_t batches = SIGNAL_LEN * sizeof(float) / bytes_per_batch;
        
        for (uint32_t batch = 0; batch < batches; batch++) {
            uint32_t offset = batch * 64;
            res = f_write(&original_file, &record->original_signal[offset], 
                         bytes_per_batch, &bw);
            if (res != FR_OK || bw != bytes_per_batch) {
                printf("Failed (batch%d, error%d, written%d bytes)\r\n", batch, res, bw);
                return -1;
            }
        }
        printf("Success, ");
        
        printf("Reconstructed signal...");
        /* Write reconstructed data - Also batch write */
        for (uint32_t batch = 0; batch < batches; batch++) {
            uint32_t offset = batch * 64;
            res = f_write(&reconstructed_file, &record->reconstructed_signal[offset], 
                         bytes_per_batch, &bw);
            if (res != FR_OK || bw != bytes_per_batch) {
                printf("Failed (batch%d, error%d, written%d bytes)\r\n", batch, res, bw);
                return -1;
            }
        }
        printf("Success\r\n");
        
        /* Sync immediately after each window write */
        f_sync(&original_file);
        f_sync(&reconstructed_file);
        
        /* Give SD card some time */
        HAL_Delay(1);
    }
    
    printf("Signal cache flush completed: %d windows written to SD card\r\n", g_logger->signal_cache_count);
    
    /* Clear cache */
    g_logger->signal_cache_count = 0;
    return 0;
}

/* ================ Tension Result Logging ================ */

int log_tension_result(const tension_result_t* result,
                      uint32_t window_id,
                      float processing_time_ms,
                      bool reconstruction_used,
                      float packet_loss_rate) {
    
    if (!g_logger || !g_logger->logging_enabled || !result) return 0;
    
    /* Check cache space */
    if (g_logger->result_cache_count >= RESULT_CACHE_SIZE) {
        if (flush_result_cache() != 0) {
            printf("Result cache flush failed\r\n");
            return -1;
        }
    }
    
    /* Add to cache */
    tension_record_t* record = &g_logger->result_cache[g_logger->result_cache_count];
    
    record->window_id = window_id;
    record->timestamp = result->timestamp;
    record->tension = result->tension;
    record->frequency = result->frequency;
    record->dominant_mode = result->dominant_mode;
    record->confidence = result->confidence;
    record->processing_time_ms = processing_time_ms;
    record->reconstruction_used = reconstruction_used;
    record->packet_loss_rate = packet_loss_rate;
    
    g_logger->result_cache_count++;
    
    return 0;
}

static int flush_result_cache(void) {
    if (!g_logger || g_logger->result_cache_count == 0) return 0;
    
    /* Ensure files are opened */
    if (!files_opened && open_data_files() != 0) {
        return -1;
    }
    
    char line_buffer[256];
    UINT bw;
    FRESULT res;
    
    printf("Flushing result cache (%d results)...\r\n", g_logger->result_cache_count);
    
    /* Write CSV data */
    for (uint32_t i = 0; i < g_logger->result_cache_count; i++) {
        tension_record_t* record = &g_logger->result_cache[i];
        
        /* Format CSV line */
        sprintf(line_buffer, "%u,%u,%.3f,%.3f,%u,%.4f,%.2f,%s,%.2f\r\n",
                record->window_id,
                record->timestamp,
                record->tension,
                record->frequency,
                record->dominant_mode,
                record->confidence,
                record->processing_time_ms,
                record->reconstruction_used ? "YES" : "NO",
                record->packet_loss_rate);
        
        /* Write to file */
        res = f_write(&tension_file, line_buffer, strlen(line_buffer), &bw);
        if (res != FR_OK) {
            printf("Tension result write failed: %d\r\n", res);
            return -1;
        }
    }
    
    /* Sync to SD card */
    f_sync(&tension_file);
    
    g_logger->total_records_written += g_logger->result_cache_count;
    printf("Result cache flush completed: %d results written to SD card (Total: %d)\r\n", 
           g_logger->result_cache_count, g_logger->total_records_written);
    
    /* Clear cache */
    g_logger->result_cache_count = 0;
    return 0;
}

/* ================ Cache Management ================ */

int flush_all_caches(void) {
    int result = 0;
    
    printf("Flushing all caches to SD card...\r\n");
    
    if (flush_signal_cache() != 0) {
        printf("Signal cache flush failed\r\n");
        result = -1;
    }
    
    if (flush_result_cache() != 0) {
        printf("Result cache flush failed\r\n");
        result = -1;
    }
    
    if (result == 0) {
        printf("All caches successfully flushed to SD card\r\n");
    }
    
    return result;
}

/* ================ Analysis Logging ================ */

int create_analysis_header(void) {
    FIL log_file;
    FRESULT res;
    
    res = f_open(&log_file, ANALYSIS_LOG_FILE, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("Cannot create analysis log file: %d\r\n", res);
        return -1;
    }
    
    char header[512];
    sprintf(header, 
            "=== STM32 Cable Tension Analysis Log ===\r\n"
            "System: STM32F407 Single Sensor Version\r\n"
            "Date: %s\r\n"
            "Signal Length: %d samples\r\n"
            "Sampling Frequency: %.1f Hz\r\n"
            "Cable Length: %.1f m\r\n"
            "Cable Density: %.1f kg/m\r\n"
            "Design Tension: %.1f kN\r\n"
            "=========================================\r\n\r\n",
            __DATE__ " " __TIME__,
            SIGNAL_LEN,
            FS,
            CABLE_LENGTH,
            CABLE_DENSITY,
            DESIGN_TENSION);
    
    UINT bw;
    res = f_write(&log_file, header, strlen(header), &bw);
    if (res != FR_OK) {
        printf("Analysis header file write failed: %d\r\n", res);
        f_close(&log_file);
        return -1;
    }
    
    f_close(&log_file);
    printf("Analysis log file created successfully: %s\r\n", ANALYSIS_LOG_FILE);
    
    return 0;
}

int append_analysis_summary(uint32_t total_windows, 
                           float avg_tension, 
                           float std_tension,
                           float avg_processing_time) {
    FIL log_file;
    FRESULT res;
    
    res = f_open(&log_file, ANALYSIS_LOG_FILE, FA_OPEN_APPEND | FA_WRITE);
    if (res != FR_OK) {
        printf("Cannot open analysis log file: %d\r\n", res);
        return -1;
    }
    
    char summary[512];
    sprintf(summary,
            "Analysis Summary:\r\n"
            "Total Windows Processed: %u\r\n"
            "Average Tension: %.3f kN\r\n"
            "Standard Deviation: %.3f kN\r\n"
            "Average Processing Time: %.2f ms\r\n"
            "Design Tension Error: %.2f%%\r\n"
            "Total Records Logged: %u\r\n"
            "Analysis Completed: %s\r\n"
            "=========================================\r\n",
            total_windows,
            avg_tension,
            std_tension,
            avg_processing_time,
            fabsf(avg_tension - DESIGN_TENSION) / DESIGN_TENSION * 100,
            g_logger ? g_logger->total_records_written : 0,
            __DATE__ " " __TIME__);
    
    UINT bw;
    res = f_write(&log_file, summary, strlen(summary), &bw);
    if (res != FR_OK) {
        printf("Analysis summary write failed: %d\r\n", res);
        f_close(&log_file);
        return -1;
    }
    
    f_close(&log_file);
    printf("Analysis summary written to log\r\n");
    return 0;
}

/* ================ Utility Functions ================ */

uint32_t get_total_logged_records(void) {
    return g_logger ? g_logger->total_records_written + g_logger->result_cache_count : 0;
}

bool is_logging_enabled(void) {
    return g_logger ? g_logger->logging_enabled : false;
}

float get_estimated_file_size_mb(void) {
    if (!g_logger) return 0.0f;
    
    uint32_t total_records = g_logger->total_records_written + g_logger->result_cache_count;
    
    /* Size estimation per record:
     * Original signal: 256 * 4 = 1024 bytes
     * Reconstructed signal: 256 * 4 = 1024 bytes  
     * CSV line: approximately 80 bytes
     */
    float size_per_record = 2048.0f + 80.0f;
    float total_size_bytes = total_records * size_per_record;
    
    return total_size_bytes / (1024.0f * 1024.0f);
}

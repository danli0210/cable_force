/**
 ****************************************************************************************************
 * @file        data_logger.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-06-01
 * @brief       Data Logging Module Header
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

#ifndef DATA_LOGGER_H
#define DATA_LOGGER_H

#include "./FATFS/source/ff.h"
#include <stdint.h>
#include <stdbool.h>
#include "./TENSION/tension.h"

/* File name configuration */
#define ORIGINAL_DATA_FILE      "0:original_data.bin"
#define RECONSTRUCTED_DATA_FILE "0:reconstructed_data.bin"
#define TENSION_RESULTS_FILE    "0:tension_results.csv"
#define ANALYSIS_LOG_FILE       "0:analysis_log.txt"

/* Cache configuration - Memory usage optimization */
#ifndef SIGNAL_CACHE_SIZE
	#define SIGNAL_CACHE_SIZE       2       /* Cache 2 signal windows before writing */
#endif

#ifndef RESULT_CACHE_SIZE
	#define RESULT_CACHE_SIZE       10      /* Cache 10 results before writing */
#endif

/* Data recording structures */
typedef struct {
    float original_signal[SIGNAL_LEN];
    float reconstructed_signal[SIGNAL_LEN];
    uint8_t signal_mask[SIGNAL_LEN];
    uint32_t window_id;
    uint32_t timestamp;
    bool has_packet_loss;
    float packet_loss_rate;
} signal_record_t;

typedef struct {
    uint32_t window_id;
    uint32_t timestamp;
    float tension;
    float frequency;
    uint8_t dominant_mode;
    float confidence;
    float processing_time_ms;
    bool reconstruction_used;
    float packet_loss_rate;
} tension_record_t;

/* Cache management structure */
typedef struct {
    signal_record_t signal_cache[SIGNAL_CACHE_SIZE];
    tension_record_t result_cache[RESULT_CACHE_SIZE];
    uint32_t signal_cache_count;
    uint32_t result_cache_count;
    bool logging_enabled;
    uint32_t total_records_written;
} data_logger_t;

/* Function declarations */
#ifdef __cplusplus
extern "C" {
#endif

/* Initialization and cleanup */
int data_logger_init(void);
int data_logger_cleanup(void);
void data_logger_enable(bool enable);

/* Data recording */
int log_signal_data(const float* original_signal, 
                   const float* reconstructed_signal,
                   const uint8_t* mask,
                   uint32_t window_id,
                   bool has_packet_loss,
                   float packet_loss_rate);

int log_tension_result(const tension_result_t* result,
                      uint32_t window_id,
                      float processing_time_ms,
                      bool reconstruction_used,
                      float packet_loss_rate);

/* Force flush cache to files */
int flush_all_caches(void);

/* Utility functions */
int create_analysis_header(void);
int append_analysis_summary(uint32_t total_windows, 
                           float avg_tension, 
                           float std_tension,
                           float avg_processing_time);

/* Status queries */
uint32_t get_total_logged_records(void);
bool is_logging_enabled(void);
float get_estimated_file_size_mb(void);

#ifdef __cplusplus
}
#endif

#endif // DATA_LOGGER_H
						   
						   
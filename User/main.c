/**
 ****************************************************************************************************
 * @file        main.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V2.0
 * @date        2025-07-16
 * @brief       STM32F407 Cable Tension Analysis System - Real-time Single Sensor with Data Logging
 * @copyright   Copyright (c) 2025 Southeast University. All rights reserved.
 * @license     This software is provided for academic and research purposes only.
 *              Commercial use requires explicit permission from the author.
 ****************************************************************************************************
 * @attention
 *
 * Key Features:
 * - Timer-triggered data acquisition and analysis
 * - Real-time single sensor tension estimation
 * - Serial port real-time data analysis output
 * - Continuous processing simulation for 1-minute data
 * - Raw and reconstructed signal storage to SD card
 * - Tension analysis results storage to SD card
 *
 ****************************************************************************************************
 */

// Required header files
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/KEY/key.h"
#include "./BSP/TIMER/gtim.h"
#include "./CMSIS/DSP/Include/arm_math.h"
#include "./BSP/SRAM/sram.h"
#include "./BSP/SDIO/sdio_sdcard.h"
#include "./FATFS/exfuns/exfuns.h"
#include "./MALLOC/malloc.h"
#include "./TENSION/tension.h"
#include "./DATALOG/data_logger.h"
#include "config.h"  // System configuration file
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

// ================ Global Variables ================

// Data file paths
// static const char* signal_file = "0:test_signal.bin";
// static const char* mask_file = "0:test_mask.bin";

static const char* signal_file = "0:real_signal.bin";
static const char* mask_file = "0:real_mask.bin";

// Single sensor data buffer
static float single_sensor_data[SIGNAL_LEN];
static uint8_t single_sensor_mask[SIGNAL_LEN];

// Real-time analysis system - using external SRAM for large data storage
static float *full_data_buffer = NULL;  // Store 1-minute data (60s * 100Hz * 10) - external SRAM allocation
static uint8_t *full_mask_buffer = NULL; // Store corresponding mask data - external SRAM allocation
static uint32_t total_data_length = 0;
static uint32_t current_read_position = 0;
static bool analysis_active = false;

// Data logging related
static bool logging_enabled = true;
static float *temp_reconstructed_buffer = NULL; // Temporary reconstructed signal buffer

// Tension analysis results
static tension_result_t latest_result;
static realtime_stats_t system_stats;
static uint32_t analysis_counter = 0;

// Timer trigger
volatile bool timer_trigger = false;

// Performance statistics
static float total_processing_time = 0.0f;
static uint32_t successful_analyses = 0;

// ================ Function Declarations ================

void system_init(void);
void system_cleanup(void);
int load_full_data_from_sd(void);
void process_realtime_data(void);
void update_and_display_results(void);
void print_system_status(void);
void reset_analysis_system(void);
void print_logging_status(void);
void finalize_analysis_session(void);

// ================ Main Function ================

int main(void)
{
    uint8_t key;
    uint32_t led_counter = 0;
    uint32_t last_flush_time = 0;
    
    // Initialize system
    system_init();
    
    printf("\r\n========================================\r\n");
    printf("STM32F407 Cable Tension Analysis System - Real-time Single Sensor + Data Logging\r\n");
    printf("Timer-based Continuous Data Analysis and Storage\r\n");
    printf("========================================\r\n");
    
    printf("\r\nOperation Instructions:\r\n");
    printf("KEY0 - Load data files and start real-time analysis\r\n");
    printf("KEY1 - Stop analysis and reset system\r\n");
    printf("WK_UP - Enable/Disable data logging to SD card\r\n");
    printf("Timer automatically triggers data reading and analysis\r\n\r\n");
    
    // Main loop
    while (1)
    {
        key = key_scan(0);
        
        switch(key) {
            case KEY0_PRES:
                printf("\r\n[KEY0] Starting data file loading and real-time analysis...\r\n");
                LED0(0); // Turn on LED to indicate processing start
                
                // Load complete data file
                if (load_full_data_from_sd() == 0) {
                    printf("Data loaded successfully, starting timer for real-time analysis\r\n");
                    analysis_active = true;
                    current_read_position = 0;
                    analysis_counter = 0;
                    total_processing_time = 0.0f;
                    successful_analyses = 0;
                    
                    // If logging is enabled, initialize data logger
                    if (logging_enabled) {
                        data_logger_enable(true);
                        printf("Data logging enabled\r\n");
                    }
                    
                    // Start timer - trigger every 100ms (simulate real-time data)
                    gtim_timx_int_init(9999, 8399); // 100ms @ 84MHz
                } else {
                    printf("Data file loading failed\r\n");
                }
                
                LED0(1); // Turn off LED
                break;
                
            case KEY1_PRES:
                printf("\r\n[KEY1] Stopping analysis and resetting system...\r\n");
                analysis_active = false;
                finalize_analysis_session();
                reset_analysis_system();
                printf("System has been reset\r\n");
                break;
                
            case WKUP_PRES:
                logging_enabled = !logging_enabled;
                printf("\r\n[WK_UP] Data logging: %s\r\n", 
                       logging_enabled ? "Enabled" : "Disabled");
                if (logging_enabled && !analysis_active) {
                    printf("Please start analysis first, then data logging will be automatically activated\r\n");
                }
                break;
                
            default:
                // No key pressed processing
                break;
        }
        
        // Check timer trigger
        if (timer_trigger && analysis_active) {
            timer_trigger = false;
            process_realtime_data();
        }
        
        // Periodically flush data cache to SD card (every 5 seconds)
        if (logging_enabled && analysis_active && 
            (HAL_GetTick() - last_flush_time > 5000)) {
            flush_all_caches();
            last_flush_time = HAL_GetTick();
        }
        
        // LED blinking indicates system running
        led_counter++;
        if (led_counter % 1000 == 0) {
            if (analysis_active) {
                LED0_TOGGLE(); // Blink during analysis
                if (led_counter % 10000 == 0) {
                    print_logging_status();
                }
            } else {
                led_counter = 0;
                LED0(1); // Keep off when idle
            }
        }
        
        delay_ms(1);
    }
}

// ================ System Initialization ================

void system_init(void)
{
    HAL_Init();                             // Initialize HAL library
    sys_stm32_clock_init(336, 8, 2, 7);     // Set clock to 168MHz
    delay_init(168);                        // Initialize delay function
    usart_init(115200);                     // Initialize USART
    led_init();                             // Initialize LED
    key_init();                             // Initialize keys
    sram_init();                            // Initialize external SRAM
    
    // Initialize memory pools
    my_mem_init(SRAMIN);                    // Internal SRAM memory pool
    my_mem_init(SRAMEX);                    // External SRAM memory pool
    my_mem_init(SRAMCCM);                   // CCM memory pool
    
    // Allocate external SRAM for large data cache
    full_data_buffer = (float *)mymalloc(SRAMEX, 60000 * sizeof(float));
    full_mask_buffer = (uint8_t *)mymalloc(SRAMEX, 60000 * sizeof(uint8_t));
    temp_reconstructed_buffer = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));
    
    if (full_data_buffer == NULL || full_mask_buffer == NULL || 
        temp_reconstructed_buffer == NULL) {
        printf("External SRAM allocation failed!\r\n");
        while(1); // Stop execution
    }
    printf("External SRAM cache allocated successfully \r\n");
	
	// detect_maximum_signal_length();
    
    // Initialize SD card and file system
    sd_init();
    exfuns_init();
    f_mount(fs[0], "0:", 1);                // Mount SD card
    
    // Initialize tension analysis system
    tension_init();
    realtime_processor_init();
    
    // Initialize data logger
    if (data_logger_init() != 0) {
        printf("Data logger initialization failed\r\n");
    }
    
    // Clear data buffers
    memset(single_sensor_data, 0, sizeof(single_sensor_data));
    memset(single_sensor_mask, 1, sizeof(single_sensor_mask));
    memset(full_data_buffer, 0, 60000 * sizeof(float));
    memset(full_mask_buffer, 1, 60000 * sizeof(uint8_t)); // Default no packet loss
    memset(temp_reconstructed_buffer, 0, SIGNAL_LEN * sizeof(float));
    memset(&system_stats, 0, sizeof(system_stats));
    
    printf("System initialization completed!\r\n");
}

// ================ SD Card Data Loading ================

int load_full_data_from_sd(void)
{
    FIL file;
    UINT br;
    
    printf("Loading complete data and mask files...\r\n");
    
    // 1. Read signal file
    if (f_open(&file, signal_file, FA_READ) != FR_OK) {
        printf("Cannot open signal file: %s\r\n", signal_file);
        return -1;
    }
    
    // Get file size
    FSIZE_t file_size = f_size(&file);
    total_data_length = file_size / sizeof(float);
    
    printf("Signal file size: %d bytes, data points: %d\r\n", (int)file_size, total_data_length);
    
    // Check buffer size
    if (total_data_length > 60000) {
        printf("Warning: Data volume exceeds buffer, will truncate to first 60000 points\r\n");
        total_data_length = 60000;
    }
    
    // Read signal data to buffer
    UINT bytes_to_read = total_data_length * sizeof(float);
    if (f_read(&file, full_data_buffer, bytes_to_read, &br) != FR_OK) {
        printf("Signal file reading failed\r\n");
        f_close(&file);
        return -1;
    }
    f_close(&file);
    printf("Signal data loaded successfully: %d data points (%d bytes)\r\n", total_data_length, br);
    
    // 2. Read mask file (optional)
    if (f_open(&file, mask_file, FA_READ) == FR_OK) {
        FSIZE_t mask_file_size = f_size(&file);
        uint32_t mask_length = mask_file_size / sizeof(uint8_t);
        
        printf("Mask file size: %d bytes, mask points: %d\r\n", (int)mask_file_size, mask_length);
        
        if (mask_length >= total_data_length) {
            UINT mask_bytes_to_read = total_data_length * sizeof(uint8_t);
            if (f_read(&file, full_mask_buffer, mask_bytes_to_read, &br) == FR_OK) {
                printf("Mask data loaded successfully: %d bytes\r\n", br);
                
                // Count packet loss
                uint32_t lost_count = 0;
                for (uint32_t i = 0; i < total_data_length; i++) {
                    if (full_mask_buffer[i] == 0) {
                        lost_count++;
                    }
                }
                float loss_rate = (float)lost_count / total_data_length * 100.0f;
                printf("Data loss rate: %.2f%% (%d/%d)\r\n", loss_rate, lost_count, total_data_length);
            } else {
                printf("Mask file reading failed, using default mask (no packet loss)\r\n");
                memset(full_mask_buffer, 1, total_data_length);
            }
        } else {
            printf("Mask file length mismatch, using default mask (no packet loss)\r\n");
            memset(full_mask_buffer, 1, total_data_length);
        }
        f_close(&file);
    } else {
        printf("Mask file does not exist: %s, using default mask (no packet loss)\r\n", mask_file);
        memset(full_mask_buffer, 1, total_data_length);
    }
    
    printf("Total analysis duration: %.1f seconds (based on 100ms interval)\r\n", 
           (float)(total_data_length / SIGNAL_LEN) * 0.1f);
    
    return 0;
}

// ================ Real-time Data Processing ================

void process_realtime_data(void)
{
    // Check if there's still data to read
    if (current_read_position + SIGNAL_LEN >= total_data_length) {
        printf("\r\n=== Data Processing Completed ===\r\n");
        printf("Total processed data windows: %d\r\n", analysis_counter);
        analysis_active = false;
        finalize_analysis_session();
        return;
    }
    
    // Copy current window data from buffer
    memcpy(single_sensor_data, 
           &full_data_buffer[current_read_position], 
           SIGNAL_LEN * sizeof(float));
    
    // Copy corresponding mask data from buffer
    memcpy(single_sensor_mask,
           &full_mask_buffer[current_read_position],
           SIGNAL_LEN * sizeof(uint8_t));
    
    // Check packet loss in current window
    float window_loss_rate;
    bool has_window_loss = detect_packet_loss(single_sensor_mask, SIGNAL_LEN, &window_loss_rate);
    
    printf("\r\n--- Window %d ---\r\n", analysis_counter + 1);
    printf("Data position: %d - %d", 
           current_read_position, current_read_position + SIGNAL_LEN - 1);
    if (has_window_loss) {
        printf(" | Loss rate: %.1f%%", window_loss_rate);
    }
    printf("\r\n");
    
    // Execute single sensor tension analysis
    uint32_t start_time = HAL_GetTick();
    
    int result = process_single_sensor_data(single_sensor_data, 
                                          single_sensor_mask,
                                          &latest_result, 
                                          &system_stats);
    
    uint32_t processing_time = HAL_GetTick() - start_time;
    
    if (result == 0) {
        // Set timestamp
        latest_result.timestamp = analysis_counter;
        successful_analyses++;
        total_processing_time += processing_time;
        
        printf("Analysis completed: processing time %d ms\r\n", processing_time);
        
        // Real-time display results
        update_and_display_results();
        
        // If logging is enabled, save data
        if (logging_enabled) {
            // Get reconstructed signal - from tension analysis module
            float* reconstructed_signal = get_reconstructed_signal();
            
            if (reconstructed_signal) {
                // Log signal data
                printf("Logging signal data - window %d\r\n", analysis_counter);
                int log_signal_result = log_signal_data(single_sensor_data, 
                               reconstructed_signal,
                               single_sensor_mask,
                               analysis_counter,
                               has_window_loss,
                               window_loss_rate);
                
                if (log_signal_result != 0) {
                    printf("Signal data logging failed\r\n");
                }
                
                // Log tension results
                int log_result_result = log_tension_result(&latest_result,
                                  analysis_counter,
                                  (float)processing_time,
                                  has_window_loss,
                                  window_loss_rate);
                
                if (log_result_result != 0) {
                    printf("Tension result logging failed\r\n");
                } else {
                    printf("Data logging successful\r\n");
                }
            } else {
                printf("Warning: Unable to get reconstructed signal data\r\n");
            }
        }
        
    } else {
        printf("Analysis failed (error code: %d)\r\n", result);
    }
    
    // Update position and counter
    current_read_position += WINDOW_STRIDE; // Use window stride
    analysis_counter++;
    
    // Display statistics every 10 analyses
    if (analysis_counter % 10 == 0) {
        print_system_status();
    }
}

// ================ Result Update and Display ================

void update_and_display_results(void)
{
    printf("Tension: %.2f kN | Frequency: %.2f Hz | Mode: %d | Confidence: %.3f\r\n",
           latest_result.tension,
           latest_result.frequency, 
           latest_result.dominant_mode,
           latest_result.confidence);
    
    // Check result reasonableness
    if (latest_result.tension > 0 && latest_result.tension < 15000) {
        float error = fabsf(latest_result.tension - DESIGN_TENSION) / DESIGN_TENSION * 100;
        if (error < 5.0f) {
            printf("Result normal (error: %.1f%%)\r\n", error);
        } else if (error < 10.0f) {
            printf("Large error (error: %.1f%%)\r\n", error);
        } else {
            printf("Abnormal result (error: %.1f%%)\r\n", error);
        }
    } else {
        printf("Tension value exceeds reasonable range\r\n");
    }
}

void print_system_status(void)
{
    if (system_stats.valid_samples > 0) {
        printf("\r\n=== System Status (Analysis #%d) ===\r\n", analysis_counter);
        printf("Average tension: %.2f kN\r\n", system_stats.mean_tension);
        printf("Standard deviation: %.2f kN (%.1f%%)\r\n", 
               system_stats.std_tension,
               system_stats.std_tension / system_stats.mean_tension * 100);
        printf("Design tension: %.1f kN\r\n", DESIGN_TENSION);
        printf("Overall error: %.2f%%\r\n", 
               fabsf(system_stats.mean_tension - DESIGN_TENSION) / DESIGN_TENSION * 100);
        printf("Processing progress: %.1f%% (%d/%d)\r\n",
               (float)current_read_position / total_data_length * 100,
               current_read_position, total_data_length);
        
        if (successful_analyses > 0) {
            printf("Average processing time: %.1f ms\r\n", total_processing_time / successful_analyses);
        }
        
        if (logging_enabled) {
            printf("Logged: %d records, file size: %.1f MB\r\n",
                   get_total_logged_records(), get_estimated_file_size_mb());
        }
        
        printf("================================\r\n");
    }
}

void print_logging_status(void)
{
    if (logging_enabled && analysis_active) {
        printf("\r\n[Data Logging] Status: Active | Logged: %d | File size: %.1f MB\r\n",
               get_total_logged_records(), get_estimated_file_size_mb());
    }
}

// ================ Analysis Session Finalization ================

void finalize_analysis_session(void)
{
    if (logging_enabled) {
        printf("\r\n Finalizing data logging...\r\n");
        
        // Flush all caches
        flush_all_caches();
        
        // Write analysis summary
        if (successful_analyses > 0) {
            float avg_processing_time = total_processing_time / successful_analyses;
            append_analysis_summary(analysis_counter,
                                  system_stats.mean_tension,
                                  system_stats.std_tension,
                                  avg_processing_time);
        }
        
        // Disable data logging
        data_logger_enable(false);
        
        printf("Data logging completed and saved to SD card\r\n");
        printf("File list:\r\n");
        printf("- %s (Original signal data)\r\n", ORIGINAL_DATA_FILE);
        printf("- %s (Reconstructed signal data)\r\n", RECONSTRUCTED_DATA_FILE);
        printf("- %s (Tension analysis results)\r\n", TENSION_RESULTS_FILE);
        printf("- %s (Analysis log)\r\n", ANALYSIS_LOG_FILE);
    }
}

// ================ System Reset ================

void reset_analysis_system(void)
{
    analysis_active = false;
    current_read_position = 0;
    analysis_counter = 0;
    timer_trigger = false;
    total_processing_time = 0.0f;
    successful_analyses = 0;
    
    // Stop timer
    gtim_timx_stop();
    
    // Reset statistics
    memset(&system_stats, 0, sizeof(system_stats));
    memset(&latest_result, 0, sizeof(latest_result));
    
    printf("Analysis system has been reset\r\n");
}

// ================ System Cleanup ================

void system_cleanup(void)
{
    if (analysis_active) {
        finalize_analysis_session();
    }
    
    // Cleanup data logger
    data_logger_cleanup();
    
    // Free memory
    if (full_data_buffer) {
        myfree(SRAMEX, full_data_buffer);
        full_data_buffer = NULL;
    }
    if (full_mask_buffer) {
        myfree(SRAMEX, full_mask_buffer);
        full_mask_buffer = NULL;
    }
    if (temp_reconstructed_buffer) {
        myfree(SRAMEX, temp_reconstructed_buffer);
        temp_reconstructed_buffer = NULL;
    }
    
    printf("System cleanup completed\r\n");
}

// ================ Timer Interrupt Callback ================

/**
 * @brief Timer interrupt callback function - used to trigger real-time analysis
 * @note  Called in HAL_TIM_PeriodElapsedCallback function in gtim.c
 */
void trigger_realtime_analysis(void)
{
    if (analysis_active) {
        timer_trigger = true;
        LED1_TOGGLE(); // LED1 indicates timer trigger
    }
}

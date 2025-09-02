/**
 ****************************************************************************************************
 * @file        config.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-07-16
 * @brief       System Configuration Header for Cable Tension Analysis System
 * @copyright   Copyright (c) 2025 Southeast University. All rights reserved.
 * @license     This software is provided for academic and research purposes only.
 *              Commercial use requires explicit permission from the author.
 ****************************************************************************************************
 */

#ifndef CONFIG_H
#define CONFIG_H

// ================ System Function Switches ================

// Data logging function control
#define ENABLE_DATA_LOGGING         1       // Enable data logging functionality
#define ENABLE_SIGNAL_LOGGING       1       // Enable signal data logging
#define ENABLE_RESULT_LOGGING       1       // Enable result data logging
#define ENABLE_ANALYSIS_LOGGING     1       // Enable analysis log recording

// Debug function control
#define ENABLE_DEBUG_OUTPUT         1       // Enable debug output
#define ENABLE_PERFORMANCE_TEST     1       // Enable performance testing
#define ENABLE_VALIDATION_TEST      1       // Enable validation testing

// Memory optimization control
#define USE_EXTERNAL_SRAM           1       // Use external SRAM
#define OPTIMIZE_FOR_MEMORY         1       // Memory optimization mode

// ================ Memory Allocation Strategy ================

// Buffer size configuration (adjust according to available memory)
#if OPTIMIZE_FOR_MEMORY
    // Memory-constrained mode - reduce buffer sizes
    #define SIGNAL_CACHE_SIZE       2       // Cache 2 signal windows
    #define RESULT_CACHE_SIZE       10      // Cache 10 results
    #define MAX_DATA_BUFFER_SIZE    30000   // Maximum data buffer size (~30 seconds data)
#else
    // Standard mode
    #define SIGNAL_CACHE_SIZE       4       // Cache 4 signal windows
    #define RESULT_CACHE_SIZE       20      // Cache 20 results
    #define MAX_DATA_BUFFER_SIZE    60000   // Maximum data buffer size (~60 seconds data)
#endif

// ================ File System Configuration ================

// File naming configuration
#define USE_TIMESTAMP_IN_FILENAME   0       // Include timestamp in filename
#define AUTO_INCREMENT_FILENAME     1       // Auto-increment filename

// File size limits (MB)
#define MAX_SIGNAL_FILE_SIZE        10.0f   // Maximum signal file size
#define MAX_RESULT_FILE_SIZE        1.0f    // Maximum result file size

// ================ Data Logging Configuration ================

// Recording precision control
#define TENSION_PRECISION           4       // Tension recording precision (decimal places)
#define FREQUENCY_PRECISION         4       // Frequency recording precision
#define TIME_PRECISION              2       // Time recording precision

// Data compression options
#define ENABLE_DATA_COMPRESSION     0       // Enable data compression (requires additional memory)

// ================ Performance Optimization Configuration ================

// Processing frequency control
#define ANALYSIS_INTERVAL_MS        100     // Analysis interval (milliseconds)
#define FLUSH_INTERVAL_SEC          5       // Cache flush interval (seconds)
#define STATUS_REPORT_INTERVAL      10      // Status report interval (number of analyses)

// ARM DSP optimization
#define USE_ARM_DSP_OPTIMIZATION    1       // Use ARM DSP optimization
#define FFT_USE_HARDWARE_ACC        0       // Use hardware FFT acceleration (if available)

// ================ Error Handling Configuration ================

// Error recovery strategy
#define AUTO_RETRY_ON_ERROR         1       // Auto retry on error
#define MAX_RETRY_COUNT             3       // Maximum retry count
#define ERROR_RECOVERY_DELAY_MS     100     // Error recovery delay

// Data integrity checking
#define ENABLE_DATA_CHECKSUM        0       // Enable data checksum (uses additional memory)
#define ENABLE_FILE_INTEGRITY_CHECK 1       // Enable file integrity check

// ================ Debug and Diagnostic Configuration ================

#if ENABLE_DEBUG_OUTPUT
    #define DEBUG_PRINTF(fmt, ...)  printf("[DEBUG] " fmt, ##__VA_ARGS__)
    #define INFO_PRINTF(fmt, ...)   printf("[INFO] " fmt, ##__VA_ARGS__)
    #define WARN_PRINTF(fmt, ...)   printf("[WARN] " fmt, ##__VA_ARGS__)
    #define ERROR_PRINTF(fmt, ...)  printf("[ERROR] " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINTF(fmt, ...)  do {} while(0)
    #define INFO_PRINTF(fmt, ...)   do {} while(0)
    #define WARN_PRINTF(fmt, ...)   do {} while(0)
    #define ERROR_PRINTF(fmt, ...)  printf("[ERROR] " fmt, ##__VA_ARGS__)
#endif

// Memory usage reporting
#define ENABLE_MEMORY_REPORT        1       // Enable memory usage reporting
#define MEMORY_REPORT_INTERVAL      50      // Memory report interval (number of analyses)

// ================ Compatibility Configuration ================

// Backward compatibility options
#define MAINTAIN_LEGACY_INTERFACE   1       // Maintain legacy interface compatibility
#define SUPPORT_MULTI_SENSOR_API    1       // Support multi-sensor API (compatibility)

// SD card compatibility
#define SD_CARD_SPEED_CLASS         4       // SD card speed class requirement (Class 4 and above)
#define USE_FATFS_LONG_FILENAME     1       // Use long filename support

// ================ Version Information ================

#define SYSTEM_VERSION_MAJOR        1
#define SYSTEM_VERSION_MINOR        0
#define SYSTEM_VERSION_PATCH        0
#define SYSTEM_BUILD_DATE           __DATE__ " " __TIME__

// Version string generation macros
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define SYSTEM_VERSION_STRING       TOSTRING(SYSTEM_VERSION_MAJOR) "." \
                                   TOSTRING(SYSTEM_VERSION_MINOR) "." \
                                   TOSTRING(SYSTEM_VERSION_PATCH)

// ================ Conditional Compilation Checks ================

// Check configuration consistency
#if ENABLE_DATA_LOGGING && !ENABLE_SIGNAL_LOGGING && !ENABLE_RESULT_LOGGING
    #error "Data logging functionality is enabled but no logging type is selected"
#endif

#if OPTIMIZE_FOR_MEMORY && SIGNAL_CACHE_SIZE > 2
    #warning "Consider reducing cache size in memory optimization mode"
#endif

#if !USE_EXTERNAL_SRAM && MAX_DATA_BUFFER_SIZE > 10000
    #error "Data buffer size is too large when external SRAM is not used"
#endif

// ================ Platform-Specific Configuration ================

// STM32F407 specific configuration
#ifdef STM32F407xx
    #define EXTERNAL_SRAM_SIZE      (1024 * 1024)  // 1MB external SRAM
    #define INTERNAL_SRAM_SIZE      (128 * 1024)   // 128KB internal SRAM
    #define CCM_SRAM_SIZE           (64 * 1024)    // 64KB CCM SRAM
#endif

#endif // CONFIG_H

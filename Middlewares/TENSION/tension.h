/**
 ****************************************************************************************************
 * @file        tension.h
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-07-16
 * @brief       Public API for cable tension estimation with CS-based reconstruction (DCT + OMP)
 * @copyright   Copyright (c) 2025 Southeast University.
 * @license     Academic/Research License. This software is provided for academic and research use only.
 *              Commercial use, redistribution, or sublicensing requires prior written permission
 *              from the copyright owner.
 * @ipnotice    Intellectual Property Notice: All source code, headers, and binaries are proprietary
 *              to Southeast University and the author. Unauthorized commercial use is prohibited.
 ****************************************************************************************************
 */

#ifndef TENSION_H
#define TENSION_H

#include "./CMSIS/DSP/Include/arm_math.h"
#include <stdint.h>
#include <stdbool.h>

#ifndef SIGNAL_LEN_MAX
#define SIGNAL_LEN_MAX          256     // Maximum possible signal length
#endif

#ifndef SIGNAL_LEN_MIN
#define SIGNAL_LEN_MIN          64      // Minimum acceptable signal length
#endif

#define DCT_BATCH_SIZE          16      // Number of rows processed per batch when generating DCT basis

// System Parameters - CRITICAL: WINDOW_SIZE must be a power of 2 for ARM FFT
#define SIGNAL_LEN              256     // Signal length (power of 2 for FFT)
#define NUM_SENSORS             4       // Number of wireless sensors
#define K_SPARSITY              15      // Sparsity level for CS reconstruction
#define WINDOW_SIZE             128     // Sliding window size (MUST be power of 2)
#define WINDOW_STRIDE           64      // Window stride (50% overlap)
#define FS                      50.0f   // Sampling frequency [Hz]

// Cable Parameters (engineering parameters)
// Example sets (keep one active):
// #define CABLE_LENGTH         214.9f   // Cable length [m]
// #define CABLE_DENSITY        46.5f    // Linear mass density [kg/m]
// #define DESIGN_TENSION       3150.0f  // Design tension [kN]

// #define CABLE_LENGTH         22.5f
// #define CABLE_DENSITY        53.2f
// #define DESIGN_TENSION       4019.3f

#define CABLE_LENGTH            172.6760f  // Cable length [m]
#define CABLE_DENSITY           85.9f      // Linear mass density [kg/m]
#define DESIGN_TENSION          5760.1f    // Design tension [kN]

// Real-time Processing Parameters
#define FREQ_BUFFER_SIZE        100     // Buffer size for frequency history
#define TENSION_BUFFER_SIZE     100     // Buffer size for tension history
#define MAX_PACKET_LOSS         20      // Maximum packet loss percentage [%]

// Mode number limits
#define MIN_MODE_NUMBER         1       // Minimum valid mode number
#define MAX_MODE_NUMBER         50      // Maximum valid mode number

// Single-Sensor Configuration (kept for compatibility with multi-sensor interfaces)
typedef enum {
    SENSOR_MAIN = 0,    // Primary sensor
    SENSOR_BACKUP1,     // Backup sensor 1 (reserved for compatibility)
    SENSOR_BACKUP2,     // Backup sensor 2 (reserved for compatibility)
    SENSOR_BACKUP3      // Backup sensor 3 (reserved for compatibility)
} sensor_type_t;

// Data structures
typedef struct {
    float   data[SIGNAL_LEN];
    uint8_t mask[SIGNAL_LEN];
    bool    has_packet_loss;
    float   packet_loss_rate;   // [%]
    uint32_t timestamp;
} sensor_data_t;

typedef struct {
    sensor_data_t sensors[NUM_SENSORS];
    uint32_t      window_start;
    uint32_t      window_end;
    bool          is_valid;
} multi_sensor_window_t;

typedef struct {
    float   frequency;      // [Hz]
    float   amplitude;      // arbitrary units
    uint8_t mode_number;    // identified mode index
    float   confidence;     // 0..1
} dominant_mode_t;

typedef struct {
    float    tension;        // [kN]
    float    frequency;      // [Hz]
    uint8_t  dominant_mode;  // mode index used for tension calc
    float    confidence;     // 0..1
    uint32_t timestamp;
} tension_result_t;

typedef struct {
    float     freq_history[FREQ_BUFFER_SIZE];
    float     tension_history[TENSION_BUFFER_SIZE];
    uint32_t  history_index;
    uint32_t  valid_samples;
    float     mean_tension;  // [kN]
    float     std_tension;   // [kN]
} realtime_stats_t;

// Function declarations
#ifdef __cplusplus
extern "C" {
#endif

// Testing / capability probing
uint32_t detect_maximum_signal_length(void);

// Initialization
void tension_init(void);
void realtime_processor_init(void);

// Single sensor data processing
int process_single_sensor_data(const float* sensor_data, const uint8_t* mask,
                               tension_result_t* result, realtime_stats_t* stats);

// Packet loss detection and reconstruction
bool detect_packet_loss(const uint8_t* mask, uint32_t length, float* loss_rate);
void simulate_packet_loss(uint8_t* mask, uint32_t length, float loss_percentage);

// Compressive Sensing reconstruction
int cs_reconstruction_dct(const float* measurements, const uint8_t* mask,
                          float* reconstructed, uint32_t length);

// Simple interpolation reconstruction (fallback)
int simple_interpolation_reconstruction(const float* measurements, const uint8_t* mask,
                                       float* reconstructed, uint32_t length);

// Mode number / frequency utilities
uint8_t calculate_mode_number_from_frequency(float frequency);
float   calculate_theoretical_frequency(uint8_t mode_number, float tension_kN);

// Frequency analysis
float estimate_dominant_frequency_single(const float* signal, uint32_t length,
                                         dominant_mode_t* result);

// Tension estimation and statistics
float calculate_tension_from_frequency(float frequency, uint8_t mode_number);
void  update_realtime_statistics(realtime_stats_t* stats, float tension, float frequency);

// Main processing (multi-sensor-compatible interface)
int run_realtime_tension_estimation(
    const float   multi_sensor_input[NUM_SENSORS][SIGNAL_LEN],
    const uint8_t multi_sensor_masks[NUM_SENSORS][SIGNAL_LEN],
    tension_result_t* results,
    uint32_t*         num_results,
    realtime_stats_t* stats);

// Utility functions
void  print_tension_results(const tension_result_t* results, uint32_t count);
float calculate_reconstruction_error(const float* original, const float* reconstructed, uint32_t length);

// Data access helpers (for logging/inspection)
float* get_reconstructed_signal(void);
float* get_original_signal_buffer(void);

// Debug and validation
void validate_single_sensor_analysis(void);
void validate_cs_reconstruction(void);
void benchmark_single_sensor_processing(void);

#ifdef __cplusplus
}
#endif

#endif // TENSION_H

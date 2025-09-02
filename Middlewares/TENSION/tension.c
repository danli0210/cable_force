/**
 ****************************************************************************************************
 * @file        tension.c
 * @author      Dan Li (danli@seu.edu.cn)
 * @affiliation Southeast University
 * @version     V1.0
 * @date        2025-07-16
 * @brief       Cable tension estimation with CS-based reconstruction (DCT + OMP) and FFT peak picking
 * @copyright   Copyright (c) 2025 Southeast University.
 * @license     Academic/Research License. This software is provided for academic and research use only.
 *              Commercial use, redistribution, or sublicensing requires prior written permission
 *              from the copyright owner.
 * @ipnotice    Intellectual Property Notice: All source code, documentation, and compiled artifacts
 *              are proprietary to Southeast University and the author. Unauthorized commercial use is prohibited.
 ****************************************************************************************************
 * @attention
 *
 * Development Platform: STM32F407 Discovery/Explorer Board
 * Target MCU: STM32F407VGT6
 * System Core: ARM Cortex-M4 @ 168 MHz
 *
 * Revision History
 *   V1.0  2025-07-16  Initial public release
 *
 ****************************************************************************************************
 */

// === dct_omp_tension.c - Enhanced with Compressive Sensing Reconstruction ===
#include "./TENSION/tension.h"
#include "./BSP/SDIO/sdio_sdcard.h"
#include "./FATFS/exfuns/exfuns.h"
#include "./BSP/RNG/rng.h"
#include "./MALLOC/malloc.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

// Global variables for single sensor processing
static float *dct_coefficients = NULL;
static float *temp_signal_buffer = NULL;
static float *reconstructed_buffer = NULL;
static float *fft_buffer = NULL;
static float *fft_output = NULL;
static arm_rfft_fast_instance_f32 rfft_instance;

// DCT basis matrix for CS reconstruction
static float **dct_basis_matrix = NULL;
static float *cs_coefficients = NULL;

// Real-time processing variables
static realtime_stats_t global_stats;
static uint32_t processing_counter = 0;

// Memory requirement calculation
static size_t calculate_memory_requirement(uint32_t signal_len, bool use_dct_matrix) {
    size_t total = 0;

    // Basic buffers (6 float arrays)
    total += signal_len * sizeof(float) * 6;

    if (use_dct_matrix) {
        // DCT matrix (full precomputed)
        total += signal_len * sizeof(float*);              // Row pointers
        total += signal_len * signal_len * sizeof(float);  // Matrix data
    }

    // CS processing buffers (estimated)
    total += signal_len * sizeof(float) * 2; // Additional working buffers

    return total;
}

// Test memory availability by attempting allocation
uint32_t detect_maximum_signal_length(void) {
    uint32_t test_len = SIGNAL_LEN_MAX;

    printf("Probing available external SRAM capacity...\r\n");

    while (test_len >= SIGNAL_LEN_MIN) {
        size_t required_memory = calculate_memory_requirement(test_len, true);
        printf("Testing SIGNAL_LEN=%lu (requires %lu KB)...",
               (unsigned long)test_len, (unsigned long)(required_memory / 1024));

        // Try to allocate test memory
        void* test_ptr = mymalloc(SRAMEX, required_memory);
        if (test_ptr) {
            myfree(SRAMEX, test_ptr);
            printf(" OK\r\n");
            return test_len;
        } else {
            printf(" FAILED\r\n");
            test_len = test_len / 2;  // Try half the size
            if (test_len < SIGNAL_LEN_MIN) {
                test_len = SIGNAL_LEN_MIN;
            }
        }
    }

    return SIGNAL_LEN_MIN;
}

// ================ Data Access Functions ================

float* get_reconstructed_signal(void) {
    return reconstructed_buffer;
}

float* get_original_signal_buffer(void) {
    return temp_signal_buffer;
}

// ================ DCT Basis Matrix Generation ================

/*
static void generate_dct_basis_matrix(void) {
    if (!dct_basis_matrix) return;

    float scale0 = sqrtf(1.0f / SIGNAL_LEN);
    float scale = sqrtf(2.0f / SIGNAL_LEN);

    for (int k = 0; k < SIGNAL_LEN; k++) {
        for (int n = 0; n < SIGNAL_LEN; n++) {
            float angle = PI * (2 * n + 1) * k / (2.0f * SIGNAL_LEN);
            dct_basis_matrix[k][n] = (k == 0) ? scale0 * cosf(angle) : scale * cosf(angle);
        }
    }
}
*/

static int generate_dct_basis_matrix(void) {
    if (!dct_basis_matrix || SIGNAL_LEN == 0) return -1;

    float scale0 = sqrtf(1.0f / SIGNAL_LEN);
    float scale = sqrtf(2.0f / SIGNAL_LEN);

    printf("Generating DCT basis matrix %lux%lu (batch size: %u)...\r\n",
           (unsigned long)SIGNAL_LEN, (unsigned long)SIGNAL_LEN, (unsigned)DCT_BATCH_SIZE);

    float angle_increment = PI / (2.0f * SIGNAL_LEN);

    for (uint32_t batch_start = 0; batch_start < SIGNAL_LEN; batch_start += DCT_BATCH_SIZE) {
        uint32_t batch_end = (batch_start + DCT_BATCH_SIZE > SIGNAL_LEN) ?
                             SIGNAL_LEN : batch_start + DCT_BATCH_SIZE;

        printf("Processing batch: %lu-%lu/%lu (%.1f%%)\r\n",
               (unsigned long)batch_start, (unsigned long)(batch_end - 1),
               (unsigned long)SIGNAL_LEN, (float)batch_start * 100.0f / SIGNAL_LEN);

        // Process current batch
        for (uint32_t k = batch_start; k < batch_end; k++) {
            float base_angle = angle_increment * k;

            for (uint32_t n = 0; n < SIGNAL_LEN; n++) {
                float angle = base_angle * (2 * n + 1);
                float cos_val = cosf(angle);
                dct_basis_matrix[k][n] = (k == 0) ? scale0 * cos_val : scale * cos_val;
            }
        }

        // Watchdog refresh if enabled
        #ifdef USE_WATCHDOG
        HAL_IWDG_Refresh(&hiwdg);
        #endif

        // Yield some time to other tasks
        HAL_Delay(5);

        // Optional: stack usage check
        // printf("Stack usage: %d bytes\r\n", get_stack_usage());
    }

    printf("DCT basis matrix generation completed\r\n");
    return 0;
}

// ================ Initialization Functions ================

void tension_init(void) {
    // Allocate buffers in external SRAM
    dct_coefficients     = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));
    temp_signal_buffer   = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));
    reconstructed_buffer = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));
    fft_buffer           = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));
    fft_output           = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));
    cs_coefficients      = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));

    // Allocate DCT basis matrix
    dct_basis_matrix = (float **)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float *));
    if (dct_basis_matrix) {
        for (int i = 0; i < (int)SIGNAL_LEN; i++) {
            dct_basis_matrix[i] = (float *)mymalloc(SRAMEX, SIGNAL_LEN * sizeof(float));
        }
    }

    if (!dct_coefficients || !temp_signal_buffer || !reconstructed_buffer ||
        !fft_buffer || !fft_output || !dct_basis_matrix || !cs_coefficients) {
        printf("Initialization failed: insufficient memory for tension analysis!\r\n");
        return;
    }

    // Generate DCT basis matrix
    generate_dct_basis_matrix();

    // Initialize FFT instance
    arm_status status = arm_rfft_fast_init_f32(&rfft_instance, SIGNAL_LEN);
    if (status != ARM_MATH_SUCCESS) {
        printf("FFT initialization failed, status: %d\r\n", status);
        return;
    }

    printf("Single-sensor tension analysis system initialized\r\n");
}

void realtime_processor_init(void) {
    memset(&global_stats, 0, sizeof(realtime_stats_t));
    processing_counter = 0;
    printf("Real-time processor initialized\r\n");
}

// ================ Mode Number Calculation ================

uint8_t calculate_mode_number_from_frequency(float frequency) {
    if (frequency <= 0) return MIN_MODE_NUMBER;

    // String theory relation
    // f = (n/2L) * sqrt(T_design/老) => n = f * 2L / sqrt(T_design/老)
    float sqrt_ratio = sqrtf(DESIGN_TENSION * 1000.0f / CABLE_DENSITY);
    float mode_float = frequency * 2.0f * CABLE_LENGTH / sqrt_ratio;

    uint8_t mode_number = (uint8_t)roundf(mode_float);

    // Clamp to valid range
    if (mode_number < MIN_MODE_NUMBER) mode_number = MIN_MODE_NUMBER;
    if (mode_number > MAX_MODE_NUMBER) mode_number = MAX_MODE_NUMBER;

    return mode_number;
}

float calculate_theoretical_frequency(uint8_t mode_number, float tension_kN) {
    if (mode_number == 0 || tension_kN <= 0) return 0.0f;

    // f = (n/2L) * sqrt(T/老)
    float frequency = ((float)mode_number / (2.0f * CABLE_LENGTH)) *
                      sqrtf(tension_kN * 1000.0f / CABLE_DENSITY);

    return frequency;
}

// ================ Packet Loss Detection ================

bool detect_packet_loss(const uint8_t* mask, uint32_t length, float* loss_rate) {
    uint32_t lost_samples = 0;

    for (uint32_t i = 0; i < length; i++) {
        if (mask[i] == 0) {
            lost_samples++;
        }
    }

    *loss_rate = (float)lost_samples / length * 100.0f;
    return (lost_samples > 0);
}

void simulate_packet_loss(uint8_t* mask, uint32_t length, float loss_percentage) {
    // Initialize all samples as valid
    memset(mask, 1, length);

    if (loss_percentage <= 0) return;

    uint32_t samples_to_lose = (uint32_t)(length * loss_percentage / 100.0f);

    // Simulate random loss
    for (uint32_t i = 0; i < samples_to_lose; i++) {
        uint32_t pos = rng_get_random_num() % length;
        mask[pos] = 0;
    }
}

// ================ Compressive Sensing Reconstruction ================

int cs_reconstruction_dct(const float* measurements, const uint8_t* mask,
                          float* reconstructed, uint32_t length) {

    // Count available measurements
    uint32_t num_measurements = 0;
    for (uint32_t i = 0; i < length; i++) {
        if (mask[i] == 1) num_measurements++;
    }

    printf("CS reconstruction: %lu/%lu samples available\r\n",
           (unsigned long)num_measurements, (unsigned long)length);

    if (num_measurements < length / 8) {  // More permissive threshold
        printf("Warning: too few measurements, falling back to linear interpolation\r\n");
        return simple_interpolation_reconstruction(measurements, mask, reconstructed, length);
    }

    // Build measurement vector
    float *measurement_vector = (float *)mymalloc(SRAMEX, num_measurements * sizeof(float));
    if (!measurement_vector) {
        printf("CS reconstruction failed: memory allocation error\r\n");
        return -1;
    }

    uint32_t meas_idx = 0;
    for (uint32_t i = 0; i < length; i++) {
        if (mask[i] == 1) {
            measurement_vector[meas_idx++] = measurements[i];
        }
    }

    // Build sensing matrix (sampling matrix * DCT basis)
    float **phi_matrix = (float **)mymalloc(SRAMEX, num_measurements * sizeof(float *));
    if (!phi_matrix) {
        myfree(SRAMEX, measurement_vector);
        return -1;
    }

    for (uint32_t i = 0; i < num_measurements; i++) {
        phi_matrix[i] = (float *)mymalloc(SRAMEX, length * sizeof(float));
        if (!phi_matrix[i]) {
            // Cleanup
            for (uint32_t j = 0; j < i; j++) {
                myfree(SRAMEX, phi_matrix[j]);
            }
            myfree(SRAMEX, phi_matrix);
            myfree(SRAMEX, measurement_vector);
            return -1;
        }
    }

    // Fill sensing matrix: rows select observed time indices
    uint32_t row = 0;
    for (uint32_t i = 0; i < length; i++) {
        if (mask[i] == 1) {
            for (uint32_t j = 0; j < length; j++) {
                phi_matrix[row][j] = dct_basis_matrix[j][i];
            }
            row++;
        }
    }

    // Orthogonal Matching Pursuit (OMP)
    memset(cs_coefficients, 0, length * sizeof(float));

    float *residual = (float *)mymalloc(SRAMEX, num_measurements * sizeof(float));
    int *selected_atoms = (int *)mymalloc(SRAMEX, K_SPARSITY * sizeof(int));

    if (!residual || !selected_atoms) {
        if (residual) myfree(SRAMEX, residual);
        if (selected_atoms) myfree(SRAMEX, selected_atoms);
        for (uint32_t i = 0; i < num_measurements; i++) {
            myfree(SRAMEX, phi_matrix[i]);
        }
        myfree(SRAMEX, phi_matrix);
        myfree(SRAMEX, measurement_vector);
        return -1;
    }

    memcpy(residual, measurement_vector, num_measurements * sizeof(float));

    int max_iterations = (K_SPARSITY < (int)num_measurements) ? K_SPARSITY : (int)num_measurements;

    for (int iter = 0; iter < max_iterations; iter++) {
        float max_correlation = 0.0f;
        int best_atom = 0;

        // Select atom with largest correlation
        for (uint32_t atom = 0; atom < length; atom++) {
            float correlation = 0.0f;
            for (uint32_t i = 0; i < num_measurements; i++) {
                correlation += residual[i] * phi_matrix[i][atom];
            }
            correlation = fabsf(correlation);

            if (correlation > max_correlation) {
                max_correlation = correlation;
                best_atom = (int)atom;
            }
        }

        if (max_correlation < 1e-6f) break; // No significant correlation

        selected_atoms[iter] = best_atom;

        // Least-squares coefficient update for the selected atom
        float norm_sq = 0.0f;
        for (uint32_t i = 0; i < num_measurements; i++) {
            norm_sq += phi_matrix[i][best_atom] * phi_matrix[i][best_atom];
        }

        if (norm_sq > 1e-6f) {
            float projection = 0.0f;
            for (uint32_t i = 0; i < num_measurements; i++) {
                projection += residual[i] * phi_matrix[i][best_atom];
            }
            cs_coefficients[best_atom] = projection / norm_sq;

            // Residual update
            for (uint32_t i = 0; i < num_measurements; i++) {
                residual[i] -= cs_coefficients[best_atom] * phi_matrix[i][best_atom];
            }
        }
    }

    // Reconstruct signal using DCT basis
    memset(reconstructed, 0, length * sizeof(float));
    for (uint32_t i = 0; i < length; i++) {
        for (uint32_t j = 0; j < length; j++) {
            reconstructed[i] += cs_coefficients[j] * dct_basis_matrix[j][i];
        }
    }

    // Overwrite with measured samples to respect known data
    for (uint32_t i = 0; i < length; i++) {
        if (mask[i] == 1) {
            reconstructed[i] = measurements[i];
        }
    }

    // Cleanup
    myfree(SRAMEX, residual);
    myfree(SRAMEX, selected_atoms);
    for (uint32_t i = 0; i < num_measurements; i++) {
        myfree(SRAMEX, phi_matrix[i]);
    }
    myfree(SRAMEX, phi_matrix);
    myfree(SRAMEX, measurement_vector);

    printf("CS reconstruction completed\r\n");
    return 0;
}

// ================ Simple Linear Interpolation (Fallback) ================

int simple_interpolation_reconstruction(const float* measurements, const uint8_t* mask,
                                        float* reconstructed, uint32_t length) {
    // Copy/interpolate data
    for (uint32_t i = 0; i < length; i++) {
        if (mask[i] == 1) {
            reconstructed[i] = measurements[i];
        } else {
            // Linear interpolation for missing sample
            float prev_val = 0.0f, next_val = 0.0f;
            int prev_idx = -1, next_idx = -1;

            // Find previous valid
            for (int j = (int)i - 1; j >= 0; j--) {
                if (mask[j] == 1) {
                    prev_val = measurements[j];
                    prev_idx = j;
                    break;
                }
            }

            // Find next valid
            for (uint32_t j = i + 1; j < length; j++) {
                if (mask[j] == 1) {
                    next_val = measurements[j];
                    next_idx = j;
                    break;
                }
            }

            // Interpolate
            if (prev_idx >= 0 && next_idx >= 0) {
                float weight = (float)(i - prev_idx) / (float)(next_idx - prev_idx);
                reconstructed[i] = prev_val + weight * (next_val - prev_val);
            } else if (prev_idx >= 0) {
                reconstructed[i] = prev_val;
            } else if (next_idx >= 0) {
                reconstructed[i] = next_val;
            } else {
                reconstructed[i] = 0.0f;
            }
        }
    }
    return 0;
}

// ================ Single Sensor Frequency Analysis ================

float estimate_dominant_frequency_single(const float* signal, uint32_t length,
                                         dominant_mode_t* result) {
    // Copy to FFT buffer
    memcpy(fft_buffer, signal, length * sizeof(float));

    // Execute FFT
    arm_rfft_fast_f32(&rfft_instance, fft_buffer, fft_output, 0);

    // Power spectrum peak search
    float max_magnitude = 0.0f;
    uint32_t peak_bin = 0;

    for (uint32_t i = 1; i < length / 2; i++) {
        float real = fft_output[2 * i];
        float imag = fft_output[2 * i + 1];
        float magnitude = sqrtf(real * real + imag * imag);

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            peak_bin = i;
        }
    }

    float dominant_freq = (float)peak_bin * FS / length;

    // Fill result struct
    if (result) {
        result->frequency   = dominant_freq;
        result->amplitude   = max_magnitude;
        result->mode_number = calculate_mode_number_from_frequency(dominant_freq);
        result->confidence  = max_magnitude / (max_magnitude + 1e-6f);
    }

    return dominant_freq;
}

// ================ Tension Calculation ================

float calculate_tension_from_frequency(float frequency, uint8_t mode_number) {
    if (frequency <= 0.0f || mode_number == 0) {
        return 0.0f;
    }

    // String model: T = 4 * 老 * L^2 * (f/n)^2
    float ratio = frequency / (float)mode_number;
    float tension = 4.0f * CABLE_DENSITY * CABLE_LENGTH * CABLE_LENGTH * ratio * ratio;

    return tension / 1000.0f; // N -> kN
}

// ================ Real-time Statistics ================

void update_realtime_statistics(realtime_stats_t* stats, float tension, float frequency) {
    if (!stats) return;

    uint32_t idx = stats->history_index % TENSION_BUFFER_SIZE;
    stats->tension_history[idx] = tension;
    stats->freq_history[idx]    = frequency;

    stats->history_index++;
    if (stats->valid_samples < TENSION_BUFFER_SIZE) {
        stats->valid_samples++;
    }

    // Mean
    float sum = 0.0f;
    for (uint32_t i = 0; i < stats->valid_samples; i++) {
        sum += stats->tension_history[i];
    }
    stats->mean_tension = sum / (float)stats->valid_samples;

    // Standard deviation
    if (stats->valid_samples > 1) {
        float sum_sq_diff = 0.0f;
        for (uint32_t i = 0; i < stats->valid_samples; i++) {
            float diff = stats->tension_history[i] - stats->mean_tension;
            sum_sq_diff += diff * diff;
        }
        stats->std_tension = sqrtf(sum_sq_diff / (float)(stats->valid_samples - 1));

        // Numerical safety
        if (!isfinite(stats->std_tension) || stats->std_tension < 0.0f) {
            stats->std_tension = 0.0f;
        }
    } else {
        stats->std_tension = 0.0f;
    }
}

// ================ Single Sensor Processing Function ================

int process_single_sensor_data(const float* sensor_data, const uint8_t* mask,
                               tension_result_t* result, realtime_stats_t* stats) {
    if (!sensor_data || !result) {
        printf("Invalid input parameters\r\n");
        return -1;
    }

    // Save original signal
    memcpy(temp_signal_buffer, sensor_data, SIGNAL_LEN * sizeof(float));

    // Packet loss detection
    float loss_rate = 0.0f;
    bool has_loss = detect_packet_loss(mask, SIGNAL_LEN, &loss_rate);

    if (has_loss) {
        printf("Detected %.1f%% packet loss, running reconstruction...\r\n", loss_rate);
        // CS reconstruction
        if (cs_reconstruction_dct(sensor_data, mask,
                                  reconstructed_buffer, SIGNAL_LEN) != 0) {
            printf("Signal reconstruction failed\r\n");
            return -1;
        }
        printf("Signal reconstruction done\r\n");
    } else {
        // Direct copy
        memcpy(reconstructed_buffer, sensor_data, SIGNAL_LEN * sizeof(float));
    }

    // Frequency analysis
    dominant_mode_t freq_result;
    float dominant_freq = estimate_dominant_frequency_single(reconstructed_buffer,
                                                             SIGNAL_LEN, &freq_result);

    if (dominant_freq <= 0.0f || dominant_freq > FS / 2.0f) {
        printf("Invalid dominant frequency: %.2f Hz\r\n", dominant_freq);
        return -1;
    }

    // Tension calculation
    float tension = calculate_tension_from_frequency(dominant_freq, freq_result.mode_number);

    if (tension <= 0.0f) {
        printf("Tension calculation failed: %.2f kN\r\n", tension);
        return -1;
    }

    // Populate result
    result->tension       = tension;
    result->frequency     = dominant_freq;
    result->dominant_mode = freq_result.mode_number;
    result->confidence    = freq_result.confidence;
    result->timestamp     = processing_counter++;

    // Update stats
    if (stats) {
        update_realtime_statistics(stats, tension, dominant_freq);
    }

    printf("Analysis: T=%.2f kN, f=%.2f Hz, mode=%d, confidence=%.3f\r\n",
           tension, dominant_freq, freq_result.mode_number, freq_result.confidence);

    return 0;
}

// ================ Compatible Multi-Sensor Interface ================

int run_realtime_tension_estimation(const float multi_sensor_input[NUM_SENSORS][SIGNAL_LEN],
                                    const uint8_t multi_sensor_masks[NUM_SENSORS][SIGNAL_LEN],
                                    tension_result_t* results,
                                    uint32_t* num_results,
                                    realtime_stats_t* stats) {

    if (!multi_sensor_input || !results || !num_results) {
        return -1;
    }

    *num_results = 0;

    // Use sensor #0 for now
    int rc = process_single_sensor_data(multi_sensor_input[0],
                                        multi_sensor_masks[0],
                                        &results[0], stats);

    if (rc == 0) {
        *num_results = 1;
    }

    return rc;
}

// ================ Utility Functions ================

void print_tension_results(const tension_result_t* results, uint32_t count) {
    if (count == 0) {
        printf("No valid analysis results\r\n");
        return;
    }

    printf("\n=== Tension Estimation Results ===\r\n");
    for (uint32_t i = 0; i < count; i++) {
        printf("Window %lu: Tension=%.2f kN, Frequency=%.2f Hz, Mode=%d, Confidence=%.3f\r\n",
               (unsigned long)i, results[i].tension, results[i].frequency,
               results[i].dominant_mode, results[i].confidence);
    }
}

float calculate_reconstruction_error(const float* original, const float* reconstructed, uint32_t length) {
    float error_sum = 0.0f, original_sum = 0.0f;

    for (uint32_t i = 0; i < length; i++) {
        float error = original[i] - reconstructed[i];
        error_sum += error * error;
        original_sum += original[i] * original[i];
    }

    if (original_sum > 1e-10f) {
        return sqrtf(error_sum / original_sum) * 100.0f;
    }
    return 0.0f;
}

// ================ Validation Functions ================

void validate_single_sensor_analysis(void) {
    printf("\n=== Single-Sensor Analysis Validation ===\r\n");

    // Create test signal
    float test_signal[SIGNAL_LEN];
    uint8_t test_mask[SIGNAL_LEN];

    // Generate a signal around the theoretical frequency for 3150 kN (mode 14)
    float theoretical_freq = calculate_theoretical_frequency(14, DESIGN_TENSION);
    printf("Theoretical frequency (mode 14, T=%.0f kN): %.2f Hz\r\n", DESIGN_TENSION, theoretical_freq);

    for (int i = 0; i < (int)SIGNAL_LEN; i++) {
        test_signal[i] = 0.5f * sinf(2 * PI * theoretical_freq * i / FS) +
                         0.3f * sinf(2 * PI * theoretical_freq * 2 * i / FS);
        test_mask[i] = 1;
    }

    // Analyze test signal
    tension_result_t test_result;
    realtime_stats_t test_stats = {0};

    int rc = process_single_sensor_data(test_signal, test_mask, &test_result, &test_stats);

    if (rc == 0) {
        float error = fabsf(test_result.tension - DESIGN_TENSION) / DESIGN_TENSION * 100.0f;
        printf("Validation: Tension=%.2f kN, Error=%.2f%%\r\n", test_result.tension, error);

        if (error < 5.0f) {
            printf("Validation PASSED\r\n");
        } else {
            printf("Validation FAILED: excessive error\r\n");
        }
    } else {
        printf("Validation FAILED: processing error\r\n");
    }
}

void validate_cs_reconstruction(void) {
    printf("\n=== CS Reconstruction Validation ===\r\n");

    // Create test signal
    float test_signal[SIGNAL_LEN];
    for (int i = 0; i < (int)SIGNAL_LEN; i++) {
        test_signal[i] = sinf(2 * PI * 5.0f * i / FS) + 0.5f * sinf(2 * PI * 12.0f * i / FS);
    }

    // Test multiple loss rates
    float loss_rates[] = {5.0f, 10.0f, 15.0f, 20.0f};
    int num_tests = (int)(sizeof(loss_rates) / sizeof(loss_rates[0]));

    for (int t = 0; t < num_tests; t++) {
        uint8_t mask[SIGNAL_LEN];
        simulate_packet_loss(mask, SIGNAL_LEN, loss_rates[t]);

        float reconstructed[SIGNAL_LEN];
        int rc = cs_reconstruction_dct(test_signal, mask, reconstructed, SIGNAL_LEN);

        if (rc == 0) {
            float error = calculate_reconstruction_error(test_signal, reconstructed, SIGNAL_LEN);
            printf("Loss: %5.1f%% | Reconstruction error: %6.3f%%\r\n",
                   loss_rates[t], error);
        } else {
            printf("Loss: %5.1f%% | Reconstruction FAILED\r\n", loss_rates[t]);
        }
    }
}

void benchmark_single_sensor_processing(void) {
    printf("\n=== Single-Sensor Processing Benchmark ===\r\n");

    float test_signal[SIGNAL_LEN];
    uint8_t test_mask[SIGNAL_LEN];
    tension_result_t test_result;

    // Initialize test data
    for (int i = 0; i < (int)SIGNAL_LEN; i++) {
        test_signal[i] = sinf(2 * PI * 10.5f * i / FS);
        test_mask[i] = 1;
    }

    uint32_t start_time = HAL_GetTick();

    // Process 10 times
    int success_count = 0;
    for (int i = 0; i < 10; i++) {
        if (process_single_sensor_data(test_signal, test_mask, &test_result, NULL) == 0) {
            success_count++;
        }
    }

    uint32_t end_time = HAL_GetTick();

    printf("Elapsed time: %lu ms (10 runs)\r\n", (unsigned long)(end_time - start_time));
    printf("Success: %d/10\r\n", success_count);
    printf("Throughput: %.1f windows/s\r\n",
           success_count * 1000.0f / (float)(end_time - start_time));
}

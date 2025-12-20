/* =============================================================================
 *  File:       adc_driver.h
 *  
 *  Description:
 *      Driver for the Sensor Controller Interface (SCIF) ADC Task.
 *      Manages the connection between the Cortex-M4F Application and the
 *      Sensor Controller Engine (SCE) execution logic.
 *
 *  Project:    Multi-Sensor Data Acquisition Platform
 * =============================================================================
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

// -----------------------------------------------------------------------------
// Structures
// -----------------------------------------------------------------------------

/**
 * @brief   ADC Driver Context.
 *          Encapsulates buffer storage and synchronization state.
 */
typedef struct {
    // Buffering
    uint8_t          storage[2048];             
    ring_buffer_t    buffer;
    stream_metrics_t stats;

    // State & Config
    bool             initialized;
    bool             powered;
    uint32_t         frequency_hz;
    uint16_t         matrix_size;
    
    // Timing Logic (for implicit timestamping)
    double           oldest_sample_timestamp; 
    double           us_per_sample;    
} adc_driver_t;

// Global Singleton
extern adc_driver_t g_adc;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool ADC_init(void);
bool ADC_startStreaming(void);
bool ADC_stopStreaming(void);
bool ADC_isStreaming(void);
void ADC_setFrequency(uint32_t new_rate);
void ADC_setMatrixSize(uint16_t size);

// Data Path
void ADC_writeBuffer(uint8_t* data, uint16_t len); // Call from SCIF Alert
uint16_t ADC_getAvailableBytes(void);
void ADC_getStats(stream_metrics_t* stats_out);
void ADC_resetStats(void);

#ifdef __cplusplus
}
#endif

#endif // ADC_DRIVER_H
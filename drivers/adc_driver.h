#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h" // Unified Buffer

// Global instance
extern struct adc_driver_s g_adc;

typedef struct adc_driver_s {
    // 1. Unified Buffer
    uint8_t          storage[2048];             
    ring_buffer_t    buffer;
    stream_metrics_t stats;

    // 2. Hardware/State
    bool             initialized;
    bool             powered;
    uint32_t         frequency_hz;
    uint16_t         matrix_size;
    double           oldest_sample_timestamp; 
    double           us_per_sample;    
} adc_driver_t;

// API
bool ADC_init(void);
bool ADC_startStreaming(void);
bool ADC_stopStreaming(void);
bool ADC_isRunning(void);
void ADC_setFrequency(uint32_t new_rate);
void ADC_setMatrixSize(uint16_t size);

// Data Path
void ADC_writeBuffer(uint8_t* data, uint16_t len); // Call from SCIF Alert
uint16_t ADC_getAvailableBytes(void);
void ADC_getStats(stream_metrics_t* stats_out);
void ADC_resetStats(void);

#endif
#ifndef ISM330_DRIVER_H
#define ISM330_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <ti/sysbios/knl/Semaphore.h>
#include "sensor_platform.h"
#include "ism330dhcx_reg.h"
#include "ring_buffer.h" // The new unified file

// --- FIFO CONFIGURATION ---
#define ISM_BYTES_PER_SAMPLE    7   

// TRIGGER LEVEL: 6 samples * 3.5 words/sample = 21 words

// 120 words = 240 bytes = 34.2 samples
// 140 words = 280 bytes = 40 samples 
// 175 words = 350 bytes = 50 samples
// 350 words = 700 bytes = 100 samples 
// CONSTANTS FOR WATERMARK

#define ISM_FIFO_WTM_WORDS      120

// Global instance
extern struct ism330_driver_s g_ism330;

// ODR mapping (exposed for system monitor)
extern const uint16_t gyr_odr_map[];
extern const float gyr_odr_freq[];

typedef struct ism330_driver_s {
    // 1. Unified Ring Buffer Components
    uint8_t             storage[1024];               // Memory (Power of 2, 1024 is safer for 833Hz+)
    ring_buffer_t       buffer;                     // Interface
    stream_metrics_t    stats;                      // Metrics (Overflows, Bytes Sent, etc)

    // 2. Hardware/RTOS Handles
    bool                initialized;
    bool                powered;
    uint8_t             odr_index;

    // TIMESTAMP LOGIC (Revised to double)
    double              oldest_sample_timestamp; 
    double              us_per_sample;
    
    stmdev_ctx_t        dev_ctx;
    platform_i2c_bus_t* i2c_bus;
    Semaphore_Handle    drdy_sem;
} ism330_driver_t;

// ============================================================================
// PUBLIC API
// ============================================================================
bool ISM330_init(platform_i2c_bus_t* i2c_bus);
bool ISM330_startStreaming(void);
bool ISM330_stopStreaming(void);
bool ISM330_isStreaming(void);
bool ISM330_reconfigureOdr(uint32_t odr_index);

// Data Path
void ISM330_writeBuffer(uint8_t* data, uint16_t len);
int32_t ISM330_readFifoBatch_Block(uint8_t* buffer, uint16_t total_bytes);
uint16_t ISM330_getAvailableBytes(void);
void ISM330_getStats(stream_metrics_t* stats_out);
void ISM330_resetStats(void);

#endif // ISM330_DRIVER_H
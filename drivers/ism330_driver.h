/*
 * =============================================================================
 *  File:       ism330_driver.h
 *  
 *  Description:
 *      Driver for the ISM330DHCX 6-DoF IMU.
 *      Manages FIFO batching and burst-read operations.
 * =============================================================================
 */

#ifndef ISM330_DRIVER_H
#define ISM330_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <ti/sysbios/knl/Semaphore.h>

#include "sensor_platform.h"
#include "ring_buffer.h"
#include "/vendor/ism330dhcx_reg.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

#define ISM_BYTES_PER_SAMPLE    7   

// TRIGGER LEVEL: 6 samples * 3.5 words/sample = 21 words
// 120 words = 240 bytes = 34.2 samples
// 140 words = 280 bytes = 40 samples 
// 175 words = 350 bytes = 50 samples
// 350 words = 700 bytes = 100 samples 
// CONSTANTS FOR WATERMARK
#define ISM_FIFO_WTM_WORDS      120

// -----------------------------------------------------------------------------
// Structures
// -----------------------------------------------------------------------------

typedef struct {
    // Buffering
    uint8_t             storage[1024];               
    ring_buffer_t       buffer;                     
    stream_metrics_t    stats;                      

    // State
    bool                initialized;
    bool                powered;
    uint8_t             odr_index;

    // Timing
    double              oldest_sample_timestamp; 
    double              us_per_sample;

    // Hardware Handles
    stmdev_ctx_t        dev_ctx;
    platform_i2c_bus_t* i2c_bus;
    Semaphore_Handle    drdy_sem;
} ism330_driver_t;

// Global Singleton
extern ism330_driver_t g_ism330;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

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

#ifdef __cplusplus
}
#endif

#endif // ISM330_DRIVER_H
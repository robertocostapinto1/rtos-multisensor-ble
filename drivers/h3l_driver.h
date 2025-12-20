/* =============================================================================
 *  File:       h3l_driver.h
 *  
 *  Description:
 *      Driver for the H3LIS331DL High-G Accelerometer.
 *      Implements a Hybrid Interrupt/Polling architecture for legacy sensors.
 * =============================================================================
 */

#ifndef H3L_DRIVER_H
#define H3L_DRIVER_H

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
#include "/vendor/h3lis331dl_reg.h"

// -----------------------------------------------------------------------------
// Structures
// -----------------------------------------------------------------------------
/**
 * @brief   H3L Driver Context.
 */
typedef struct {
    // Buffering
    uint8_t             storage[1024];               
    ring_buffer_t       buffer;                     
    stream_metrics_t    stats;                      

    // State
    bool                initialized;
    bool                powered;
    uint8_t             odr_index;
    
    // Timing & Watchdog
    double              oldest_sample_timestamp; 
    double              us_per_sample;
    uint32_t            timeout_ticks;   

    // Hardware Handles
    stmdev_ctx_t        dev_ctx;
    platform_i2c_bus_t* i2c_bus;
    Semaphore_Handle    drdy_sem;           
} h3l_driver_t;

// Global Singleton
extern h3l_driver_t g_h3l;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
bool H3L_init(platform_i2c_bus_t* i2c_bus);
bool H3L_startStreaming(void);
bool H3L_stopStreaming(void);
bool H3L_isStreaming(void);
bool H3L_reconfigureOdr(uint32_t odr_index);

// Data Path
void H3L_writeBuffer(uint8_t* data, uint16_t len);
uint16_t H3L_getAvailableBytes(void);
void H3L_getStats(stream_metrics_t* stats_out);
void H3L_resetStats(void);


#ifdef __cplusplus
}
#endif

#endif // H3L_DRIVER_H
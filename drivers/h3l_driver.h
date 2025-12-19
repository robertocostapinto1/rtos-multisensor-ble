#ifndef H3L_DRIVER_H
#define H3L_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <ti/sysbios/knl/Semaphore.h>
#include "sensor_platform.h"
#include "h3lis331dl_reg.h"
#include "ring_buffer.h" // Unified Buffer

// Global instance
extern struct h3l_driver_s g_h3l;

extern const uint16_t acc_odr_map[];
extern const float acc_odr_freq[];

typedef struct h3l_driver_s {
    // 1. Unified Buffer
    uint8_t             storage[1024];               // Memory (Power of 2, 1024 is safer for 833Hz+)
    ring_buffer_t       buffer;                     // Interface
    stream_metrics_t    stats;                      // Metrics (Overflows, Bytes Sent, etc)

    // 2. Hardware/State
    bool                initialized;
    bool                powered;
    uint8_t             odr_index;
    double              oldest_sample_timestamp; 
    double              us_per_sample;
    uint32_t            timeout_ticks;              // 

    stmdev_ctx_t        dev_ctx;
    platform_i2c_bus_t* i2c_bus;
    Semaphore_Handle    drdy_sem;
} h3l_driver_t;

// API
bool H3L_init(platform_i2c_bus_t* i2c_bus);
bool H3L_startStreaming(void);
bool H3L_stopStreaming(void);
bool H3L_isStreaming(void);
bool H3L_reconfigureOdr(uint32_t odr_index);

// Data Path
void H3L_writeBuffer(uint8_t* data, uint16_t len); // Call from H3L Task
uint16_t H3L_getAvailableBytes(void);
void H3L_getStats(stream_metrics_t* stats_out);
void H3L_resetStats(void);

#endif
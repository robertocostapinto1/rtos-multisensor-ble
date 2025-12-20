/*
 * =============================================================================
 *  File:       data_stream_engine.c
 *  
 *  Description:
 *      Implementation of the Data Stream Engine. Contains the logic for
 *      packet batching, timestamp synchronization, and zero-copy transmission.
 * =============================================================================
 */
// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <string.h>

/* BLE Stack Headers */
#include "bcomdef.h"
#include "gatt.h"
#include "icall_ble_api.h" 
#include "util.h"

/* Project Headers */
#include "data_stream_engine.h"
#include <simple_gatt_profile.h>

/* Drivers */
#include "/drivers/ism330_driver.h"
#include "/drivers/h3l_driver.h"
#include "/drivers/adc_driver.h"
#include "/drivers/ring_buffer.h"

// -----------------------------------------------------------------------------
// Constants & Macros
// -----------------------------------------------------------------------------
#define SP_THROUGHPUT_EVT       Event_Id_00

// Characteristic handles
#define ADC_NOTIFY_IDX 11
#define GYR_NOTIFY_IDX 21
#define ACC_NOTIFY_IDX 31

// Batch size target (240 bytes payload + 4 bytes header = 244 bytes)
#define BATCH_SIZE_TARGET       240

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
extern ICall_SyncHandle syncEvent;

// Telemetry Metrics
static uint32_t ble_buffer_failures = 0;
static uint32_t ble_packets_sent = 0;

// -----------------------------------------------------------------------------
// Private Helper Functions
// -----------------------------------------------------------------------------

/**
 * @brief   Core Zero-Copy Transmission Logic.
 *          Peeks at the ring buffer, formats the packet, and commits only on success.
 * 
 * @param   connHandle      BLE Connection Handle
 * @param   charHandle      GATT Characteristic Attribute Handle
 * @param   rb              Pointer to Sensor Ring Buffer
 * @param   p_driver_ts     Pointer to Driver's persistent timestamp
 * @param   us_per_sample   Microseconds per sample (for linear projection)
 * @param   sample_size     Size of one sample in bytes
 * @param   min_batch_size  Minimum bytes required to trigger a send
 * 
 * @return  bool            True if more data remains in buffer
 */
static bool process_sensor_stream(uint16_t connHandle, 
                                  uint16_t charHandle, 
                                  ring_buffer_t* rb, 
                                  double* p_driver_ts, 
                                  double us_per_sample, 
                                  uint16_t sample_size,
                                  uint16_t min_batch_size) 
{
    // 1. Check Availability
    uint16_t available = rb_available(rb);
    
    // Drop-Oldest Logic: If buffer is critical (>80%), force send/drop to catch up
    bool buffer_critical = (available > (rb->size * 4 / 5));

    if (available < min_batch_size && !buffer_critical) 
    {
        return false; // Wait for more data to accumulate
    }

    // 2. Calculate Payload Size
    uint16_t to_send = (available > BATCH_SIZE_TARGET) ? BATCH_SIZE_TARGET : available;
    to_send -= (to_send % sample_size); 
    
    if (to_send == 0) return false;

    // 3. Allocate BLE Packet (Stack Heap)
    uint16_t allocLen = to_send + 4; // Data + 4B Header
    attHandleValueNoti_t noti;
    noti.pValue = (uint8_t *)GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, allocLen, &allocLen);
    
    if (noti.pValue == NULL) {
        ble_buffer_failures++;

        if (buffer_critical) {
            rb_commit(rb, to_send); // DROP data if critical to maintain real-time
        }
        return true;  // Return true to retry later
    }

    // 4. Construct Header (Hardware Timestamp)
    uint32_t current_ts = *p_driver_ts;
    memcpy(noti.pValue, &current_ts, 4);

    // 5. Zero-Copy Data Transfer (Critical Section)
    uintptr_t key = HwiP_disable();
    // Safety re-check
    if (rb_available(rb) < to_send)
    {
        to_send = rb_available(rb) - (rb_available(rb) % sample_size);
    }

    if (to_send > 0) {
        rb_peek(rb, noti.pValue + 4, to_send);
    }
    HwiP_restore(key);

    if (to_send == 0) {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        return false;
    }

    // 6. Transmit to Link Layer
    noti.len = to_send + 4;
    noti.handle = charHandle;

    if (GATT_Notification(connHandle, &noti, FALSE) == SUCCESS) 
    {
        // Success: Advance Ring Buffer Tail
        rb_commit(rb, to_send);
        ble_packets_sent++;
        
        // Update Timestamp Projection (Closed-Loop Correction)
        if (rb_available(rb) > 0) 
        {
             uint32_t samples_sent = 0;
             if (sample_size == 3) {
                 samples_sent = (to_send / 3) * 2; // Packed ADC
             } else {
                 samples_sent = to_send / sample_size;
             }
             
             *p_driver_ts += (samples_sent * us_per_sample);
        }
        
        // Return true if we can burst more immediately
        return (rb_available(rb) >= min_batch_size); 
    } 
    else 
    {
        // Failure: Free memory, do not advance tail
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        ble_buffer_failures++;
        return true; // Retry requested
    }
}

// Renamed from BLE_sendData
bool DataStream_processEvents(uint16_t connHandle) 
{
    bool more_data = false;
    
    // 1. Accelerometer (Priority High - No FIFO)
    if (g_h3l.powered) {
        more_data |= process_sensor_stream(
            connHandle, 
            simpleProfileAttrTbl[ACC_NOTIFY_IDX].handle, 
            &g_h3l.buffer, 
            &g_h3l.oldest_sample_timestamp, 
            g_h3l.us_per_sample, 6, BATCH_SIZE_TARGET
        );
    }

    // 2. Gyroscope (Priority Medium - Hardware FIFO)
    if (g_ism330.powered) {
        more_data |= process_sensor_stream(
            connHandle, 
            simpleProfileAttrTbl[GYR_NOTIFY_IDX].handle, 
            &g_ism330.buffer, 
            &g_ism330.oldest_sample_timestamp, 
            g_ism330.us_per_sample, 6, BATCH_SIZE_TARGET
        );
    }

    // 3. ADC (Priority Low - High Bandwidth)
    if (g_adc.powered) {
        more_data |= process_sensor_stream(
            connHandle, 
            simpleProfileAttrTbl[ADC_NOTIFY_IDX].handle, 
            &g_adc.buffer, 
            &g_adc.oldest_sample_timestamp, 
            g_adc.us_per_sample, 3, BATCH_SIZE_TARGET
        );
    }

    // Scheduler Logic: Yield and Retry if data remains
    if (more_data) Event_post(syncEvent, SP_THROUGHPUT_EVT);
    return more_data;
}


void DataStream_updateTelemetry(uint16_t connHandle) 
{
    // Telemetry Structure (Packed for OTA efficiency)
    typedef struct __attribute__((packed)) {
        // Timestamp (4 bytes)
        uint32_t timestamp_ms;
        // ADC Stats (18 bytes)
        uint32_t adc_samples; uint32_t adc_drops; uint32_t adc_bytes; 
        uint32_t adc_reconf;  uint16_t adc_util;
        // Gyro Stats (18 bytes)
        uint32_t gyr_samples; uint32_t gyr_drops; uint32_t gyr_bytes;
        uint32_t gyr_reconf;  uint16_t gyr_util;
        // Accel Stats(18 bytes)
        uint32_t acc_samples; uint32_t acc_drops; uint32_t acc_bytes;
        uint32_t acc_reconf;  uint16_t acc_util;
        // System Flags (4 bytes)
        uint8_t flags[4];
    } telemetry_data_t;
    
    telemetry_data_t t = {0};
    
    t.timestamp_ms = Clock_getTicks(); // 10us ticks -> converted client side

    // Snapshot ADC
    ADC_getStats(&g_adc.stats);
    t.adc_samples = g_adc.stats.samples_processed;
    t.adc_drops   = g_adc.stats.overflow_events;
    t.adc_bytes   = g_adc.stats.bytes_sent;
    t.adc_reconf  = g_adc.stats.reconfigurations;
    t.adc_util    = g_adc.stats.peak_utilization_percent;

    // Snapshot Gyro
    ISM330_getStats(&g_ism330.stats);
    t.gyr_samples = g_ism330.stats.samples_processed;
    t.gyr_drops   = g_ism330.stats.overflow_events;
    t.gyr_bytes   = g_ism330.stats.bytes_sent;
    t.gyr_reconf  = g_ism330.stats.reconfigurations;
    t.gyr_util    = g_ism330.stats.peak_utilization_percent;

    // Snapshot Accel
    H3L_getStats(&g_h3l.stats);
    t.acc_samples = g_h3l.stats.samples_processed;
    t.acc_drops   = g_h3l.stats.overflow_events;
    t.acc_bytes   = g_h3l.stats.bytes_sent;
    t.acc_reconf  = g_h3l.stats.reconfigurations;
    t.acc_util    = g_h3l.stats.peak_utilization_percent;

    // Status Flags
    t.flags[0] = ADC_isStreaming();
    t.flags[1] = ISM330_isStreaming();
    t.flags[2] = H3L_isStreaming();
    t.flags[3] = (connHandle != LINKDB_CONNHANDLE_INVALID);

    // Update Characteristic
    SimpleProfile_SetParameter(TELEMETRY_CHAR, sizeof(telemetry_data_t), &t);
}

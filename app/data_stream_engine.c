#include <stdint.h>
#include "bcomdef.h"
#include "gatt.h"
#include <simple_gatt_profile.h>
#include "data_stream_engine.h"
#include "/drivers/ism330_driver.h"
#include "/drivers/h3l_driver.h"
#include "/drivers/adc_driver.h"
#include <icall_ble_api.h> // For GATT_bm_alloc

// External handle to the main task event (so we can repost the event)
extern ICall_SyncHandle syncEvent;
#define SP_THROUGHPUT_EVT Event_Id_00 // Must match simple_peripheral

#define ADC_NOTIFY_IDX 11
#define GYR_NOTIFY_IDX 21
#define ACC_NOTIFY_IDX 31

// Track BLE buffer failures
static uint32_t ble_buffer_failures = 0;
static uint32_t ble_packets_sent = 0;

/**
 * @brief Processes a sensor stream: Checks buffer, Adds Timestamp, Sends.
 */
static bool process_sensor_stream(uint16_t connHandle, 
                                  uint16_t charHandle, 
                                  ring_buffer_t* rb, 
                                  double* p_driver_ts, 
                                  double us_per_sample, 
                                  uint16_t sample_size,
                                  uint16_t min_batch_size) 
{
    // 1. Check availability
    uint16_t available = rb_available(rb);
    
    // We send: [4 Bytes TS] + [Data]
    // Max Payload for 251 MTU is ~244. So Max Data = 240.
    const uint16_t MAX_DATA_CAP = 240; 
    
    // Wait for batch, unless buffer is critical (>80%)
    bool buffer_critical = (available > (rb->size * 4 / 5));
    if (available < min_batch_size && !buffer_critical) return false;

    // Calc size
    uint16_t to_send = (available > MAX_DATA_CAP) ? MAX_DATA_CAP : available;
    to_send -= (to_send % sample_size); // Align
    
    if (to_send == 0) return false;

    // 2. Alloc
    uint16_t allocLen = to_send + 4; // Data + Header
    attHandleValueNoti_t noti;
    noti.pValue = (uint8_t *)GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, allocLen, &allocLen);
    
    if (noti.pValue == NULL) {
        ble_buffer_failures++;
        if (buffer_critical) {
            rb_commit(rb, to_send); // DROP data to prevent crash
        }
        return true; 
    }

    // 3. Write Header (Timestamp)
    uint32_t current_ts = *p_driver_ts;
    memcpy(noti.pValue, &current_ts, 4);

    // 4. Write Data
    uintptr_t key = HwiP_disable();
    // Safety re-check
    if (rb_available(rb) < to_send) to_send = rb_available(rb) - (rb_available(rb) % sample_size);
    
    if (to_send > 0) {
        rb_peek(rb, noti.pValue + 4, to_send);
    }
    HwiP_restore(key);

    if (to_send == 0) {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        return false;
    }

    // 5. Send
    noti.len = to_send + 4;
    noti.handle = charHandle;

    if (GATT_Notification(connHandle, &noti, FALSE) == SUCCESS) 
    {
        rb_commit(rb, to_send);
        ble_packets_sent++;
        
        // 6. Advance Timestamp Logic
        if (rb_available(rb) > 0) 
        {
             uint32_t samples_sent = 0;
             if (sample_size == 3) samples_sent = (to_send / 3) * 2; // ADC packed
             else samples_sent = to_send / sample_size;
             
             *p_driver_ts += (samples_sent * us_per_sample);
        }
        
        // Return true if we can burst more
        return (rb_available(rb) >= min_batch_size); 
    } 
    else 
    {
        GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        ble_buffer_failures++;
        return true; 
    }
}

// Renamed from BLE_sendData
bool DataStream_processEvents(uint16_t connHandle) 
{
 bool more = false;
    // Batch ~200 bytes for high efficiency
    const uint16_t BATCH = 240;
    const uint16_t min_BATCH = 120; 
    
    // 1. Accelerometer (Priority High - No FIFO)
    if (g_h3l.powered) {
        more |= process_sensor_stream(
            connHandle, 
            simpleProfileAttrTbl[ACC_NOTIFY_IDX].handle, 
            &g_h3l.buffer, 
            &g_h3l.oldest_sample_timestamp, 
            g_h3l.us_per_sample, 6, BATCH
        );
    }

    // 2. Gyroscope (Priority Medium - Hardware FIFO)
    if (g_ism330.powered) {
        more |= process_sensor_stream(
            connHandle, 
            simpleProfileAttrTbl[GYR_NOTIFY_IDX].handle, 
            &g_ism330.buffer, 
            &g_ism330.oldest_sample_timestamp, 
            g_ism330.us_per_sample, 6, BATCH
        );
    }

    // 3. ADC (Priority Low - High Bandwidth)
    if (g_adc.powered) {
        more |= process_sensor_stream(
            connHandle, 
            simpleProfileAttrTbl[ADC_NOTIFY_IDX].handle, 
            &g_adc.buffer, 
            &g_adc.oldest_sample_timestamp, 
            g_adc.us_per_sample, 3, BATCH
        );
    }

    if (more) Event_post(syncEvent, SP_THROUGHPUT_EVT);
    return more;
}


void DataStream_updateTelemetry(uint16_t connHandle) {
    // Packed structure (62 bytes total)
    typedef struct __attribute__((packed)) {
        // Timestamp (4 bytes)
        uint32_t timestamp_ms;
        
        // ADC Stats (18 bytes)
        uint32_t adc_samples_processed;
        uint32_t adc_overflow_events;
        uint32_t adc_bytes_sent;
        uint32_t adc_reconfigurations;
        uint16_t adc_peak_buffer_util;
        
        // GYR Stats (18 bytes)
        uint32_t gyr_samples_processed;
        uint32_t gyr_overflow_events;
        uint32_t gyr_bytes_sent;
        uint32_t gyr_reconfigurations;
        uint16_t gyr_peak_buffer_util;
        
        // ACC Stats (18 bytes)
        uint32_t acc_samples_processed;
        uint32_t acc_overflow_events;
        uint32_t acc_bytes_sent;
        uint32_t acc_reconfigurations;
        uint16_t acc_peak_buffer_util;
        
        // System State (4 bytes)
        uint8_t adc_active;
        uint8_t gyr_active;
        uint8_t acc_active;
        uint8_t connection_status;
    } telemetry_data_t;
    
    telemetry_data_t telemetry;
    
    // Fill timestamp
    telemetry.timestamp_ms = Clock_getTicks();
    
    // Fill ADC stats
    ADC_getStats(&g_adc.stats);  // Get current stats
    telemetry.adc_samples_processed = g_adc.stats.samples_processed;
    telemetry.adc_overflow_events = g_adc.stats.overflow_events;
    telemetry.adc_bytes_sent = g_adc.stats.bytes_sent;
    telemetry.adc_reconfigurations = g_adc.stats.reconfigurations;
    telemetry.adc_peak_buffer_util = g_adc.stats.peak_utilization_percent;
    telemetry.adc_active = ADC_isRunning();
    
    // Fill GYR stats
    ISM330_getStats(&g_ism330.stats);
    telemetry.gyr_samples_processed = g_ism330.stats.samples_processed;
    telemetry.gyr_overflow_events = g_ism330.stats.overflow_events;
    telemetry.gyr_bytes_sent = g_ism330.stats.bytes_sent;
    telemetry.gyr_reconfigurations = g_ism330.stats.reconfigurations;
    telemetry.gyr_peak_buffer_util = g_ism330.stats.peak_utilization_percent;
    telemetry.gyr_active = ISM330_isStreaming();
    
    // Fill ACC stats
    H3L_getStats(&g_h3l.stats);
    telemetry.acc_samples_processed = g_h3l.stats.samples_processed;
    telemetry.acc_overflow_events = g_h3l.stats.overflow_events;
    telemetry.acc_bytes_sent = g_h3l.stats.bytes_sent;
    telemetry.acc_reconfigurations = g_h3l.stats.reconfigurations;
    telemetry.acc_peak_buffer_util = g_h3l.stats.peak_utilization_percent;
    telemetry.acc_active = H3L_isStreaming();
    
    // Connection status
    telemetry.connection_status = (connHandle != LINKDB_CONNHANDLE_INVALID);
    
    // Update characteristic
    SimpleProfile_SetParameter(TELEMETRY_CHAR, 
                              sizeof(telemetry_data_t),  // 62 bytes
                              (void*)&telemetry);
}


#ifndef DATA_STREAM_ENGINE_H
#define DATA_STREAM_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include <ti/sysbios/knl/Event.h>
#include "/drivers/ring_buffer.h"

// Initialize the streaming engine (if needed)
void DataStream_init(void);

// The Main Throughput Loop (Call this from simple_peripheral when SP_THROUGHPUT_EVT fires)
bool DataStream_processEvents(uint16_t connHandle);

// Update the telemetry characteristic (Call periodically)
void DataStream_updateTelemetry(uint16_t connHandle);

#endif // DATA_STREAM_ENGINE_H
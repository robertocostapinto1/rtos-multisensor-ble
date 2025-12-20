/* =============================================================================
 *  File:       data_stream_engine.h
 *  
 *  Description:
 *      Defines the interface for the High-Throughput Data Streaming Engine.
 *      This module orchestrates the movement of sensor data from ring buffers
 *      to the BLE stack using Zero-Copy and Adaptive Batching strategies.
 *
 *  Project:    Multi-Sensor Data Acquisition Platform
 *  Author:     José Roberto Costa Pinto
 * =============================================================================
 */

#ifndef DATA_STREAM_ENGINE_H
#define DATA_STREAM_ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>

// -----------------------------------------------------------------------------
// Public API Functions
// -----------------------------------------------------------------------------

/**
 * @brief   Initializes the data stream engine state machines.
 */
void DataStream_init(void);

/**
 * @brief   Main Throughput Loop.
 *          Iterates through all active sensor buffers and attempts to flush
 *          data to the BLE stack using the Peak-Commit strategy.
 * 
 * @param   connHandle  Active BLE connection handle.
 * @return  true        If more data remains (caller should re-schedule event).
 * @return  false       If buffers are empty or radio is full.
 */
bool DataStream_processEvents(uint16_t connHandle);

/**
 * @brief   Updates the Telemetry Characteristic (Performance Metrics).
 *          Should be called periodically (e.g., every 1s or 5s).
 * 
 * @param   connHandle  Active BLE connection handle.
 */
void DataStream_updateTelemetry(uint16_t connHandle);

#ifdef __cplusplus
}
#endif

#endif /* DATA_STREAM_ENGINE_H */
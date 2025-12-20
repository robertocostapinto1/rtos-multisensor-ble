/* =============================================================================
 *  File:       ring_buffer.h
 *  
 *  Description:
 *      A generic, lock-free (single-producer/single-consumer) ring buffer 
 *      implementation optimized for embedded systems.
 *      
 *      Features:
 *      - Static memory allocation support.
 *      - "Drop-Oldest" overwrite strategy for real-time streams.
 *      - Zero-Copy "Peek" and "Commit" semantics.
 *      - Built-in performance metrics.
 * 
 *  Project:    Multi-Sensor Data Acquisition Platform
 * =============================================================================
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h> 
#include <ti/drivers/dpl/HwiP.h>

// -----------------------------------------------------------------------------
// Typedefs & Structures
// -----------------------------------------------------------------------------

/**
 * @brief Performance metrics for the data stream.
 */
typedef struct {
    uint32_t samples_processed;         // Total items written
    uint32_t overflow_events;           // Number of times old data was overwritten
    uint32_t bytes_sent;                // Total bytes successfully committed (read)
    uint32_t reconfigurations;          // Number of sensor config changes
    uint16_t peak_utilization_percent;  // Max buffer usage (0-100%)
} stream_metrics_t;

/**
 * @brief Main Ring Buffer Control Structure.
 */
typedef struct {
    uint8_t*  buffer;       // Pointer to storage array
    uint16_t  size;         // Total size (Must be Power of 2)
    uint16_t  mask;         // Size - 1 (For bitwise wrapping)
    volatile uint16_t head; // Write Index (Producer)
    volatile uint16_t tail; // Read Index (Consumer)
    
    stream_metrics_t* metrics; // Pointer to associated metrics
} ring_buffer_t;


// -----------------------------------------------------------------------------
// Inline Implementation
// -----------------------------------------------------------------------------
/**
 * @brief   Initialize the ring buffer.
 * @param   rb       Pointer to the ring buffer struct.
 * @param   storage  Pointer to the allocated byte array.
 * @param   size     Size of storage (MUST be a Power of 2, e.g., 1024).
 * @param   mets     Pointer to the metrics struct to update.
 */
static inline void rb_init(ring_buffer_t* rb, uint8_t* storage, uint16_t size, stream_metrics_t* mets) 
{
    rb->buffer = storage;
    rb->size = size;
    rb->mask = size - 1;
    rb->head = 0;
    rb->tail = 0;
    rb->metrics = mets;
}


/**
 * @brief   Calculate bytes available for reading.
 * @return  Number of bytes currently in the buffer.
 */
static inline uint16_t rb_available(ring_buffer_t* rb) 
{
    uint16_t h = rb->head;
    uint16_t t = rb->tail;
    return (h >= t) ? (h - t) : (rb->size - t + h);
}

/**
 * @brief   Write data to the buffer using "Drop-Oldest" strategy.
 *          Safe to call from ISR contexts (Single Producer).
 * 
 * @param   data  Source data pointer.
 * @param   len   Length of data to write.
 */
static inline void rb_write(ring_buffer_t* rb, const uint8_t* data, uint16_t len) 
{
    if (len == 0) return;
    if (len > rb->size) return; // Safety check

    // 1. Calculate Usage for Metrics
    uint16_t used = rb_available(rb);
    
    // Update Peak Utilization
    uint16_t current_util = (uint32_t)(used * 100) / rb->size;
    if (current_util > rb->metrics->peak_utilization_percent) {
        rb->metrics->peak_utilization_percent = current_util;
    }

    // 2. Check for Overflow
    // We leave 1 byte sentinel to distinguish full vs empty in some logic, 
    // though mask logic handles full wrap. 
    uint16_t free = rb->size - used - 1; 
    
    if (free < len) 
    {
        // DROP OLDEST STRATEGY:
        // Force the tail forward to make room for new real-time data.
        uint16_t bytes_to_drop = len - free;
        rb->tail = (rb->tail + bytes_to_drop) & rb->mask;
        rb->metrics->overflow_events++;
    }

    // 3. Write Data (Handle Wrap-around)
    uint16_t chunk1 = rb->size - rb->head;
    if (len <= chunk1) 
    {
        memcpy(&rb->buffer[rb->head], data, len);
    } else 
    {
        memcpy(&rb->buffer[rb->head], data, chunk1);
        memcpy(&rb->buffer[0], data + chunk1, len - chunk1);
    }

    // 4. Atomic Head Update
    // Compiler barrier ensures data is written before the index updates.
    uint16_t new_head = (rb->head + len) & rb->mask;
    __asm volatile("" : : : "memory"); 
    rb->head = new_head;
    
    rb->metrics->samples_processed++; 
}

/**
 * @brief   Peek at data without removing it (Zero-Copy Read).
 *          Copies data to 'dest' but does NOT advance the tail.
 * 
 * @param   dest  Destination buffer.
 * @param   len   Bytes to peek.
 */
static inline void rb_peek(ring_buffer_t* rb, uint8_t* dest, uint16_t len) 
{
    uint16_t tail = rb->tail;
    uint16_t chunk1 = rb->size - tail;
    
    if (len <= chunk1) 
    {
        memcpy(dest, &rb->buffer[tail], len);
    } else 
    {
        memcpy(dest, &rb->buffer[tail], chunk1);
        memcpy(dest, &rb->buffer[0], len - chunk1);
    }
}

/**
 * @brief   Commit a read operation (Advance Tail).
 *          Must be called after a successful transmission.
 *          Uses HwiP to ensure atomicity against ISRs.
 * 
 * @param   len   Number of bytes to discard/advance.
 */
static inline void rb_commit(ring_buffer_t* rb, uint16_t len) 
{
    uintptr_t key = HwiP_disable(); 
    rb->tail = (rb->tail + len) & rb->mask;
    rb->metrics->bytes_sent += len;
    HwiP_restore(key);
}

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
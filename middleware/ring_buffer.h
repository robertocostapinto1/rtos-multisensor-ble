#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h> 
#include <ti/drivers/dpl/HwiP.h>

// Metrics structure for Thesis
typedef struct {
    uint32_t samples_processed;
    uint32_t overflow_events;
    uint32_t bytes_sent;
    uint32_t reconfigurations;
    uint16_t peak_utilization_percent; // 0-100%
} stream_metrics_t;

// Generic Ring Buffer Object
typedef struct {
    uint8_t*  buffer;       // Pointer to storage array
    uint16_t  size;         // Total size (Must be Power of 2)
    uint16_t  mask;         // Size - 1
    volatile uint16_t head; // Write Index
    volatile uint16_t tail; // Read Index
    
    // Pointer to the metrics struct in the parent driver
    stream_metrics_t* metrics; 
} ring_buffer_t;

// Init
static inline void rb_init(ring_buffer_t* rb, uint8_t* storage, uint16_t size, stream_metrics_t* mets) 
{
    rb->buffer = storage;
    rb->size = size;
    rb->mask = size - 1;
    rb->head = 0;
    rb->tail = 0;
    rb->metrics = mets;
}

// Check available bytes for READING
static inline uint16_t rb_available(ring_buffer_t* rb) 
{
    uint16_t h = rb->head;
    uint16_t t = rb->tail;
    return (h >= t) ? (h - t) : (rb->size - t + h);
}

// ISR-Safe Write with "Drop Oldest" Strategy
// Updates Peak Utilization and Overflow metrics automatically
static inline void rb_write(ring_buffer_t* rb, const uint8_t* data, uint16_t len) 
{
    if (len == 0) return;
    if (len > rb->size) return; // Sanity check

    // 1. Calculate Usage for Metrics
    uint16_t used = rb_available(rb);
    
    // Update Peak Utilization (Simple calculation for efficiency)
    // To save division in ISR, we approximate or do it simply: (used * 100) / size
    uint16_t current_util = (uint32_t)(used * 100) / rb->size;
    if (current_util > rb->metrics->peak_utilization_percent) {
        rb->metrics->peak_utilization_percent = current_util;
    }

    // 2. Check for Overflow (Space needed vs Space free)
    uint16_t free = rb->size - used - 1; // Keep 1 byte sentinel
    
    if (free < len) 
    {
        // DROP OLDEST: Advance tail to make room
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
    uint16_t new_head = (rb->head + len) & rb->mask;
    __asm volatile("" : : : "memory"); // Compiler barrier
    rb->head = new_head;
    rb->metrics->samples_processed++; // Or increment by bytes, depends on your definition
}

// "Peek" Copy: Copies data to destination but DOES NOT move tail.
// Returns bytes actually copied.
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

// Commit Read: Advances tail after successful BLE send
static inline void rb_commit(ring_buffer_t* rb, uint16_t len) 
{
    uintptr_t key = HwiP_disable(); // Short lock to update tail safely
    rb->tail = (rb->tail + len) & rb->mask;
    rb->metrics->bytes_sent += len;
    HwiP_restore(key);
}

#endif
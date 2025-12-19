#ifndef SENSOR_PLATFORM_H
#define SENSOR_PLATFORM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* TI Drivers */
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/knl/Semaphore.h>

#include "ti_drivers_config.h"

// I2C config (from your original code)
#ifndef CONFIG_I2C_0
#error "CONFIG_I2C_0 not defined - adjust board config ID"
#endif

// Maximum stacked I2C TX buffer we allow (no VLA). ST register writes are short — 32 is safe. 
// CRITICAL UPDATE: Increased for FIFO Burst Reads
// H3L (32 samples * 6 bytes = 192)
// ISM (50 samples * 6 bytes = 300) -> Set to 512 for safety
#define MAX_I2C_XFER            512 

// Platform bus structure for I2C
typedef struct {
    I2C_Handle i2c;            /* shared bus handle */
    uint16_t targetAddr7;      /* 7-bit address for TI I2C */
    bool autoInc;              /* if true, OR reg with 0x80 for multi-byte I2C */
} platform_i2c_bus_t;

// ============================================================================
// PUBLIC API
// ============================================================================

void SensorsPlatform_open(void);
void SensorsPlatform_init(void);
void SensorsPlatform_delay(uint32_t ms);
int32_t I2C_platformWrite(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
int32_t I2C_platformRead(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

// Global platform resources (from your original code)
extern platform_i2c_bus_t gIsmBus;
extern platform_i2c_bus_t gH3lBus;

extern Semaphore_Handle gI2CMutex;
extern I2C_Handle g_i2c_handle;

#endif // SENSOR_PLATFORM_H
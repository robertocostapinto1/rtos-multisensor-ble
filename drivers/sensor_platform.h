/*
 * =============================================================================
 *  File:       sensor_platform.h
 *  
 *  Description:
 *      Hardware Abstraction Layer (HAL) for the Sensor Platform.
 *      Manages shared resources (I2C Bus) and defines system-wide constants.
 * =============================================================================
 */

#ifndef SENSOR_PLATFORM_H
#define SENSOR_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* TI Drivers */
#include <ti/drivers/I2C.h>
#include <ti/drivers/GPIO.h>
#include <ti/sysbios/knl/Semaphore.h>

#include "ti_drivers_config.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// Validates board configuration
#ifndef CONFIG_I2C_0
#error "CONFIG_I2C_0 not defined - adjust board config ID"
#endif

// Maximum I2C buffer size.
// Critical for FIFO burst reads (ISM330 requires up to 300+ bytes)
#define MAX_I2C_XFER            512 

// -----------------------------------------------------------------------------
// Structures
// -----------------------------------------------------------------------------

/**
 * @brief Abstraction for an I2C device on the shared bus.
 */
typedef struct {
    I2C_Handle i2c;            /* Shared bus handle */
    uint16_t targetAddr7;      /* 7-bit I2C address */
    bool autoInc;              /* Logic flag: does this sensor need MSB set for auto-inc? */
} platform_i2c_bus_t;

// -----------------------------------------------------------------------------
// Global Resources
// -----------------------------------------------------------------------------
extern platform_i2c_bus_t gIsmBus;
extern platform_i2c_bus_t gH3lBus;

extern Semaphore_Handle gI2CMutex;
extern I2C_Handle g_i2c_handle;

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

/**
 * @brief   Initializes I2C, GPIOs, and Synchronization Primitives.
 */
void SensorsPlatform_init(void);

/**
 * @brief   Blocking delay (wraps Task_sleep).
 * @param   ms  Milliseconds to sleep.
 */
void SensorsPlatform_delay(uint32_t ms);

/**
 * @brief   Performs an I2C Write transaction.
 *          Thread-safe (Protected by Mutex).
 * 
 * @param   handle  Pointer to platform_i2c_bus_t.
 * @param   reg     Register address.
 * @param   bufp    Data to write.
 * @param   len     Number of bytes.
 * @return  0 on success, -1 on failure.
 */
int32_t I2C_platformWrite(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);

/**
 * @brief   Performs an I2C Read transaction.
 *          Thread-safe (Protected by Mutex).
 * 
 * @param   handle  Pointer to platform_i2c_bus_t.
 * @param   reg     Register address.
 * @param   bufp    Buffer to store read data.
 * @param   len     Number of bytes.
 * @return  0 on success, -1 on failure.
 */
int32_t I2C_platformRead(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_PLATFORM_H
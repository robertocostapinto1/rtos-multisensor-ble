/* =============================================================================
 *  File:       sensor_platform.c
 *  
 *  Description:
 *      Implementation of the Hardware Abstraction Layer.
 *      Handles I2C arbitration (Mutex), GPIO configuration, and Driver Handles.
 * =============================================================================
 */

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <string.h>
#include <stdio.h>
#include <xdc/runtime/System.h>

/* RTOS & Drivers */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/I2C.h>

/* Project Headers */
#include "sensor_platform.h"
#include "/vendor/h3lis331dl_reg.h" // For address definitions

// -----------------------------------------------------------------------------
// Constants & Definitions
// -----------------------------------------------------------------------------

/* Address Translation */
#define H3L_I2C_ADDR_7BIT        (H3LIS331DL_I2C_ADD_H >> 1) 
#define ISM330_I2C_ADDR_7BIT     0x6B

// -----------------------------------------------------------------------------
// Global Resources
// -----------------------------------------------------------------------------

// Bus Descriptors
platform_i2c_bus_t gIsmBus = {0};
platform_i2c_bus_t gH3lBus = {0};

// Synchronization
Semaphore_Handle gI2CMutex = NULL;
I2C_Handle g_i2c_handle = NULL;

// Static buffer prevents Stack Overflow in the Task
// (Allocated in .bss rather than .stack)
static uint8_t g_i2c_tx_buf[MAX_I2C_XFER + 1];

// -----------------------------------------------------------------------------
// Private Functions
// -----------------------------------------------------------------------------

static void I2C_platformOpen(void)
{
    I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);
    
    // High Speed I2C (1 MHz) for throughput
    i2cParams.bitRate = I2C_1000kHz; 

    I2C_init();
    g_i2c_handle = I2C_open(CONFIG_I2C_0, &i2cParams);
    
    if (g_i2c_handle == NULL) {
        // Fatal Error: I2C failed to open
        while (1);
    }

    /* Create Mutex for shared bus access */
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    gI2CMutex = Semaphore_create(1, &semParams, NULL);
}

// -----------------------------------------------------------------------------
// Public API Implementation
// -----------------------------------------------------------------------------

void SensorsPlatform_init(void)
{
    // 1. Initialize Bus
    I2C_platformOpen();

    // 2. Configure ISM330 Descriptor
    gIsmBus.i2c = g_i2c_handle;
    gIsmBus.targetAddr7 = ISM330_I2C_ADDR_7BIT;
    gIsmBus.autoInc = false; // ISM handles auto-inc internally/via register choice

    // 3. Configure H3L Descriptor
    gH3lBus.i2c = g_i2c_handle;
    gH3lBus.targetAddr7 = H3L_I2C_ADDR_7BIT; 
    gH3lBus.autoInc = true;  // H3L requires MSB set for auto-inc

    // 4. Configure GPIO Interrupts
    GPIO_init();

    GPIO_setConfig(CONFIG_GPIO_INT_ISM,
      GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING);

    GPIO_setConfig(CONFIG_GPIO_INT_H3L,
      GPIO_CFG_IN_NOPULL  | GPIO_CFG_IN_INT_RISING);

    printf("Sensor Platform initialized!\n");
}

void SensorsPlatform_delay(uint32_t ms)
{
    Task_sleep(ms); // Uses BIOS ticks - adjust if your tick rate differs
}


int32_t I2C_platformWrite(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    platform_i2c_bus_t *bus = (platform_i2c_bus_t *)handle;
    if (!bus || !bus->i2c) return -1;
    if (len > MAX_I2C_XFER) return -1;

    // Acquire Mutex
    if (gI2CMutex) Semaphore_pend(gI2CMutex, BIOS_WAIT_FOREVER);

    // Handle Hardware-Specific Auto-Increment logic
    uint8_t reg_byte = reg;
    if (bus->autoInc && len > 1) {
        reg_byte |= 0x80;
    }

    // Prepare Transaction
    // Use static global buffer to save stack space
    g_i2c_tx_buf[0] = reg_byte;
    memcpy(&g_i2c_tx_buf[1], bufp, len);

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.writeBuf = g_i2c_tx_buf;
    trans.writeCount = len + 1;
    trans.readBuf = NULL;
    trans.readCount = 0;
    trans.targetAddress = bus->targetAddr7;

    bool ok = I2C_transfer(bus->i2c, &trans);

    // Release Mutex
    if (gI2CMutex) Semaphore_post(gI2CMutex);
    
    return ok ? 0 : -1;
}

int32_t I2C_platformRead(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    platform_i2c_bus_t *bus = (platform_i2c_bus_t *)handle;
    if (!bus || !bus->i2c) return -1;

    // Acquire Mutex
    if (gI2CMutex) Semaphore_pend(gI2CMutex, BIOS_WAIT_FOREVER);

    uint8_t reg_byte = reg;
    if (bus->autoInc && len > 1) {
        reg_byte |= 0x80;
    }

    // Use static buffer for the write byte
    g_i2c_tx_buf[0] = reg_byte;

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.writeBuf = g_i2c_tx_buf;
    trans.writeCount = 1;
    trans.readBuf = bufp;
    trans.readCount = len;
    trans.targetAddress = bus->targetAddr7;

    bool ok = I2C_transfer(bus->i2c, &trans);

    // Release Mutex
    if (gI2CMutex) Semaphore_post(gI2CMutex);
    
    return ok ? 0 : -1;
}
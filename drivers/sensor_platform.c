#include "sensor_platform.h"
#include "h3lis331dl_reg.h"
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/I2C.h>
#include <string.h>
#include <stdio.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/System.h>
#include </drivers/h3l_driver.h>

/* H3L ST header gives 8-bit addresses; we convert to 7-bit (>>1) */
#define H3L_I2C_ADDR_7BIT        (H3LIS331DL_I2C_ADD_H >> 1)  /* use _ADD_H >>1 if SA0=1 */
#define ISM330_I2C_ADDR_7BIT 0x6B

// Global platform resources
platform_i2c_bus_t gIsmBus = {0};
platform_i2c_bus_t gH3lBus = {0};
Semaphore_Handle gI2CMutex = NULL;
Semaphore_Handle gIsmDRDYSem = NULL;
Semaphore_Handle gH3lDRDYSem = NULL;
I2C_Handle g_i2c_handle = NULL;

// Static buffer prevents Stack Overflow in the Task
static uint8_t g_i2c_tx_buf[MAX_I2C_XFER + 1];

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

void I2C_platformOpen(void)
{
    I2C_Params i2cParams;
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_1000kHz;

    I2C_init();
    g_i2c_handle = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (g_i2c_handle == NULL) {
        // fatal — replace with application error handling
        while (1);
    }

    /* create mutex to protect I2C transfers */
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    gI2CMutex = Semaphore_create(1, &semParams, NULL);

}

void SensorsPlatform_init(void)
{
    I2C_platformOpen();

    // Configure ISM device bus descriptor
    gIsmBus.i2c = g_i2c_handle;
    gIsmBus.targetAddr7 = ISM330_I2C_ADDR_7BIT;
    gIsmBus.autoInc = false;

    // Configure H3L device bus descriptor
    gH3lBus.i2c = g_i2c_handle;
    gH3lBus.targetAddr7 = H3L_I2C_ADDR_7BIT; 
    gH3lBus.autoInc = true;

    GPIO_init();

    GPIO_setConfig(CONFIG_GPIO_INT_ISM,
      GPIO_CFG_IN_NOPULL | GPIO_CFG_IN_INT_RISING);

    GPIO_setConfig(CONFIG_GPIO_INT_H3L,
      GPIO_CFG_IN_NOPULL  | GPIO_CFG_IN_INT_RISING);

    printf("Sensor Platform initialized! \n");

}

void SensorsPlatform_delay(uint32_t ms)
{
    Task_sleep(ms); // Uses BIOS ticks - adjust if your tick rate differs
}

/*int32_t I2C_platformWrite(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    platform_i2c_bus_t *bus = (platform_i2c_bus_t *)handle;
    if (!bus || !bus->i2c) return -1;
    if (len > MAX_I2C_XFER) return -1;

    // ⚠️ Check for buffer overrun
    if (len > MAX_I2C_XFER) {
        printf("CRITICAL: I2C WRITE TOO LONG: %u bytes\n", len);
        return -1;
    }
    
    uint8_t txBuf[MAX_I2C_XFER + 1];

    // Protect the shared transfer resources 
    if (gI2CMutex) Semaphore_pend(gI2CMutex, BIOS_WAIT_FOREVER);

    // prepare register byte with optional auto-increment 
    uint8_t reg_byte = reg;
    if (bus->autoInc && len > 1) {
        reg_byte |= 0x80;
    }

    // static local tx buffer - safe because we hold gI2CMutex 
    //uint8_t txBuf[MAX_I2C_XFER + 1];
    txBuf[0] = reg_byte;
    memcpy(&txBuf[1], bufp, len);

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.writeBuf = txBuf;
    trans.writeCount = len + 1;
    trans.readBuf = NULL;
    trans.readCount = 0;
    trans.targetAddress = bus->targetAddr7;

    bool ok = I2C_transfer(bus->i2c, &trans);

    if (gI2CMutex) Semaphore_post(gI2CMutex);
    return ok ? 0 : -1;
}

int32_t I2C_platformRead(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    platform_i2c_bus_t *bus = (platform_i2c_bus_t *)handle;
    if (!bus || !bus->i2c) return -1;

    if (gI2CMutex) {
        if (!Semaphore_pend(gI2CMutex, 100)) { // 100ms timeout
            printf("I2C BUS DEADLOCK - releasing\n");
            return -1;
        }
    }
    uint8_t reg_byte = reg;
    if (bus->autoInc && len > 1) {
        reg_byte |= 0x80;
    }

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.writeBuf = &reg_byte;
    trans.writeCount = 1;
    trans.readBuf = bufp;
    trans.readCount = len;
    trans.targetAddress = bus->targetAddr7;

    bool ok = I2C_transfer(bus->i2c, &trans);

    if (gI2CMutex) Semaphore_post(gI2CMutex);
    return ok ? 0 : -1;
}*/

int32_t I2C_platformWrite(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    platform_i2c_bus_t *bus = (platform_i2c_bus_t *)handle;
    if (!bus || !bus->i2c) return -1;
    if (len > MAX_I2C_XFER) return -1;

    if (gI2CMutex) Semaphore_pend(gI2CMutex, BIOS_WAIT_FOREVER);

    uint8_t reg_byte = reg;
    if (bus->autoInc && len > 1) {
        reg_byte |= 0x80;
    }

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

    if (gI2CMutex) Semaphore_post(gI2CMutex);
    return ok ? 0 : -1;
}

int32_t I2C_platformRead(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    platform_i2c_bus_t *bus = (platform_i2c_bus_t *)handle;
    if (!bus || !bus->i2c) return -1;

    if (gI2CMutex) Semaphore_pend(gI2CMutex, BIOS_WAIT_FOREVER);

    uint8_t reg_byte = reg;
    if (bus->autoInc && len > 1) {
        reg_byte |= 0x80;
    }

    // Use the static buffer for the write part of the read transaction too
    // This is safer than using a local stack variable address
    g_i2c_tx_buf[0] = reg_byte;

    I2C_Transaction trans;
    memset(&trans, 0, sizeof(trans));
    trans.writeBuf = g_i2c_tx_buf;
    trans.writeCount = 1;
    trans.readBuf = bufp;
    trans.readCount = len;
    trans.targetAddress = bus->targetAddr7;

    bool ok = I2C_transfer(bus->i2c, &trans);

    if (gI2CMutex) Semaphore_post(gI2CMutex);
    return ok ? 0 : -1;
}
// h3l_driver.c - H3LIS331DL High-G Accelerometer Driver
#include "h3l_driver.h"
#include "ism330_driver.h"
#include <string.h>
#include <stdio.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h" 
#include <ti/sysbios/BIOS.h>

const uint16_t acc_odr_map[] = {
    H3LIS331DL_ODR_Hz5, 
    H3LIS331DL_ODR_1Hz, 
    H3LIS331DL_ODR_2Hz,
    H3LIS331DL_ODR_5Hz, 
    H3LIS331DL_ODR_10Hz, 
    H3LIS331DL_ODR_50Hz,
    H3LIS331DL_ODR_100Hz, 
    H3LIS331DL_ODR_400Hz, 
    H3LIS331DL_ODR_1kHz
};

const float acc_odr_freq[] = {
    0.5, 1, 2, 5, 10, 50, 100, 400, 1000
};

// Global driver context
h3l_driver_t g_h3l = {0};
Semaphore_Handle h3lGlobalSem = NULL;

// PRIVATE HELPERS
static inline bool h3l_clean_reset(void);
static inline bool h3l_configure_interrupts(bool enable);
static inline bool h3l_enable_axes(bool enable);
static void H3L_interruptCallback(uint_least8_t index);

extern uint32_t getMicrosecondTimestamp(void);

// ============================================================================
// DIRECT DRIVER IMPLEMENTATION
// ============================================================================

bool H3L_init(platform_i2c_bus_t *i2c_bus)
{
    if(g_h3l.initialized) return true;
    if(!i2c_bus) return false;

    // 1. Clear Context FIRST
    memset(&g_h3l, 0, sizeof(h3l_driver_t));

    // 2. Init Ring Buffer
    rb_init(&g_h3l.buffer, 
            g_h3l.storage, 
            sizeof(g_h3l.storage), 
            &g_h3l.stats);

    // Create global semaphore ONCE
    if (h3lGlobalSem == NULL) {
        Semaphore_Params semParams;
        Semaphore_Params_init(&semParams);
        //semParams.mode = Semaphore_Mode_BINARY_PRIORITY;
        
        h3lGlobalSem = Semaphore_create(0, &semParams, NULL);
        
        if (!h3lGlobalSem) {
            printf("H3L: Cannot create global semaphore\n");
            return false;
        }
    }
    // Assign to context
    g_h3l.drdy_sem = h3lGlobalSem;

    // 4. Setup I2C
    g_h3l.dev_ctx.write_reg = I2C_platformWrite;
    g_h3l.dev_ctx.read_reg = I2C_platformRead;
    g_h3l.dev_ctx.mdelay = SensorsPlatform_delay;
    g_h3l.dev_ctx.handle = i2c_bus;
    g_h3l.i2c_bus = i2c_bus;

    // 5. Hardware Verify
    uint8_t whoamI = 0;
    if(h3lis331dl_device_id_get(&g_h3l.dev_ctx, &whoamI) != 0) 
    {
        printf("H3L: I2C COMMUNICATION FAILED\n");
        return false;  
    }
    if (whoamI != H3LIS331DL_ID) 
    {
        printf("H3L: WRONG DEVICE ID: 0x%02X\n", whoamI);
        return false;
    }

    // 6. Reset & Config
    h3l_clean_reset();  

    // Configure device  
    h3lis331dl_block_data_update_set(&g_h3l.dev_ctx, PROPERTY_ENABLE);
    h3lis331dl_pin_mode_set(&g_h3l.dev_ctx, H3LIS331DL_PUSH_PULL);
    h3lis331dl_pin_polarity_set(&g_h3l.dev_ctx, H3LIS331DL_ACTIVE_HIGH);
    
    h3lis331dl_data_rate_set(&g_h3l.dev_ctx, H3LIS331DL_ODR_OFF);
    h3lis331dl_full_scale_set(&g_h3l.dev_ctx, H3LIS331DL_400g);

    h3lis331dl_int1_notification_set(&g_h3l.dev_ctx, H3LIS331DL_INT1_LATCHED);
    h3lis331dl_pin_int1_route_set(&g_h3l.dev_ctx, H3LIS331DL_PAD1_DRDY);
        
    // Register GPIO
    GPIO_setCallback(CONFIG_GPIO_INT_H3L, H3L_interruptCallback);

    g_h3l.odr_index = 0;
    g_h3l.initialized = true;
    g_h3l.powered = false;
    
    printf("[H3L] Driver initialized! \n");

    return true;
}

bool H3L_startStreaming(void)
{
    if(!g_h3l.initialized || g_h3l.powered) return false;

    H3L_resetStats();
    g_h3l.powered = true;
   
    // Set ODR 
    if(h3lis331dl_data_rate_set(&g_h3l.dev_ctx, acc_odr_map[g_h3l.odr_index]) != 0) return false;
    
    // Enable axes
    if (!h3l_enable_axes(true)) return false;
  
    // Enable Interrupts
    if (!h3l_configure_interrupts(true)) return false;

    // Enable GPIO
    GPIO_enableInt(CONFIG_GPIO_INT_H3L);

    // CALC TIMEOUT & PERIOD (Float Precision)
    float freq = acc_odr_freq[g_h3l.odr_index];
    if (freq > 0) {
        g_h3l.us_per_sample = 1000000.0 / (double)freq;
        
        // Base ticks = Period in microseconds / 10 (assuming 10us tick)
        float base_ticks = (float)g_h3l.us_per_sample / 10.0f;
        
        // Store the exact period in ticks. 
        // The Task applies the +20% margin dynamically.
        g_h3l.timeout_ticks = (uint32_t)base_ticks;
        
        if (g_h3l.timeout_ticks == 0) g_h3l.timeout_ticks = 1;
    } 

    return true;
}

bool H3L_stopStreaming(void)
{
    if (!g_h3l.initialized || !g_h3l.powered) return false;

    GPIO_disableInt(CONFIG_GPIO_INT_H3L);
    h3l_configure_interrupts(false);
    h3lis331dl_data_rate_set(&g_h3l.dev_ctx, H3LIS331DL_ODR_OFF);
    h3l_enable_axes(false);
    
    g_h3l.powered = false;

    return true;
}

bool H3L_isStreaming(void)
{
    return g_h3l.powered; 
}

bool H3L_reconfigureOdr(uint32_t index)
{
    if (!g_h3l.initialized || index >= 9) return false;
    
    g_h3l.odr_index = index; 
    g_h3l.stats.reconfigurations++;


    if (g_h3l.powered)
    {
       float freq = acc_odr_freq[g_h3l.odr_index];
        if (freq > 0) {
            g_h3l.us_per_sample = 1000000.0 / (double)freq;
            
            // Base ticks = Period in microseconds / 10 (assuming 10us tick)
            float base_ticks = (float)g_h3l.us_per_sample / 10.0f;
            
            // Store the exact period in ticks. 
            // The Task applies the +20% margin dynamically.
            g_h3l.timeout_ticks = (uint32_t)base_ticks;
            
            if (g_h3l.timeout_ticks == 0) g_h3l.timeout_ticks = 1;
        } 

        // 3. Write H3L Register
        return (h3lis331dl_data_rate_set(&g_h3l.dev_ctx, acc_odr_map[g_h3l.odr_index]) == 0);
    } 

    return true;
}


// ============================================================================
// DATA PATH
// ============================================================================

// DATA PATH WITH RE-SYNC LOGIC
void H3L_writeBuffer(uint8_t* data, uint16_t len)
{
    if (g_h3l.powered) 
    {
        // RE-SYNC
        if (rb_available(&g_h3l.buffer) == 0) 
        {
            g_h3l.oldest_sample_timestamp = (double)getMicrosecondTimestamp();
        }

        rb_write(&g_h3l.buffer, data, len);
    }
}

uint16_t H3L_getAvailableBytes(void)
{
    return rb_available(&g_h3l.buffer);
}

void H3L_getStats(stream_metrics_t* stats_out)
{
    if (stats_out) *stats_out = g_h3l.stats;
}

void H3L_resetStats(void) 
{
    memset(&g_h3l.stats, 0, sizeof(stream_metrics_t));
    g_h3l.buffer.head = 0;
    g_h3l.buffer.tail = 0;
}

// ============================================================================
// HELPERS & INTERRUPTS
// ============================================================================

static void H3L_interruptCallback(uint_least8_t index)
{   
    if (g_h3l.initialized && g_h3l.powered && g_h3l.drdy_sem) 
    {
        Semaphore_post(h3lGlobalSem);
    }
}

static inline bool h3l_clean_reset(void)
{
    // Register resets
    uint8_t zero = 0x00;
    h3lis331dl_write_reg(&g_h3l.dev_ctx, H3LIS331DL_CTRL_REG1, &zero, 1);
    h3lis331dl_write_reg(&g_h3l.dev_ctx, H3LIS331DL_CTRL_REG3, &zero, 1);
    h3lis331dl_write_reg(&g_h3l.dev_ctx, H3LIS331DL_INT1_CFG, &zero, 1);
    h3lis331dl_write_reg(&g_h3l.dev_ctx, H3LIS331DL_CTRL_REG2, &zero, 1);
    SensorsPlatform_delay(50);
    return true;
}

static inline bool h3l_configure_interrupts(bool enable)
{
    uint8_t ctrl_reg3;
    
    // Read current CTRL_REG3
    if (h3lis331dl_read_reg(&g_h3l.dev_ctx, H3LIS331DL_CTRL_REG3, &ctrl_reg3, 1) != 0) 
    {
        return false;
    }
    
    // Set or clear I1_DRDY bit (bit 4)
    if (enable) {
        ctrl_reg3 |= (1 << 4);   // Enable DRDY interrupt on INT1
    } else {
        ctrl_reg3 &= ~(1 << 4);  // Disable DRDY interrupt
    }
    
    // Write back
    return (h3lis331dl_write_reg(&g_h3l.dev_ctx, H3LIS331DL_CTRL_REG3, &ctrl_reg3, 1) == 0);
}


static inline bool h3l_enable_axes(bool enable) 
{
    uint8_t ctrl_reg1;
    
    if (h3lis331dl_read_reg(&g_h3l.dev_ctx, H3LIS331DL_CTRL_REG1, &ctrl_reg1, 1) != 0) {
        return false;
    }
    
    if (enable) {
        ctrl_reg1 |= 0x07;  // Enable X, Y, Z axes
    } else {
        ctrl_reg1 &= ~0x07; // Disable X, Y, Z axes
    }
    
    return (h3lis331dl_write_reg(&g_h3l.dev_ctx, H3LIS331DL_CTRL_REG1, &ctrl_reg1, 1) == 0);
}
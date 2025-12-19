// ism330_driver.c - ISM330DHCX Gyroscope Driver
#include "ism330_driver.h"
#include "h3l_driver.h"
#include <string.h>
#include <stdio.h>
#include <ti/drivers/dpl/HwiP.h>
#include <ti/drivers/GPIO.h>
#include "ti_drivers_config.h" 

const uint16_t gyr_odr_map[] = {
    ISM330DHCX_GY_ODR_12Hz5,   
    ISM330DHCX_GY_ODR_26Hz,    
    ISM330DHCX_GY_ODR_52Hz,    
    ISM330DHCX_GY_ODR_104Hz,   
    ISM330DHCX_GY_ODR_208Hz,   
    ISM330DHCX_GY_ODR_416Hz,   
    ISM330DHCX_GY_ODR_833Hz,   
    ISM330DHCX_GY_ODR_1666Hz,  
    ISM330DHCX_GY_ODR_3332Hz,  
    ISM330DHCX_GY_ODR_6667Hz   
};

const float gyr_odr_freq[] = {
    12.5, 26, 52, 104, 208, 416, 833, 1666, 3332, 6667
};

// Global driver context
ism330_driver_t g_ism330 = {0};

// PRIVATE HELPERS
static bool ism_configure_interrupts(bool enable);
static void ISM330_interruptCallback(uint_least8_t index);

static uint16_t get_optimal_watermark(void);
extern uint32_t getMicrosecondTimestamp(void);

// ============================================================================
// DIRECT DRIVER IMPLEMENTATION
// ============================================================================

bool ISM330_init(platform_i2c_bus_t *i2c_bus) {
    if (g_ism330.initialized) return true;
    memset(&g_ism330, 0, sizeof(ism330_driver_t));

    rb_init(&g_ism330.buffer, g_ism330.storage, sizeof(g_ism330.storage), &g_ism330.stats);

    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    g_ism330.drdy_sem = Semaphore_create(0, &semParams, NULL);

    g_ism330.dev_ctx.write_reg = I2C_platformWrite;
    g_ism330.dev_ctx.read_reg = I2C_platformRead;
    g_ism330.dev_ctx.mdelay = SensorsPlatform_delay;
    g_ism330.dev_ctx.handle = i2c_bus;
    g_ism330.i2c_bus = i2c_bus;

    // 1. CHIP ID CHECK
    uint8_t whoamI = 0;
    for(int i=0; i<5; i++) {
        ism330dhcx_device_id_get(&g_ism330.dev_ctx, &whoamI);
        if(whoamI == ISM330DHCX_ID) break;
        SensorsPlatform_delay(10);
    }

    if (whoamI != ISM330DHCX_ID) {
        printf("ISM330: FAIL ID 0x%02X\n", whoamI);
        return false;
    }

    // 2. RESET
    uint8_t rst;
    ism330dhcx_reset_set(&g_ism330.dev_ctx, PROPERTY_ENABLE);
    do { ism330dhcx_reset_get(&g_ism330.dev_ctx, &rst); } while (rst);

    // 3. MANDATORY CONFIG
    ism330dhcx_device_conf_set(&g_ism330.dev_ctx, PROPERTY_ENABLE);
    ism330dhcx_block_data_update_set(&g_ism330.dev_ctx, PROPERTY_ENABLE);
    
    // 4. PIN CONFIG (Push-Pull, Active High)
    ism330dhcx_pin_mode_set(&g_ism330.dev_ctx, ISM330DHCX_PUSH_PULL);
    ism330dhcx_pin_polarity_set(&g_ism330.dev_ctx, ISM330DHCX_ACTIVE_HIGH);

    g_ism330.odr_index = 0; 
    g_ism330.initialized = true;

    GPIO_setCallback(CONFIG_GPIO_INT_ISM, ISM330_interruptCallback);

    printf("[ISM] Driver initialized! \n");

    return true;
}

bool ISM330_startStreaming(void) {
    if (!g_ism330.initialized) return false;

    // --- CRITICAL CONFIGURATION SEQUENCE ---

    // 1. Reset FIFO to Bypass (Flush everything)
    ism330dhcx_fifo_mode_set(&g_ism330.dev_ctx, ISM330DHCX_BYPASS_MODE);

    // 2. Set Watermark Initial State
    //g_ism330.current_watermark = get_optimal_watermark();
    //ism330dhcx_fifo_watermark_set(&g_ism330.dev_ctx, g_ism330.current_watermark);
    ism330dhcx_fifo_watermark_set(&g_ism330.dev_ctx, ISM_FIFO_WTM_WORDS);

    // 3. Configure Batching (BDR) 
    // We match BDR to the desired ODR.
    ism330dhcx_fifo_xl_batch_set(&g_ism330.dev_ctx, ISM330DHCX_XL_NOT_BATCHED);
    ism330dhcx_fifo_gy_batch_set(&g_ism330.dev_ctx, (ism330dhcx_bdr_gy_t)gyr_odr_map[g_ism330.odr_index]);
    
    // 4. Disable Compression & Aux data (Keep data structure strict: 7 bytes)
    ism330dhcx_fifo_timestamp_decimation_set(&g_ism330.dev_ctx, ISM330DHCX_NO_DECIMATION);
    ism330dhcx_fifo_temp_batch_set(&g_ism330.dev_ctx, ISM330DHCX_TEMP_NOT_BATCHED);
    ism330dhcx_compression_algo_set(&g_ism330.dev_ctx, ISM330DHCX_CMP_DISABLE);

    // 5. Route FIFO Threshold to INT1
    ism330dhcx_pin_int1_route_t int1_route = {0};
    int1_route.int1_ctrl.int1_fifo_th = 1; // Assert INT1 when FIFO reaches WTM
    ism330dhcx_pin_int1_route_set(&g_ism330.dev_ctx, &int1_route);

    // 6. Set FIFO Mode to Stream
    ism330dhcx_fifo_mode_set(&g_ism330.dev_ctx, ISM330DHCX_STREAM_MODE);

    // 7. Enable Sensor ODR (Wake up sensor)
    ism330dhcx_gy_data_rate_set(&g_ism330.dev_ctx, gyr_odr_map[g_ism330.odr_index]);
    ism330dhcx_gy_full_scale_set(&g_ism330.dev_ctx, ISM330DHCX_4000dps);

    // Clear stats
    memset(&g_ism330.stats, 0, sizeof(stream_metrics_t));
    g_ism330.powered = true;
        
    GPIO_enableInt(CONFIG_GPIO_INT_ISM);

    // FLOAT PRECISION MATH
    float freq = gyr_odr_freq[g_ism330.odr_index];
    if (freq > 0) {
        g_ism330.us_per_sample = 1000000.0 / (double)freq;
    } else {
        g_ism330.us_per_sample = 0.0;
    }

    // Reset buffer & stats
    ISM330_resetStats();

    return true;
}

bool ISM330_stopStreaming(void) {
    GPIO_disableInt(CONFIG_GPIO_INT_ISM);
    
    // Stop Sensor (ODR = 0)
    ism330dhcx_gy_data_rate_set(&g_ism330.dev_ctx, ISM330DHCX_GY_ODR_OFF);
    // Stop FIFO (BDR = 0)
    ism330dhcx_fifo_gy_batch_set(&g_ism330.dev_ctx, ISM330DHCX_GY_NOT_BATCHED);
    // Reset FIFO Mode
    ism330dhcx_fifo_mode_set(&g_ism330.dev_ctx, ISM330DHCX_BYPASS_MODE);
    
    g_ism330.powered = false;
    return true;
}

// Optimized Block Read
int32_t ISM330_readFifoBatch_Block(uint8_t* buffer, uint16_t total_bytes) {
    // Reads from 0x78 (TAG). 
    // IMPORTANT: The Platform I2C write must set the auto-increment bit (0x80)
    // The ST driver handles this in 'ism330dhcx_read_reg'
    return ism330dhcx_read_reg(&g_ism330.dev_ctx, ISM330DHCX_FIFO_DATA_OUT_TAG, buffer, total_bytes);
}

bool ISM330_isStreaming(void)
{
    return g_ism330.powered; 
}


bool ISM330_reconfigureOdr(uint32_t index)
{
    if (!g_ism330.initialized || index >= 10) return false;

    g_ism330.odr_index = index; 
    g_ism330.stats.reconfigurations++;

    if (g_ism330.powered) 
    {
        // 1. Update Timing
        float freq = gyr_odr_freq[g_ism330.odr_index];
        if (freq > 0) g_ism330.us_per_sample = 1000000.0 / (double)freq;

        // 2. Update BDR
        ism330dhcx_fifo_gy_batch_set(&g_ism330.dev_ctx, (ism330dhcx_bdr_gy_t)gyr_odr_map[g_ism330.odr_index]);

        // 3. Update ODR
        return (ism330dhcx_gy_data_rate_set(&g_ism330.dev_ctx, gyr_odr_map[g_ism330.odr_index]) == 0);
    }
    return true;
}


// ============================================================================
// DATA PATH FUNCTIONS
// ============================================================================

// DATA PATH WITH RE-SYNC LOGIC
void ISM330_writeBuffer(uint8_t* data, uint16_t len)
{
    if (g_ism330.powered) 
    {
        // RE-SYNC: If buffer empty, grab fresh hardware time
        if (rb_available(&g_ism330.buffer) == 0) 
        {
            g_ism330.oldest_sample_timestamp = (double)getMicrosecondTimestamp();
        }
        rb_write(&g_ism330.buffer, data, len);
    }
}

uint16_t ISM330_getAvailableBytes(void)
{
    return rb_available(&g_ism330.buffer);
}

void ISM330_getStats(stream_metrics_t* stats_out)
{
    if (stats_out) *stats_out = g_ism330.stats;
}

void ISM330_resetStats(void) 
{
    memset(&g_ism330.stats, 0, sizeof(stream_metrics_t));
    g_ism330.buffer.head = 0;
    g_ism330.buffer.tail = 0;
}

// ============================================================================
// INTERRUPTS
// ============================================================================
static void ISM330_interruptCallback(uint_least8_t index)
{
    if (g_ism330.initialized && 
        g_ism330.powered && 
        g_ism330.drdy_sem) 
    {
        Semaphore_post(g_ism330.drdy_sem);
    }
}

static bool ism_configure_interrupts(bool enable)
{
    ism330dhcx_pin_int1_route_t int1_route;

    if (ism330dhcx_pin_int1_route_get(&g_ism330.dev_ctx, &int1_route) != 0) return false;

    // Route Gyro DRDY to INT1
    int1_route.int1_ctrl.int1_drdy_xl = 0;
    int1_route.int1_ctrl.int1_drdy_g = (enable ? 1 : 0);

    return (ism330dhcx_pin_int1_route_set(&g_ism330.dev_ctx, &int1_route) == 0);    
}


// ============================================================================
// WATERMARK LOGIC
// ============================================================================
/*
// Helper: Calculates optimal WTM based on BOTH sensors
static uint16_t get_optimal_watermark(void) 
{
    // Access global states
    uint8_t ism_idx = g_ism330.odr_index;
    uint8_t h3l_idx = g_h3l.odr_index; // Accessed via extern in h3l_driver.h

    // The Condition: High WTM only if Gyro >= 1666Hz AND Accel == 1000Hz
    if (ism_idx >= ISM_ODR_INDEX_1666Hz && h3l_idx == H3L_ODR_INDEX_1000Hz) 
    {
        return ISM_WTM_HIGH_WORDS;
    }
    
    return ISM_WTM_LOW_WORDS;
}

// PUBLIC: Called by ISM reconfigure AND H3L reconfigure
void ISM330_updateWatermarkFromState(void)
{
    if (!g_ism330.powered) return;

    uint16_t target_wtm = get_optimal_watermark();

    if (g_ism330.current_watermark != target_wtm) {
        g_ism330.current_watermark = target_wtm;
        
        // I2C Write
        ism330dhcx_fifo_watermark_set(&g_ism330.dev_ctx, g_ism330.current_watermark);

    }
}*/
// adc_driver.c - ADC Sensor Controller Driver
#include "adc_driver.h"
#include <ti/sysbios/hal/Hwi.h>
#include <ti/drivers/dpl/HwiP.h>
#include <string.h>
#include <stdio.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Task.h>
#include <icall.h>
#include <gap.h>
#include "scif.h"

// External init for SCIF
extern void SensorController_init(void);

// Global context
adc_driver_t g_adc = {0};

// PRIVATE HELPERS
static uint16_t frequency_to_scif_param(uint32_t freq_hz);
static uint16_t scif_param_to_frequency(uint32_t scif_param);

extern uint32_t getMicrosecondTimestamp(void);

// ============================================================================
// DIRECT DRIVER IMPLEMENTATION
// ============================================================================
bool ADC_init(void)
{
    if (g_adc.initialized) return true;

    // 1. Clear Context FIRST
    memset(&g_adc, 0, sizeof(adc_driver_t));

    // 2. Init Ring Buffer
    rb_init(&g_adc.buffer, 
            g_adc.storage, 
            sizeof(g_adc.storage), 
            &g_adc.stats);

    // 3. Init Hardware Driver
    SensorController_init();

    // 4. Default Defaults
    g_adc.matrix_size = 32;
    g_adc.frequency_hz = 0; 

    // 5. Setup SCIF Structs (Initial)
    scifTaskData.adcTrigger.cfg.matrixSize = g_adc.matrix_size;
    scifTaskData.adcTrigger.cfg.alertThr = g_adc.matrix_size;
    scifTaskData.adcTrigger.cfg.frequency = frequency_to_scif_param(1);

    g_adc.initialized = true;
    g_adc.powered = false; 

    printf("[ADC] Driver initialized! \n");
    
    return true;
}

bool ADC_startStreaming(void)
{
    if (!g_adc.initialized || g_adc.powered) return false;

    // Validate parameters
    if (g_adc.matrix_size < 1 || g_adc.matrix_size > 32) return false;

    // Reset Stats
    ADC_resetStats();

    // Reset SCIF Task State
    scifResetTaskStructs(BV(SCIF_ADC_TRIGGER_TASK_ID), 
                         (1 << SCIF_STRUCT_STATE) | (1 << SCIF_STRUCT_CFG));

    // Configure SCIF
    scifTaskData.adcTrigger.cfg.matrixSize = g_adc.matrix_size;
    scifTaskData.adcTrigger.cfg.alertThr = g_adc.matrix_size;
    scifTaskData.adcTrigger.cfg.frequency = frequency_to_scif_param(g_adc.frequency_hz);
    
    // Start SCIF
    if (scifStartTasksNbl(BV(SCIF_ADC_TRIGGER_TASK_ID)) == SCIF_SUCCESS) 
    {
        g_adc.powered = true;

        uint32_t frame_freq = scif_param_to_frequency(scifTaskData.adcTrigger.cfg.frequency);
        double frame_period = 1000000.0 / (double)frame_freq;
        g_adc.us_per_sample = frame_period / (double)g_adc.matrix_size; 

        return true;
    }
    return false;
}

bool ADC_stopStreaming(void)
{
    if (!g_adc.powered) return false;
    
    if (scifStopTasksNbl(BV(SCIF_ADC_TRIGGER_TASK_ID)) == SCIF_SUCCESS) 
    {
        g_adc.powered = false;
        return true;
    }
    return false;
}

bool ADC_isRunning(void) 
{
    return g_adc.powered;
}

void ADC_setFrequency(uint32_t freq_hz)
{
    if(!g_adc.initialized || freq_hz == g_adc.frequency_hz) return;

    g_adc.frequency_hz = freq_hz;
    g_adc.stats.reconfigurations++;
    
    if (g_adc.powered) {
        ADC_stopStreaming();
        Task_sleep(1); // Wait for SCIF to settle
        ADC_startStreaming();
    }
}

void ADC_setMatrixSize(uint16_t size)
{
    if (!g_adc.initialized || size < 1 || size > 32 || size == g_adc.matrix_size) return;

    g_adc.matrix_size = size;
    g_adc.stats.reconfigurations++;

    if (g_adc.powered) {
        ADC_stopStreaming();
        Task_sleep(1);
        ADC_startStreaming();
    }
}

// ============================================================================
// DATA PATH
// ============================================================================

// Call this from ADC_processTaskAlert (SCIF Callback)
void ADC_writeBuffer(uint8_t* data, uint16_t len)
{
    if (g_adc.powered) 
    {
        // RE-SYNC: If buffer is empty, capture exact hardware time as double
        if (rb_available(&g_adc.buffer) == 0) 
        {
            g_adc.oldest_sample_timestamp = (double)getMicrosecondTimestamp();
        }
        
        rb_write(&g_adc.buffer, data, len);
    }
}

uint16_t ADC_getAvailableBytes(void)
{
    return rb_available(&g_adc.buffer);
}

void ADC_getStats(stream_metrics_t* stats_out) 
{
    if (stats_out) *stats_out = g_adc.stats;
}

void ADC_resetStats(void) 
{
    memset(&g_adc.stats, 0, sizeof(stream_metrics_t));
    g_adc.buffer.head = 0;
    g_adc.buffer.tail = 0;
}

// ============================================================================
// HELPER
// ============================================================================
static uint16_t frequency_to_scif_param(uint32_t freq_hz)
{   
    // SCIF Prescaler logic
    if(freq_hz == 5) return 4;         
    else if (freq_hz == 4) return 8;   
    else if (freq_hz == 3) return 16;  
    else if (freq_hz == 2) return 40;   
    else if (freq_hz == 1) return 400;  
    else return 4000;                    
}

static uint16_t scif_param_to_frequency(uint32_t scif_param)
{
    if(scif_param == 4) return 1000;         
    else if (scif_param == 8) return 500;   
    else if (scif_param == 16) return 250;  
    else if (scif_param == 40) return 100;   
    else if (scif_param == 400) return 10;  
    else if (scif_param == 4000) return 1;   
    else return 1;  
}
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <stdio.h>
#include <icall.h>
#include "sensor_tasks.h"
#include "/drivers/ism330_driver.h"
#include "/drivers/h3l_driver.h"
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------
#define ISM_TASK_PRIORITY       4
#define ISM_TASK_STACK_SIZE     2048

#define H3L_TASK_PRIORITY       5
#define H3L_TASK_STACK_SIZE     2048

#define ISM_BYTES_PER_SAMPLE    7
#define ISM_CHUNK_SAMPLES       4
#define ISM_CHUNK_BYTES         (ISM_CHUNK_SAMPLES * ISM_BYTES_PER_SAMPLE)

// -----------------------------------------------------------------------------
// Globals & Resources
// -----------------------------------------------------------------------------
extern ICall_SyncHandle syncEvent;
#define SP_THROUGHPUT_EVT Event_Id_00

// Shared Semaphore
extern Semaphore_Handle h3lGlobalSem;

// Intermediate Buffer for Voluntary Yield
static uint8_t ism_rx_buf[ISM_CHUNK_BYTES]; 

// Task Objects & Stacks
static Task_Struct ismTask;
static uint8_t ismTaskStack[ISM_TASK_STACK_SIZE] __attribute__ ((aligned (8)));

static Task_Struct h3lTask;
static uint8_t h3lTaskStack[H3L_TASK_STACK_SIZE] __attribute__ ((aligned (8)));

// Forward Declarations
static void ISM_createTask(void);
static void H3L_createTask(void);

// -----------------------------------------------------------------------------
// Task Functions
// -----------------------------------------------------------------------------
/**
 * @brief   Gyroscope Acquisition Task (Medium Priority).
 *          Reads FIFO in chunks to allow preemption by Accelerometer.
 */
static void ISM330_taskFxn(UArg a0, UArg a1)
{
    uint16_t fifo_words = 0;
    const uint8_t EXPECTED_TAG = 0x01; 
    uint16_t accumulated_samples = 0;

    while (1) 
    {
        // 1. Wait for Watermark Interrupt
        Semaphore_pend(g_ism330.drdy_sem, BIOS_WAIT_FOREVER);
        
        // 2. Check FIFO Level
        ism330dhcx_fifo_data_level_get(&g_ism330.dev_ctx, &fifo_words);

        // 3. Voluntary Yield loop
        // (Drain FIFO in small chunks to allow H3L preemption)
        while (fifo_words >= (ISM_CHUNK_BYTES / 2)) 
        {
            // Critical Section: I2C Bus Lock (Implicit in Driver)
            if (ISM330_readFifoBatch_Block(ism_rx_buf, ISM_CHUNK_BYTES) != 0) break; 
            // End Critical Section

            // Process Data (Tag Validation & Commit)
            for (int i = 0; i < ISM_CHUNK_SAMPLES; i++) 
            {
                uint16_t offset = i * ISM_BYTES_PER_SAMPLE;
                uint8_t tag_id = (ism_rx_buf[offset] >> 3) & 0x1F; 
                
                if (tag_id == EXPECTED_TAG) 
                {
                    ISM330_writeBuffer(&ism_rx_buf[offset + 1], 6);
                }
            }

            fifo_words -= (ISM_CHUNK_BYTES / 2);

            // Notify BLE Logic (Batch ~20 samples)
            accumulated_samples += ISM_CHUNK_SAMPLES;
            if (accumulated_samples >= 20) 
            {
                if (syncEvent) Event_post(syncEvent, SP_THROUGHPUT_EVT);
                accumulated_samples = 0;
            }
            
            // Loop repeats -> allows context switch here
        }
    }
}

/**
 * @brief   Accelerometer Acquisition Task (High Priority).
 *          Handles hard real-time interrupts for legacy sensor.
 */
static void H3L_taskFxn(UArg a0, UArg a1) 
{
    int16_t acc_raw[3];
    uint8_t sensor_data[6];
    uint8_t samples_since_notify = 0;
    h3lis331dl_status_reg_t status;
    
    // Watchdog: If period is 1000us, wait ~1200us.
    // This catches missed interrupts before the data is overwritten.
    uint32_t watchdog_timeout = (g_h3l.timeout_ticks > 0) ? g_h3l.timeout_ticks : 120;

    while (1) 
    {      
        // 1. Wait for Interrupt OR Timeout
        bool got_interrupt = Semaphore_pend(h3lGlobalSem, watchdog_timeout);
        
        bool read_data = false;

        if (got_interrupt) 
        {
            // HAPPY PATH: Hardware interrupt fired. Data is ready.
            // Skip status check to save bus time.
            read_data = true;
        }
        else 
        {
            // RECOVERY PATH: Timeout (Interrupt likely missed).
            // Check status manually to ensure valid data.
            if (h3lis331dl_status_reg_get(&g_h3l.dev_ctx, &status) == 0) {
                if (status.zyxda) {
                    read_data = true;
                }
            }
        }

        // 2. Acquisition
        if (read_data) 
        {
            if (h3lis331dl_acceleration_raw_get(&g_h3l.dev_ctx, acc_raw) == 0) 
            {
                // Serialize
                sensor_data[0] = (uint8_t)(acc_raw[0] & 0xFF);
                sensor_data[1] = (uint8_t)((acc_raw[0] >> 8) & 0xFF);
                sensor_data[2] = (uint8_t)(acc_raw[1] & 0xFF);
                sensor_data[3] = (uint8_t)((acc_raw[1] >> 8) & 0xFF);
                sensor_data[4] = (uint8_t)(acc_raw[2] & 0xFF);
                sensor_data[5] = (uint8_t)((acc_raw[2] >> 8) & 0xFF);

                H3L_writeBuffer(sensor_data, 6);
                samples_since_notify++;

                // Notify BLE Engine every 10 Samples
                if (samples_since_notify >= 10) 
                {
                    if (syncEvent != NULL) Event_post(syncEvent, SP_THROUGHPUT_EVT);
                    samples_since_notify = 0;
                }        
             }
        }
    }
}

// ============================================================================
// PUBLIC API Implementation
// ============================================================================
bool SensorTasks_init(void) 
{    
    ISM_createTask();
    ISM330_stopStreaming();     // Guarantee initial state = stopped

    H3L_createTask();
    H3L_stopStreaming();        // Guarantee initial state = stopped

    printf("Sensor tasks created! \n");

    return true;
}

// ============================================================================
// TASK CREATION Helpers
// ============================================================================
static void ISM_createTask(void)
{
  Task_Params taskParams;
  Task_Params_init(&taskParams);
  taskParams.stack = ismTaskStack;
  taskParams.stackSize = ISM_TASK_STACK_SIZE;
  taskParams.priority = ISM_TASK_PRIORITY;
  Task_construct(&ismTask, ISM330_taskFxn, &taskParams, NULL);
}

static void H3L_createTask(void)
{
  Task_Params taskParams;
  Task_Params_init(&taskParams);
  taskParams.stack = h3lTaskStack;
  taskParams.stackSize = H3L_TASK_STACK_SIZE;
  taskParams.priority = H3L_TASK_PRIORITY;
  Task_construct(&h3lTask, H3L_taskFxn, &taskParams, NULL);
}


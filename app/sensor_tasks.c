#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/BIOS.h>
#include <stdio.h>
#include <icall.h>
#include "sensor_tasks.h"
#include "/drivers/ism330_driver.h"
#include "/drivers/h3l_driver.h"
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

// External Event Handle
extern ICall_SyncHandle syncEvent;
#define SP_THROUGHPUT_EVT Event_Id_00
extern Semaphore_Handle h3lGlobalSem;

// Task Configurations
#define ISM_TASK_PRIORITY       4
#define ISM_TASK_STACK_SIZE     2048

#define H3L_TASK_PRIORITY       5
#define H3L_TASK_STACK_SIZE     2048

#define ISM_BYTES_PER_SAMPLE    7

// TUNING: 2 samples = 14 bytes = ~0.35ms block time @ 400kHz
// This allows the H3L (High Priority) to interrupt the Gyro stream.
#define ISM_CHUNK_SAMPLES       4
#define ISM_CHUNK_BYTES         (ISM_CHUNK_SAMPLES * ISM_BYTES_PER_SAMPLE)

static uint8_t ism_rx_buf[ISM_CHUNK_BYTES]; 

// Task Objects & Stacks
static Task_Struct ismTask;
static uint8_t ismTaskStack[ISM_TASK_STACK_SIZE] __attribute__ ((aligned (8)));

static Task_Struct h3lTask;
static uint8_t h3lTaskStack[H3L_TASK_STACK_SIZE] __attribute__ ((aligned (8)));

// Forward Declarations
static void ISM330_taskFxn(UArg a0, UArg a1);
static void H3L_taskFxn(UArg a0, UArg a1);
static void ISM_createTask(void);
static void H3L_createTask(void);

// ============================================================================
// PUBLIC API
// ============================================================================

bool SensorTasks_init(void) 
{    
    ISM_createTask();
    ISM330_stopStreaming(); // Driver handles startup state

    H3L_createTask();
    H3L_stopStreaming();    // Driver handles startup state

    printf("Sensor tasks created! \n");

    return true;
}

// ============================================================================
// ISM TASK (GYRO)
// ============================================================================
static void ISM330_taskFxn(UArg a0, UArg a1)
{
    uint16_t fifo_words = 0;
    const uint8_t EXPECTED_TAG = 0x01; 
    uint16_t accumulated_samples = 0;

    while (1) {
        // Wait for IRQ (Watermark reached)
        Semaphore_pend(g_ism330.drdy_sem, BIOS_WAIT_FOREVER);
        
        // Check FIFO Level
        ism330dhcx_fifo_data_level_get(&g_ism330.dev_ctx, &fifo_words);

        // Drain FIFO in small chunks to allow H3L preemption
        while (fifo_words >= (ISM_CHUNK_BYTES / 2)) 
        {
            // --- ATOMIC SECTION START ---
            if (ISM330_readFifoBatch_Block(ism_rx_buf, ISM_CHUNK_BYTES) != 0) {
                break; 
            }
            // --- ATOMIC SECTION END ---

            // Process data locally
            for (int i = 0; i < ISM_CHUNK_SAMPLES; i++) {
                uint16_t offset = i * ISM_BYTES_PER_SAMPLE;
                uint8_t tag_id = (ism_rx_buf[offset] >> 3) & 0x1F; 
                
                if (tag_id == EXPECTED_TAG) {
                    ISM330_writeBuffer(&ism_rx_buf[offset + 1], 6);
                }
            }

            fifo_words -= (ISM_CHUNK_BYTES / 2);

            // Notify BLE Logic (Batch ~20 samples)
            accumulated_samples += ISM_CHUNK_SAMPLES;
            if (accumulated_samples >= 32) {
                if (syncEvent) Event_post(syncEvent, SP_THROUGHPUT_EVT);
                accumulated_samples = 0;
            }
        }
    }
}

// ============================================================================
// H3L TASK (ACCEL)
// ============================================================================
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

        // 2. Read
        if (read_data) 
        {
            if (h3lis331dl_acceleration_raw_get(&g_h3l.dev_ctx, acc_raw) == 0) 
            {
                sensor_data[0] = (uint8_t)(acc_raw[0] & 0xFF);
                sensor_data[1] = (uint8_t)((acc_raw[0] >> 8) & 0xFF);
                sensor_data[2] = (uint8_t)(acc_raw[1] & 0xFF);
                sensor_data[3] = (uint8_t)((acc_raw[1] >> 8) & 0xFF);
                sensor_data[4] = (uint8_t)(acc_raw[2] & 0xFF);
                sensor_data[5] = (uint8_t)((acc_raw[2] >> 8) & 0xFF);

                H3L_writeBuffer(sensor_data, 6);
                samples_since_notify++;

                // Notify BLE every 10 samples (NOT 1!)
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
// TASK CREATION HELPERS
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
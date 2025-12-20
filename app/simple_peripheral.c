/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13xx_cc26xx

 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdio.h>
#include <gap.h>
#include "/scif/scif.h"

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <xdc/runtime/System.h>

#include <ti/display/Display.h>

#if (!(defined __TI_COMPILER_VERSION__) && !(defined __GNUC__))
#include <intrinsics.h>
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>

/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include <devinfoservice.h>
#include <simple_gatt_profile.h>

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC


#include "simple_peripheral.h"
#include "ti_ble_config.h"
#include <ti/sysbios/hal/Hwi.h>
#include <ti/drivers/dpl/HwiP.h>

#include <ti/drivers/I2C.h>
#include <ti/drivers/Board.h>
#include <ti_drivers_config.h>

/*********************************************************************
 * REFACTOR INCLUDES
 */
//#include "/core/sensor_manager.h"
#include "sensor_tasks.h"
#include "/drivers/ism330_driver.h"
#include "/drivers/h3l_driver.h"
#include "/drivers/adc_driver.h"
#include "data_stream_engine.h"

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event (in ms)
#define SP_PERIODIC_EVT_PERIOD               1000

// Main Task configuration
#define SP_TASK_PRIORITY                    3

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   2048
#endif

// Application events
#define SP_STATE_CHANGE_EVT                  0 /* A characteristic value has been written     */
#define SP_CHAR_CHANGE_EVT                   1 /* A characteristic configuration has changed  */
#define SP_ADV_EVT                           2 /* A subscribed advertisement activity         */
#define SP_PAIR_STATE_EVT                    3 /* The pairing state is updated                */
#define SP_PASSCODE_EVT                      4 /* A pass-code/PIN is requested during pairing */
#define SP_PERIODIC_EVT                      5 /* Periodic event                              */
#define SP_READ_RPA_EVT                      6 /* Read RPA event                              */
#define SP_SEND_PARAM_UPDATE_EVT             7 /* Request parameter update req be sent        */
#define SP_CONN_EVT                          8 /* Connection Event End notice                 */
#define SP_SC_CTRL_READY                     9 /* Sensor controller control ready             */
#define SP_SC_TASK_ALERT                    10 /* Sensor controller task alert                */
#define SP_TIMEOUT_EVT                      11

#define SCIF_BUFFER_SIZE 1280

// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SP_THROUGHPUT_EVT                    Event_Id_00
#define SP_PDU_CHANGE_EVT                    Event_Id_02
#define SP_PHY_CHANGE_EVT                    Event_Id_03

// Bitwise OR of all RTOS events to pend on
// Bitwise OR of all RTOS events to pend on
#define SP_ALL_EVENTS                        (SP_ICALL_EVT             | \
                                              SP_QUEUE_EVT             | \
                                              SP_THROUGHPUT_EVT        | \
                                              SP_PDU_CHANGE_EVT        | \
                                              SP_PHY_CHANGE_EVT           )

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// Default values for connection PHY and PDU sizec:\Users\rober\OneDrive\Documentos\University\Mestrado\2 ano\Dissertação\BP + DC INTEGRATED 25-11-25\simple_peripheral_3.0_BP+DC_integrated.c
#define DEFAULT_PHY         TS_PHY_2M
#define DEFAULT_PDU         27

#define MY_DESIRED_PDU      251
#define MY_DESIRED_PHY_IDX  1  // 0=1M, 1=2M, 2=CODED S2, 3=CODED S8

// Default TX-time values used for DLE
#define DEFAULT_TX_TIME     328
#define DEFAULT_MAX_TX_TIME 2120
#define CODED_MAX_TX_TIME   17040

// How often to read current current RPA (in ms)
#define TP_READ_RPA_EVT_PERIOD               3000

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30           
#define RSSI_1M_THRSHLD           -40           
#define RSSI_S2_THRSHLD           -50           
#define RSSI_S8_THRSHLD           -60           
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// The different peerTypes used for the connection info
#define peerType_NOTIDENTIFIED       0x00
#define peerType_LAUNCHPAD           0x01
#define peerType_IPHONE              0x02
#define peerType_ANDROID             0x04
#define peerType_CODEDPHY            0x08

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

/*********************************************************************
 * CONSTANTS AND CONFIGURATIONS
 */

#define TGAP_CONN_EST_INT_MIN   21
#define TGAP_CONN_EST_INT_MAX   22

// ==========================
// Global Buffers and Indexes
// ==========================
uint16_t connHandle = LINKDB_CONNHANDLE_INVALID;

#define GAP_CONNHANDLE_INVALID  0xFFFF

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} spPairStateData_t; 

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} spPasscodeData_t; 

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct{
  uint8_t event;                
  uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct{
  List_Elem elem;
  uint16_t  connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct{
    uint16_t            connHandle;          // Connection Handle

    // Connection parameter update
    spClockEventData_t* pParamUpdateEventData;
    Clock_Struct*       pUpdateClock;        // pointer to clock struct
    uint16_t            connInterval;        // Connection interval (1.25ms units)
    uint16_t            connLatency;         // Slave latency
    uint16_t            connTimeout;         // Supervision timeout (10ms units)

    // RSSI tracking
    int8_t              rssiArr[SP_MAX_RSSI_STORE_DEPTH];
    uint8_t             rssiCntr;
    int8_t              rssiAvg;

    // PHY / DLE / MTU state
    bool                phyCngRq;            // PHY change request in progress
    uint8_t             currPhy;             // Cached "effective" PHY (legacy field, can map to txPhy if needed)
    uint8_t             rqPhy;               // Last requested PHY
    uint8_t             txPhy;               // Actual TX PHY (from event)
    uint8_t             rxPhy;               // Actual RX PHY (from event)
    uint8_t             phyRqFailCnt;        // PHY change request fail count
    bool                isAutoPHYEnable;     // Auto PHY change enabled
    uint8_t             oldPHY;              // Previous PHY-setting
    bool                phyUpdated;          // PHY confirmed


    uint8_t             oldPDU;              // Previous PDU length
    uint16_t            maxTxOctets;         // Current max PDU size (bytes)
    uint16_t            maxTxTime;           // Current max TX time (us)
    bool                dleUpdated;          // Data Length Extension confirmed

    uint16_t            mtuSize;             // Current negotiated MTU
    bool                mtuUpdated;          // MTU confirmed

    // Misc / throughput
    bool                establishingConn;    // Connection in progress
    bool                initialParamsSet;    // Whether the initial connection parameters have been set
    uint32              oldMsgCounter;       // Previous MsgCounter-value
    bool                restartThroughput;   // Restart throughput demo after param change
    uint8_t             peerType;            // Type/features of peer device

} spConnRec_t;

typedef struct {
    bool ismActive;
    bool h3lActive;
    bool adcActive;
    uint16_t connHandle;
} throughputInfo_t;

static throughputInfo_t tpInfo = {0};

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

// ==========================
// Main Task Configuration
// ==========================
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SP_TASK_STACK_SIZE];

#define APP_EVT_EVENT_MAX 0x9
char *appEventStrings[] = {
  "APP_STATE_CHANGE_EVT     ",
  "APP_CHAR_CHANGE_EVT      ",
  "APP_ADV_EVT              ",
  "APP_PAIR_STATE_EVT       ",
  "APP_PASSCODE_EVT         ",
  "APP_READ_RPA_EVT         ",
  "APP_PERIODIC_EVT         ",
  "APP_SEND_PARAM_UPDATE_EVT",
  "APP_CONN_EVT             ",
};

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;
// Clock instance for connection timeout
static Clock_Struct clkTimeout;

// Memory to pass Timeout event ID
spClockEventData_t argTimeout = { .event = SP_TIMEOUT_EVT };

// Memory to pass periodic event ID to clock handler
spClockEventData_t argPeriodic =
{ .event = SP_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead =
{ .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// Current connection handle as chosen by menu
static uint16_t menuConnHandle = LINKDB_CONNHANDLE_INVALID;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Advertising handles
static uint8 advHandleLegacy;
static uint8 advHandleLongRange;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};

// Connection info
static spConnRec_t connInfo;

// MTU of the current connection
static uint16_t currentMTU = MAX_PDU_SIZE;

// Message counter for Throughput Demo
static uint32 msg_counter = 0;

// PHY Options
static uint16_t phyOptions = LL_PHY_OPT_NONE;

/* -------------------------- Types & Globals -------------------------- */
extern platform_i2c_bus_t gIsmBus, gH3lBus;

extern ism330_driver_t g_ism330;  

extern h3l_driver_t g_h3l;

extern adc_driver_t g_adc;

static uint32_t last_telemetry_update = 0;

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);
static void SimplePeripheral_advCB(uint32_t event, void *pBuf, uintptr_t arg); 
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);    // REMOVE? 
static void SimplePeripheral_charValueChangeCB(uint8_t paramId);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId);
static void SimplePeripheral_performPeriodicTask(void);
static void SimplePeripheral_updateRPA(void);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle, // REMOVE?
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);                
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,    // REMOVE?
                                         uint8_t status);
static void SimplePeripheral_processPairState(spPairStateData_t *pPairState);  // REMOVE?
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData); // REMOVE?
status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData);
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static void SimplePeripheral_processParamUpdate(uint16_t connHandle);
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle); // REMOVE? 
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle); // REMOVE?
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport);
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport);

/*********************************************************************
 * SENSOR CONTROLLER & ADC FUNCTIONS
 */
void SensorController_init(void);
static void scCtrlReadyCallback(void);
static void scTaskAlertCallback(void);
static void ADC_processTaskAlert(void);
static void BLE_configureHighThroughput(uint16_t connHandle);

/******************************************************************************
 * HIGH-RESOLUTION TIMING
 */
// Use the CPU cycle counter for microsecond timing
uint32_t getMicrosecondTimestamp(void) {
    // For CC2652RB - use the CPU cycle counter
    // This gives ~48MHz clock, so each tick is ~20.83ns
    //return (uint32_t)(Clock_getTicks() * 20) / 1000; // Convert to microseconds

    // TI-RTOS Clock Module typically configured for 10us ticks in BLE projects.    
    // Convert Ticks -> Microseconds
    return (uint32_t)Clock_getTicks() * 10;
}


/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * EXTERN VARIABLES
 */
extern gattCharCfg_t *simpleProfileChar4Config;
extern gattCharCfg_t *simpleProfileChar6Config;

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs =
{
  SimplePeripheral_passcodeCb,       // Passcode callback
  SimplePeripheral_pairStateCb       // Pairing/Bonding state Callback
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs =
{
  SimplePeripheral_charValueChangeCB // Simple GATT Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = spTaskStack;
  taskParams.stackSize = SP_TASK_STACK_SIZE;
  taskParams.priority = SP_TASK_PRIORITY;

  Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SimplePeripheral_init(void)
{
  Event_Params evtParams;
  Event_Params_init(&evtParams);
  syncEvent = Event_create(&evtParams, NULL);   // or Event_create(NULL, NULL)
  if (syncEvent == NULL) {
      /* handle error: Event_create failed */
  }

  memset(&connInfo, 0, sizeof(connInfo));
  connInfo.connHandle = LINKDB_CONNHANDLE_INVALID;
  connInfo.establishingConn = false;

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", SP_TASK_PRIORITY);
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, SimplePeripheral_clockHandler,
                      SP_PERIODIC_EVT_PERIOD, 0, false, (UArg)&argPeriodic);

  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  setBondManagerParameters();

  // Initialize GATT attributes
  GGS_AddService(GAP_SERVICE);                 // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue = 2;

    SimpleProfile_SetParameter(ADC_START_THROUGHPUT_CHAR, sizeof(uint8_t),
                               &charValue);
    SimpleProfile_SetParameter(ADC_FREQUENCY_CHAR, sizeof(uint8_t),
                               &charValue);
    SimpleProfile_SetParameter(ADC_SENSORS_CHAR, sizeof(uint8_t),
                               &charValue);
    SimpleProfile_SetParameter(ADC_NOTIFY_CHAR, sizeof(uint8_t),
                               &charValue); 

    SimpleProfile_SetParameter(GYR_START_THROUGHPUT_CHAR, sizeof(uint8_t),
                               &charValue);
    SimpleProfile_SetParameter(GYR_FREQUENCY_CHAR, sizeof(uint8_t),
                              &charValue);   
    SimpleProfile_SetParameter(GYR_NOTIFY_CHAR, sizeof(uint8_t),
                              &charValue); 

    SimpleProfile_SetParameter(ACC_START_THROUGHPUT_CHAR, sizeof(uint8_t),
                               &charValue);
    SimpleProfile_SetParameter(ACC_FREQUENCY_CHAR, sizeof(uint8_t),
                              &charValue);
    SimpleProfile_SetParameter(ACC_NOTIFY_CHAR, sizeof(uint8_t),
                              &charValue);                
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI events.
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME); // Max values for DLE 

  }

  // Initialize GATT Client
  GATT_InitClient();

  // By Default Allow Central to support any and all PHYs
  HCI_LE_SetDefaultPhyCmd(LL_PHY_USE_PHY_PARAM, LL_PHY_2_MBPS, LL_PHY_2_MBPS);

  // Set the Transmit Power of the Device to +5dBm
  HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);

  //Set the RX Gain to be highest
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);

  // Initialize Connection List
  SimplePeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL);

  //Initialize GAP layer for Peripheral role and register to receive GAP events
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress);

  dispHandle = Display_open(Display_Type_ANY, NULL);

  printf("BLE Stack initialized! \n");

  // Initialize array to store connection handle and RSSI values
  SimplePeripheral_initPHYRSSIArray();

}

/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{

  // Initialize sensor system
  SensorsPlatform_init();

  SimplePeripheral_init();

  ADC_init();
  
  ISM330_init(&gIsmBus);

  H3L_init(&gH3lBus);

  SensorTasks_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;
    
    events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    // Update telemetry every 5 seconds
    uint32_t now = Clock_getTicks();
    if (now - last_telemetry_update > 5000) 
    {
        DataStream_updateTelemetry(connHandle);
        last_telemetry_update = now;
    }

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
          else
          {
            //Get connection handle
            connHandle = connInfo.connHandle;
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SP_QUEUE_EVT)
      {
        //printf("=== PROCESSING APP MESSAGES ===\n");
        while (!Queue_empty(appMsgQueueHandle))
        {
          spEvt_t *pMsg = (spEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            SimplePeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }

      if (events & SP_THROUGHPUT_EVT)
      {
        DataStream_processEvents(connHandle); 
      }

      if (events & SP_PDU_CHANGE_EVT)
      {
          uint16_t newPDU = MY_DESIRED_PDU;
          uint16_t txTime = DEFAULT_TX_TIME;

          HCI_LE_SetDataLenCmd(connHandle, newPDU, txTime);
      }

      if (events & SP_PHY_CHANGE_EVT)
      {
        HCI_LE_SetPhyCmd(connHandle,
                        LL_PHY_USE_PHY_PARAM,
                        HCI_PHY_2_MBPS,
                        HCI_PHY_2_MBPS,
                        LL_PHY_OPT_NONE);

        connInfo.rqPhy = HCI_PHY_2_MBPS;
      }
    }

  }

}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:

      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);

      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          SimplePeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
          switch ( pMyMsg->cmdOpcode )
          {
            case HCI_LE_SET_PHY:
            {
              if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
              {
              }
              break;
            }

            default:
              break;
          }
          break;
        }

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;
          bool extendingTxTime = false;

          // ===== PHY Update Complete =====
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
              if (pPUC->status == SUCCESS &&
                  pPUC->rxPhy == connInfo.rqPhy)
              {
                  connInfo.phyUpdated = true;
                  connInfo.txPhy = pPUC->txPhy;
                  connInfo.rxPhy = pPUC->rxPhy;

                  // First time using CODED PHY → extend TX time
                  if ((pPUC->rxPhy == HCI_PHY_CODED) &&
                      !(connInfo.peerType & peerType_CODEDPHY))
                  {
                      connInfo.peerType |= peerType_CODEDPHY;
                      Event_post(syncEvent, SP_PDU_CHANGE_EVT);
                      extendingTxTime = true;
                  }
              }

              //SimplePeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }

          // ===== Data Length Change =====
          if (pPUC->BLEEventCode == HCI_BLE_DATA_LENGTH_CHANGE_EVENT)
          {
              hciEvt_BLEDataLengthChange_t *dleEvt = (hciEvt_BLEDataLengthChange_t*) pMsg;
              if (dleEvt->maxTxOctets >= 251) // Full-length PDU confirmed
              {
                  connInfo.maxTxOctets = dleEvt->maxTxOctets;
                  connInfo.maxTxTime   = dleEvt->maxTxTime;
                  connInfo.dleUpdated = true;

              }
              connInfo.oldPDU = dleEvt->maxRxOctets;
          }
          break;
        }

        default:
          break;
      }

      break;
    }
    default:
      // do nothing
      break;
  }
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {

  }
  else if (pMsg->method == ATT_HANDLE_VALUE_CFM)
  {

  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
      // Store updated MTU size
      currentMTU = pMsg->msg.mtuEvt.MTU;
      connInfo.mtuSize = currentMTU;
      connInfo.mtuUpdated = true;

  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg)
{
  bool dealloc = TRUE;

  if (pMsg->event <= APP_EVT_EVENT_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
  }

  switch (pMsg->event)
  {
    case SP_STATE_CHANGE_EVT: 
      break;

    case SP_CHAR_CHANGE_EVT:
      SimplePeripheral_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case SP_ADV_EVT:
      SimplePeripheral_processAdvEvent((spGapAdvEventData_t*)(pMsg->pData));
      break;

    case SP_PAIR_STATE_EVT:
      SimplePeripheral_processPairState((spPairStateData_t*)(pMsg->pData));
      break;

    case SP_PASSCODE_EVT:
      SimplePeripheral_processPasscode((spPasscodeData_t*)(pMsg->pData));
      break;

    case SP_PERIODIC_EVT:
      SimplePeripheral_performPeriodicTask();
      break;

    case SP_READ_RPA_EVT:
      SimplePeripheral_updateRPA();
      break;

    case SP_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((spClockEventData_t *)pMsg->pData)->data);

      SimplePeripheral_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }

    case SP_TIMEOUT_EVT:
    {
      // Check if the message counter is the same value as last time the clock ran out
      if (msg_counter == connInfo.oldMsgCounter)
      {
      }
      else
      {
          // Restart clock
          connInfo.oldMsgCounter = msg_counter;
          Util_restartClock(&clkTimeout, connInfo.connTimeout * 10);
      }
      break;

    }

    case SP_CONN_EVT:
    {
        // Process the event as before
        SimplePeripheral_processConnEvt((Gap_ConnEventRpt_t *)(pMsg->pData));

        // Get connection handle
        connHandle = ((Gap_ConnEventRpt_t *)pMsg->pData)->handle;

        // Reset flags for throughput setup
        connInfo.phyUpdated = false;
        connInfo.dleUpdated = false;
        connInfo.mtuUpdated = false;

        // Configure link for maximum throughput
        BLE_configureHighThroughput(connHandle);
        break;
    }

    case SP_SC_TASK_ALERT:
        ADC_processTaskAlert();
        break;

    default:
        // Do nothing.
        break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        
        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 0, 0);

        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCB, &advParams1,
                               &advHandleLegacy);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData1), advData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanResData1), scanResData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 1, 0);
        

        // Create Advertisement set #2 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCB, &advParams2,
                               &advHandleLongRange);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #2 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLongRange, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData2), advData2);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #2
        status = GapAdv_setEventMask(advHandleLongRange,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GapAdv_enable", 0);
        // Enable long range advertising for set #2
        status = GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        if (addrMode > ADDRMODE_RANDOM)
        {
          SimplePeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
                              READ_RPA_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      if (pPkt->hdr.status == SUCCESS)
      {   
        connHandle = pPkt->connectionHandle;
        connInfo.connTimeout = pPkt->connTimeout;

        // Add connection to list and start RSSI
        SimplePeripheral_addConn(pPkt->connectionHandle);

        BLE_configureHighThroughput(connHandle);
        Util_startClock(&clkPeriodic);  // Start Periodic Clock.~
      };

      // Stop advertising since there is no room for more connections
      GapAdv_disable(advHandleLongRange);
      GapAdv_disable(advHandleLegacy);

      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;
      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();

      // Remove the connection from the list and disable RSSI if needed
      SimplePeripheral_removeConn(pPkt->connectionHandle);

      connHandle = LINKDB_CONNHANDLE_INVALID;

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);
      }
      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      GapAdv_enable(advHandleLongRange, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with peripheral latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      if(pPkt->status == SUCCESS)
      {

      }
      else
      {

      }

      // Check if there are any queued parameter updates
      spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        SimplePeripheral_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
    {
      linkDBInfo_t linkInfo;
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      break;
    }
#endif

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (SimplePeripheral_enqueueMsg(SP_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }

}

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newValue;
  
  switch(paramId)
  {
    case ADC_START_THROUGHPUT_CHAR:
        SimpleProfile_GetParameter(ADC_START_THROUGHPUT_CHAR, &newValue);

        if (newValue == 1) ADC_startStreaming();

        else if( newValue == 0) ADC_stopStreaming(); 

        break;

    case ADC_FREQUENCY_CHAR:
        SimpleProfile_GetParameter(ADC_FREQUENCY_CHAR, &newValue);

        ADC_setFrequency(newValue);
        
        //SystemMonitor_ADCrequestConfig(frequency_hz, g_adc_config_manager.active_config.matrix_size);*/

        break;

    case ADC_SENSORS_CHAR:
        SimpleProfile_GetParameter(ADC_SENSORS_CHAR, &newValue);

        ADC_setMatrixSize(newValue);

        //SystemMonitor_ADCrequestConfig(g_adc_config_manager.active_config.frequency_hz, newValue);

        break;

    case GYR_START_THROUGHPUT_CHAR:
        SimpleProfile_GetParameter(GYR_START_THROUGHPUT_CHAR, &newValue);

        if (newValue == 1) ISM330_startStreaming(); 

        else if( newValue == 0) ISM330_stopStreaming(); 

        break;

    case GYR_FREQUENCY_CHAR:
        SimpleProfile_GetParameter(GYR_FREQUENCY_CHAR, &newValue);

        ISM330_reconfigureOdr(newValue);

        break;

    case ACC_START_THROUGHPUT_CHAR:
        SimpleProfile_GetParameter(ACC_START_THROUGHPUT_CHAR, &newValue);

        if (newValue == 1) H3L_startStreaming();
        

        else if(newValue == 0) H3L_stopStreaming(); 

        break;

    case ACC_FREQUENCY_CHAR:  
        SimpleProfile_GetParameter(ACC_FREQUENCY_CHAR, &newValue);

        H3L_reconfigureOdr(newValue);

        break;

    default:
        // should not reach here!
        break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *c:\Users\rober\OneDrive\Documentos\University\Mestrado\2 ano\Dissertação\Versões\Projeto 21-10-25\simple_peripheral.c
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_performPeriodicTask(void)
{

}
/*********************************************************************
 * @fn      SimplePeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg)
{
  spClockEventData_t *pData = (spClockEventData_t *)arg;

 if (pData->event == SP_PERIODIC_EVT)
 {
   // Start the next period
   Util_startClock(&clkPeriodic);

   // Post event to wake up the application
   SimplePeripheral_enqueueMsg(SP_PERIODIC_EVT, NULL);

 }
 else if (pData->event == SP_READ_RPA_EVT)
 {
   // Start the next period
   Util_startClock(&clkRpaRead);

   // Post event to read the current RPA
   SimplePeripheral_enqueueMsg(SP_READ_RPA_EVT, NULL);
 }
 else if (pData->event == SP_SEND_PARAM_UPDATE_EVT)
 {
    // Send message to app
    SimplePeripheral_enqueueMsg(SP_SEND_PARAM_UPDATE_EVT, pData);
 }
}


/*********************************************************************
 * @fn      SimplePeripheral_advCB
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCB(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
#ifndef Display_DISABLE_ALL
      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);

#endif
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(SimplePeripheral_enqueueMsg(SP_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(SimplePeripheral_enqueueMsg(SP_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(spPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
      }
      else
      {
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
      }
      else
      {
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
      }
      else
      {
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      SimplePeripheral_connEvtCB
 *
 * @brief   Connection event callback.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_connEvtCB(Gap_ConnEventRpt_t *pReport)
{
  // Enqueue the event for processing in the app context.
  if(SimplePeripheral_enqueueMsg(SP_CONN_EVT, pReport) != SUCCESS)
  {
    ICall_free(pReport);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processConnEvt
 *
 * @brief   Process connection event.
 *
 * @param pReport pointer to connection event report
 */
static void SimplePeripheral_processConnEvt(Gap_ConnEventRpt_t *pReport)
{
  // Get index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(pReport->handle);

  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return;
  }

  // If auto phy change is enabled
  if (connList[connIndex].isAutoPHYEnable == TRUE)
  {
    // Read the RSSI
    HCI_ReadRssiCmd(pReport->handle);
  }

  // Optional: store connection info for later
  connInfo.connHandle = pReport->handle;
  connInfo.establishingConn = TRUE;  // signal that connection is being set up
}



/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}

/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
      // Allocate data to send through clock handler
      connList[i].pParamUpdateEventData = ICall_malloc(sizeof(spClockEventData_t) +
                                                       sizeof (uint16_t));
      if(connList[i].pParamUpdateEventData)
      {
        connList[i].pParamUpdateEventData->event = SP_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)connList[i].pParamUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                              SimplePeripheral_clockHandler,
                              SEND_PARAM_UPDATE_DELAY, 0, true,
                              (UArg) (connList[i].pParamUpdateEventData));
        }
        else
        {
            ICall_free(connList[i].pParamUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }
#endif

      // Set default PHY to 2M
      connList[i].currPhy = HCI_PHY_2_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = SimplePeripheral_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
      memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void SimplePeripheral_clearPendingParamUpdate(uint16_t connHandle)
{
  List_Elem *curr;

  for (curr = List_head(&paramUpdateList); curr != NULL; curr = List_next(curr)) 
  {
    if (((spConnHandleEntry_t *)curr)->connHandle == connHandle)
    {
      List_remove(&paramUpdateList, curr);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
      // Free ParamUpdateEventData
      ICall_free(connList[connIndex].pParamUpdateEventData);
    }
    // Clear pending update requests from paramUpdateList
    SimplePeripheral_clearPendingParamUpdate(connHandle);
    // Stop Auto PHY Change
    SimplePeripheral_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    SimplePeripheral_clearConnListEntry(connHandle);
  }
  return connIndex;
}

/*********************************************************************
 * @fn      SimplePeripheral_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void SimplePeripheral_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
  req.connLatency = DEFAULT_DESIRED_PERIPHERAL_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
  req.intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
#endif

  connIndex = SimplePeripheral_getConnIndex(connHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    return;
  }

  if (connList[connIndex].pUpdateClock != NULL)
  {
    // Stop the clock if it's still alive
    if (Util_isActive(connList[connIndex].pUpdateClock))
    {
      Util_stopClock(connList[connIndex].pUpdateClock);
    }

    // Deconstruct the clock object
    Clock_destruct(connList[connIndex].pUpdateClock);

    // Free clock struct, only in case it is not NULL
    ICall_free(connList[connIndex].pUpdateClock);
    connList[connIndex].pUpdateClock = NULL;
  }

  // Free ParamUpdateEventData, only in case it is not NULL
  if (connList[connIndex].pParamUpdateEventData != NULL)
      ICall_free(connList[connIndex].pParamUpdateEventData);

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)connHandleEntry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      int8 rssi = (int8)pMsg->pReturnParam[3];

      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = SimplePeripheral_getConnIndex(handle);
        if (index >= MAX_NUM_BLE_CONNS)
        {
          return;
        }

        if (rssi != LL_RSSI_NOT_AVAILABLE)
        {
          connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
          connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

          int16_t sum_rssi = 0;
          for(uint8_t cnt=0; cnt<SP_MAX_RSSI_STORE_DEPTH; cnt++)
          {
            sum_rssi += connList[index].rssiArr[cnt];
          }
          connList[index].rssiAvg = (uint32_t)(sum_rssi/SP_MAX_RSSI_STORE_DEPTH);

          uint8_t phyRq = SP_PHY_NONE;
          uint8_t phyRqS = SP_PHY_NONE;
          uint8_t phyOpt = LL_PHY_OPT_NONE;

          if(connList[index].phyCngRq == FALSE)
          {
            if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
            (connList[index].currPhy != HCI_PHY_2_MBPS) &&
                 (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to higher data rate
              phyRqS = phyRq = HCI_PHY_2_MBPS;
            }
            else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                    (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to legacy regular data rate
              phyRqS = phyRq = HCI_PHY_1_MBPS;
            }
            else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                    (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to lower data rate S=2(500kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S2;
              phyRq = BLE5_CODED_S2_PHY;
            }
            else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
            {
              // try to go to lowest data rate S=8(125kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S8;
              phyRq = BLE5_CODED_S8_PHY;
            }
            if((phyRq != SP_PHY_NONE) &&
               // First check if the request for this phy change is already not honored then don't request for change
               (((connList[index].rqPhy == phyRq) &&
                 (connList[index].phyRqFailCnt < 2)) ||
                 (connList[index].rqPhy != phyRq)))
            {
              //Initiate PHY change based on RSSI
              SimplePeripheral_setPhy(connList[index].connHandle, 0,
                                      phyRqS, phyRqS, phyOpt);
              connList[index].phyCngRq = TRUE;

              // If it a request for different phy than failed request, reset the count
              if(connList[index].rqPhy != phyRq)
              {
                // then reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
              }

              if(phyOpt == LL_PHY_OPT_NONE)
              {
                connList[index].rqPhy = phyRq;
              }
              else if(phyOpt == LL_PHY_OPT_S2)
              {
                connList[index].rqPhy = BLE5_CODED_S2_PHY;
              }
              else
              {
                connList[index].rqPhy = BLE5_CODED_S8_PHY;
              }

            } // end of if ((phyRq != SP_PHY_NONE) && ...
          } // end of if (connList[index].phyCngRq == FALSE)
        } // end of if (rssi != LL_RSSI_NOT_AVAILABLE)

	  } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {

      }
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      SimplePeripheral_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void SimplePeripheral_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = SP_INVALID_HANDLE;
  }
}
/*********************************************************************
      // Set default PHY to 1M
 * @fn      SimplePeripheral_startAutoPhyChange
 *
 * @brief   Start periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 * @param   devAddr - device address
 *
 * @return  SUCCESS: Terminate started
 *          bleIncorrectMode: No link
 *          bleNoResources: No resources
 */
static status_t SimplePeripheral_startAutoPhyChange(uint16_t connHandle)
{
  status_t status = FAILURE;

  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Start Connection Event notice for RSSI calculation
  status = Gap_RegisterConnEventCb(SimplePeripheral_connEvtCB, GAP_CB_REGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Flag in connection info if successful
  if (status == SUCCESS)
  {
    connList[connIndex].isAutoPHYEnable = TRUE;
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, GAP_CB_CONN_EVENT_ALL, connHandle);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }

  return SUCCESS;
}

/*********************************************************************
* @fn      SimplePeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      spConnHandleEntry_t *connHandleEntry =
                           (spConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
{
    hciEvt_BLEPhyUpdateComplete_t *pPUC = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

    if (pPUC)
    {
        int index = SimplePeripheral_getConnIndex(pPUC->connHandle);
        if (index < MAX_NUM_BLE_CONNS)
        {
            connList[index].phyCngRq = FALSE; // no longer pending

            if (pPUC->status == SUCCESS)
            {
                connList[index].txPhy = pPUC->txPhy;
                connList[index].rxPhy = pPUC->rxPhy;
                connList[index].phyUpdated = true;
            }

            // track request failures
            if (pPUC->rxPhy != connList[index].rqPhy)
            {
                connList[index].phyRqFailCnt++;
            }
            else
            {
                connList[index].phyRqFailCnt = 0;
                connList[index].rqPhy = 0;
            }
        }
    }
    break;
}

    default:
      break;
  } // end of switch (eventCode)
}

/*********************************************************************
****************************** MY CODE *******************************
*********************************************************************/

/* -------------------------- BLE integration  -------------------------- */
/*********************************************************************/

static void BLE_configureHighThroughput(uint16_t connHandle)
{ 
    // Request PHY = 2M
    HCI_LE_SetPhyCmd(connHandle,
                     HCI_PHY_USE_PHY_PARAM,
                     HCI_PHY_2_MBPS,
                     HCI_PHY_2_MBPS,
                     0);

    connInfo.rqPhy = HCI_PHY_2_MBPS;

    // Request Data Length Extension (251 bytes PDU, ~2ms TX time)
    HCI_LE_SetDataLenCmd(connHandle, 251, 2120);

    // Exchange MTU (251)
    attExchangeMTUReq_t req;
    req.clientRxMTU = 251;

    GATT_ExchangeMTU(connHandle, &req,
                     ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity));

    // Fastest possible connection interval (7.5 ms = 6 * 1.25ms)
    gapUpdateLinkParamReq_t updateReq = {
        .connectionHandle = connHandle,
        .intervalMin = 6,
        .intervalMax = 12,
        .connLatency  = 0,
        .connTimeout  = 200
    };
    GAP_UpdateLinkParamReq(&updateReq);
}
/*********************************************************************
******************* SENSORCONTROLLER / ADC TASK *******************
*********************************************************************/
void SensorController_init(void)
{
    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    printf("Sensor Controller initialized! \n");
}

static void scCtrlReadyCallback(void)
{
  // Notify application `Control READY` is active
  SimplePeripheral_enqueueMsg(SP_SC_CTRL_READY, 0);
}

static void scTaskAlertCallback(void)
{
  // Notify application `Task ALERT` is active
  SimplePeripheral_enqueueMsg(SP_SC_TASK_ALERT, 0);
}

// ----------------------------------------------------------------------
// CRITICAL: SCIF Alert Handler
// ----------------------------------------------------------------------
void ADC_processTaskAlert(void)
{
    // 1. Early Exit / Safety
    // If driver isn't powered, we shouldn't be here. Clear and bail.
    if (!g_adc.powered) 
    {
        scifClearAlertIntSource();
        scifAckAlertEvents();
        return;
    }

    // 2. Clear Interrupt Source Immediately
    // Acknowledge the hardware interrupt so we don't miss future edges
    scifClearAlertIntSource();

    // 3. Snapshot SCIF State
    // We read volatile hardware pointers once into local variables.
    // SCIF acts as Producer, We are Consumer. 
    volatile uint16_t* scif_samples = scifTaskData.adcTrigger.output.pSamples;
    uint16_t scif_head = scifTaskData.adcTrigger.state.head;
    uint16_t scif_tail = scifTaskData.adcTrigger.state.tail;
    
    // Constants
    const uint16_t buffer_size = SCIF_BUFFER_SIZE;
    const uint16_t matrix_size = scifTaskData.adcTrigger.cfg.matrixSize;

    // Calculate Available Samples (Circular Buffer Logic)
    uint16_t samples_available = (scif_head >= scif_tail) ? 
                                 (scif_head - scif_tail) :
                                 (buffer_size - scif_tail + scif_head);

    // 4. Process Data in Chunks (Matrix Size)
    // We do NOT disable interrupts here. Bit-packing is CPU work, let the Radio/RTOS run.
    bool data_written = false;

    while (samples_available >= matrix_size) 
    {
        // Temp buffer: 32 samples (12-bit) packs into 48 bytes
        // Ensure this is large enough for your max matrix_size (32)
        uint8_t packed_buffer[48]; 
        uint16_t packed_bytes = 0;
        
        // BIT PACKING: 2x 12-bit samples -> 3 Bytes
        for (uint16_t i = 0; i + 1 < matrix_size; i += 2) 
        {
            // Read Sample 1
            uint16_t s1 = scif_samples[scif_tail] & 0x0FFF;
            scif_tail = (scif_tail + 1) % buffer_size;
            
            // Read Sample 2
            uint16_t s2 = scif_samples[scif_tail] & 0x0FFF;
            scif_tail = (scif_tail + 1) % buffer_size;
            
            // Pack: [S1_Low] [S1_Hi | S2_Lo] [S2_Hi]
            packed_buffer[packed_bytes++] = (uint8_t)(s1 & 0xFF);
            packed_buffer[packed_bytes++] = (uint8_t)(((s1 >> 8) & 0x0F) | ((s2 & 0x0F) << 4));
            packed_buffer[packed_bytes++] = (uint8_t)((s2 >> 4) & 0xFF);
        }
        
        // Handle Odd Sample Count (if matrix_size is odd)
        if (matrix_size & 1) 
        {
            uint16_t s = scif_samples[scif_tail] & 0x0FFF;
            scif_tail = (scif_tail + 1) % buffer_size;
            
            packed_buffer[packed_bytes++] = (uint8_t)(s & 0xFF);
            packed_buffer[packed_bytes++] = (uint8_t)((s >> 8) & 0x0F);
        }
        
        // 5. Write to Application Ring Buffer
        // This handles critical sections internally if needed
        ADC_writeBuffer(packed_buffer, packed_bytes);
        
        // 6. METRIC CORRECTION
        // rb_write() increments samples_processed by 1 (per call).
        // For ADC, 1 call = 'matrix_size' samples. We adjust it here.
        if (matrix_size > 1) {
            g_adc.stats.samples_processed += (matrix_size - 1);
        }

        samples_available -= matrix_size;
        data_written = true;
    }

    // 7. Handshake with SCIF
    // Update the shared tail so SCIF knows we consumed the data
    scifTaskData.adcTrigger.state.tail = scif_tail;
    
    // Re-enable SCIF alerts for the next batch
    scifTaskData.adcTrigger.state.alertEnabled = 1;
    scifAckAlertEvents();

    // 8. Trigger BLE Transmission
    if (data_written && syncEvent != NULL) 
    {
        Event_post(syncEvent, SP_THROUGHPUT_EVT);
    }
}


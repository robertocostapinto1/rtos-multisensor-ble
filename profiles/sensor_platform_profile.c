/******************************************************************************
 @file  sensor_platform_profile.c

 @brief This file contains the Sensor Platform custom GATT profile implementation.
        It manages ADC, Gyroscope, and Accelerometer configuration and high-throughput
        data streaming.

 Target Device: cc13xx_cc26xx
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <icall.h>
#include "util.h"
#include "icall_ble_api.h"
#include "sensor_platform_profile.h"

/*********************************************************************
 * CONSTANTS
 */
// Total number of attributes in the table
#define SERVAPP_NUM_ATTR_SUPPORTED        38

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID: 0xFFF0
CONST uint8 sensorProfileServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SENSOR_PROFILE_SERV_UUID), HI_UINT16(SENSOR_PROFILE_SERV_UUID)
};

// --- ADC UUIDs ---
CONST uint8 AdcStartUUID[ATT_BT_UUID_SIZE]  = { LO_UINT16(SPP_ADC_START_THROUGHPUT_UUID), HI_UINT16(SPP_ADC_START_THROUGHPUT_UUID) };
CONST uint8 AdcFreqUUID[ATT_BT_UUID_SIZE]   = { LO_UINT16(SPP_ADC_FREQUENCY_UUID), HI_UINT16(SPP_ADC_FREQUENCY_UUID) };
CONST uint8 AdcSensorsUUID[ATT_BT_UUID_SIZE]= { LO_UINT16(SPP_ADC_SENSORS_UUID), HI_UINT16(SPP_ADC_SENSORS_UUID) };
CONST uint8 AdcNotifyUUID[ATT_BT_UUID_SIZE] = { LO_UINT16(SPP_ADC_NOTIFY_UUID), HI_UINT16(SPP_ADC_NOTIFY_UUID) };

// --- GYRO UUIDs ---
CONST uint8 GyrStartUUID[ATT_BT_UUID_SIZE]  = { LO_UINT16(SPP_GYR_START_THROUGHPUT_UUID), HI_UINT16(SPP_GYR_START_THROUGHPUT_UUID) };
CONST uint8 GyrFreqUUID[ATT_BT_UUID_SIZE]   = { LO_UINT16(SPP_GYR_FREQUENCY_UUID), HI_UINT16(SPP_GYR_FREQUENCY_UUID) };
CONST uint8 GyrNotifyUUID[ATT_BT_UUID_SIZE] = { LO_UINT16(SPP_GYR_NOTIFY_UUID), HI_UINT16(SPP_GYR_NOTIFY_UUID) };

// --- ACCEL UUIDs ---
CONST uint8 AccStartUUID[ATT_BT_UUID_SIZE]  = { LO_UINT16(SPP_ACC_START_THROUGHPUT_UUID), HI_UINT16(SPP_ACC_START_THROUGHPUT_UUID) };
CONST uint8 AccFreqUUID[ATT_BT_UUID_SIZE]   = { LO_UINT16(SPP_ACC_FREQUENCY_UUID), HI_UINT16(SPP_ACC_FREQUENCY_UUID) };
CONST uint8 AccNotifyUUID[ATT_BT_UUID_SIZE] = { LO_UINT16(SPP_ACC_NOTIFY_UUID), HI_UINT16(SPP_ACC_NOTIFY_UUID) };

// --- SYSTEM UUIDs ---
CONST uint8 TelemetryUUID[ATT_BT_UUID_SIZE]     = { LO_UINT16(SPP_TELEMETRY_UUID), HI_UINT16(SPP_TELEMETRY_UUID) };

/*********************************************************************
 * LOCAL VARIABLES
 */

static sensorPlatformProfileCBs_t *sensorProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - Variables
 */

// Service Attribute
static CONST gattAttrType_t sensorProfileService = { ATT_BT_UUID_SIZE, sensorProfileServUUID };

// --- ADC Variables ---
static uint8 AdcStartProps = GATT_PROP_WRITE;
static uint8 AdcStartVal = 0;
static uint8 AdcStartDesc[] = "ADC START";

static uint8 AdcFreqProps = GATT_PROP_WRITE;
static uint8 AdcFreqVal = 0;
static uint8 AdcFreqDesc[] = "ADC FREQUENCY";

static uint8 AdcSensorsProps = GATT_PROP_WRITE;
static uint8 AdcSensorsVal = 0;
static uint8 AdcSensorsDesc[] = "ADC SENSORS";

static uint8 AdcNotifyProps = GATT_PROP_NOTIFY;
static uint8 AdcNotifyVal[SPP_DATA_CHAR_LEN] = {0};
static gattCharCfg_t *AdcNotifyConfig;
static uint8 AdcNotifyDesc[] = "ADC NOTIFY";

// --- GYRO Variables ---
static uint8 GyrStartProps = GATT_PROP_WRITE;
static uint8 GyrStartVal = 0;
static uint8 GyrStartDesc[] = "GYR START";

static uint8 GyrFreqProps = GATT_PROP_WRITE;
static uint8 GyrFreqVal = 0;
static uint8 GyrFreqDesc[] = "GYR FREQUENCY";

static uint8 GyrNotifyProps = GATT_PROP_NOTIFY;
static uint8 GyrNotifyVal[SPP_DATA_CHAR_LEN] = {0};
static gattCharCfg_t *GyrNotifyConfig;
static uint8 GyrNotifyDesc[] = "GYR NOTIFY";

// --- ACCEL Variables ---
static uint8 AccStartProps = GATT_PROP_WRITE;
static uint8 AccStartVal = 0;
static uint8 AccStartDesc[] = "ACC START";

static uint8 AccFreqProps = GATT_PROP_WRITE;
static uint8 AccFreqVal = 0;
static uint8 AccFreqDesc[] = "ACC FREQUENCY";

static uint8 AccNotifyProps = GATT_PROP_NOTIFY;
static uint8 AccNotifyVal[SPP_DATA_CHAR_LEN] = {0};
static gattCharCfg_t *AccNotifyConfig;
static uint8 AccNotifyDesc[] = "ACC NOTIFY";

// --- TELEMETRY Variables ---
static uint8 TelemetryProps = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8 TelemetryVal[SPP_TELEMETRY_LEN] = {0};
static gattCharCfg_t *TelemetryConfig;
static uint8 TelemetryDesc[] = "TELEMETRY";

/*********************************************************************
 * Profile Attributes - Table
 */
gattAttribute_t sensorProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // Simple Profile Service
  { { ATT_BT_UUID_SIZE, primaryServiceUUID }, GATT_PERMIT_READ, 0, (uint8 *)&sensorProfileService },

  // --- ADC ---
  // Start
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &AdcStartProps },
  { { ATT_BT_UUID_SIZE, AdcStartUUID }, GATT_PERMIT_WRITE, 0, &AdcStartVal },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, AdcStartDesc },

  // Frequency
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &AdcFreqProps },
  { { ATT_BT_UUID_SIZE, AdcFreqUUID }, GATT_PERMIT_WRITE, 0, &AdcFreqVal },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, AdcFreqDesc },

  // Sensors
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &AdcSensorsProps },
  { { ATT_BT_UUID_SIZE, AdcSensorsUUID }, GATT_PERMIT_WRITE, 0, &AdcSensorsVal },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, AdcSensorsDesc },

  // Notify (Index 12: CCCD)
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &AdcNotifyProps },
  { { ATT_BT_UUID_SIZE, AdcNotifyUUID }, 0, 0, AdcNotifyVal },
  { { ATT_BT_UUID_SIZE, clientCharCfgUUID }, GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8 *)&AdcNotifyConfig },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, AdcNotifyDesc },

  // --- GYRO ---
  // Start
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &GyrStartProps },
  { { ATT_BT_UUID_SIZE, GyrStartUUID }, GATT_PERMIT_WRITE, 0, &GyrStartVal },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, GyrStartDesc },

  // Frequency
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &GyrFreqProps },
  { { ATT_BT_UUID_SIZE, GyrFreqUUID }, GATT_PERMIT_WRITE, 0, &GyrFreqVal },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, GyrFreqDesc },

  // Notify (Index 22: CCCD)
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &GyrNotifyProps },
  { { ATT_BT_UUID_SIZE, GyrNotifyUUID }, 0, 0, GyrNotifyVal },
  { { ATT_BT_UUID_SIZE, clientCharCfgUUID }, GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8 *)&GyrNotifyConfig },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, GyrNotifyDesc },

  // --- ACCEL ---
  // Start
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &AccStartProps },
  { { ATT_BT_UUID_SIZE, AccStartUUID }, GATT_PERMIT_WRITE, 0, &AccStartVal },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, AccStartDesc },

  // Frequency
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &AccFreqProps },
  { { ATT_BT_UUID_SIZE, AccFreqUUID }, GATT_PERMIT_WRITE, 0, &AccFreqVal },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, AccFreqDesc },

  // Notify (Index 32: CCCD)
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &AccNotifyProps },
  { { ATT_BT_UUID_SIZE, AccNotifyUUID }, 0, 0, AccNotifyVal },
  { { ATT_BT_UUID_SIZE, clientCharCfgUUID }, GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8 *)&AccNotifyConfig },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, AccNotifyDesc },

  // --- TELEMETRY ---
  // (Index 36: CCCD)
  { { ATT_BT_UUID_SIZE, characterUUID }, GATT_PERMIT_READ, 0, &TelemetryProps },
  { { ATT_BT_UUID_SIZE, TelemetryUUID }, GATT_PERMIT_READ, 0, TelemetryVal },
  { { ATT_BT_UUID_SIZE, clientCharCfgUUID }, GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8 *)&TelemetryConfig },
  { { ATT_BT_UUID_SIZE, charUserDescUUID }, GATT_PERMIT_READ, 0, TelemetryDesc },

};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sensorProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                          uint16_t maxLen, uint8_t method);
static bStatus_t sensorProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset,
                                           uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
CONST gattServiceCBs_t sensorProfileCBs =
{
  sensorProfile_ReadAttrCB,  // Read callback function pointer
  sensorProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

bStatus_t SensorPlatformProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Allocate Client Characteristic Configuration tables
  AdcNotifyConfig     = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS );
  GyrNotifyConfig     = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS );
  AccNotifyConfig     = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS );
  TelemetryConfig     = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) * MAX_NUM_BLE_CONNS );

  if ( !AdcNotifyConfig || !GyrNotifyConfig || !AccNotifyConfig || !TelemetryConfig )
  {
    return ( bleMemAllocError );
  }

  // Initialize CCCDs
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, AdcNotifyConfig );
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, GyrNotifyConfig );
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, AccNotifyConfig );
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, TelemetryConfig );

  if ( services & SENSOR_PROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( sensorProfileAttrTbl,
                                          GATT_NUM_ATTRS( sensorProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &sensorProfileCBs );
  }

  return ( status );
}

bStatus_t SensorPlatformProfile_RegisterAppCBs( sensorPlatformProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    sensorProfile_AppCBs = appCallbacks;
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

bStatus_t SensorPlatformProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SPP_ADC_START_THROUGHPUT_CHAR:
        if ( len == sizeof ( uint8 ) ) AdcStartVal = *((uint8*)value);
        else ret = bleInvalidRange;
        break;

    case SPP_ADC_FREQUENCY_CHAR:
        if ( len == sizeof ( uint8 ) ) AdcFreqVal = *((uint8*)value);
        else ret = bleInvalidRange;
        break;

    case SPP_ADC_SENSORS_CHAR:
        if ( len == sizeof ( uint8 ) ) AdcSensorsVal = *((uint8*)value);
        else ret = bleInvalidRange;
        break;

    case SPP_ADC_NOTIFY_CHAR:
        if (len <= SPP_DATA_CHAR_LEN ) {
            VOID memcpy(AdcNotifyVal, value, len);
            GATTServApp_ProcessCharCfg( AdcNotifyConfig, AdcNotifyVal, FALSE,
                                        sensorProfileAttrTbl, GATT_NUM_ATTRS( sensorProfileAttrTbl ),
                                        INVALID_TASK_ID, sensorProfile_ReadAttrCB );
        } else ret = bleInvalidRange;
        break;

    case SPP_GYR_START_THROUGHPUT_CHAR:
        if ( len == sizeof ( uint8 ) ) GyrStartVal = *((uint8*)value);
        else ret = bleInvalidRange;
        break;

    case SPP_GYR_FREQUENCY_CHAR:
        if ( len == sizeof ( uint8 ) ) GyrFreqVal = *((uint8*)value);
        else ret = bleInvalidRange;
        break;

    case SPP_GYR_NOTIFY_CHAR:
        if (len <= SPP_DATA_CHAR_LEN ) {
            VOID memcpy(GyrNotifyVal, value, len);
            GATTServApp_ProcessCharCfg( GyrNotifyConfig, GyrNotifyVal, FALSE,
                                        sensorProfileAttrTbl, GATT_NUM_ATTRS( sensorProfileAttrTbl ),
                                        INVALID_TASK_ID, sensorProfile_ReadAttrCB );
        } else ret = bleInvalidRange;
        break;

    case SPP_ACC_START_THROUGHPUT_CHAR:
        if ( len == sizeof ( uint8 ) ) AccStartVal = *((uint8*)value);
        else ret = bleInvalidRange;
        break;

    case SPP_ACC_FREQUENCY_CHAR:
        if ( len == sizeof ( uint8 ) ) AccFreqVal = *((uint8*)value);
        else ret = bleInvalidRange;
        break;

    case SPP_ACC_NOTIFY_CHAR:
        if (len <= SPP_DATA_CHAR_LEN ) {
            VOID memcpy(AccNotifyVal, value, len);
            GATTServApp_ProcessCharCfg( AccNotifyConfig, AccNotifyVal, FALSE,
                                        sensorProfileAttrTbl, GATT_NUM_ATTRS( sensorProfileAttrTbl ),
                                        INVALID_TASK_ID, sensorProfile_ReadAttrCB );
        } else ret = bleInvalidRange;
        break;

    case SPP_TELEMETRY_CHAR:
        if (len <= SPP_TELEMETRY_LEN) {
            VOID memcpy(TelemetryVal, value, len);
            GATTServApp_ProcessCharCfg( TelemetryConfig, TelemetryVal, FALSE,
                                        sensorProfileAttrTbl, GATT_NUM_ATTRS( sensorProfileAttrTbl ),
                                        INVALID_TASK_ID, sensorProfile_ReadAttrCB );
        } else ret = bleInvalidRange;
        break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ( ret );
}

bStatus_t SensorPlatformProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SPP_ADC_START_THROUGHPUT_CHAR:
      *((uint8*)value) = AdcStartVal;
      break;

    case SPP_ADC_FREQUENCY_CHAR:
      *((uint8*)value) = AdcFreqVal;
      break;

    case SPP_ADC_SENSORS_CHAR:
      *((uint8*)value) = AdcSensorsVal;
      break;

    case SPP_GYR_START_THROUGHPUT_CHAR:
      *((uint8*)value) = GyrStartVal;
      break;

    case SPP_GYR_FREQUENCY_CHAR:
      *((uint8*)value) = GyrFreqVal;
      break;

    case SPP_ACC_START_THROUGHPUT_CHAR:
      *((uint8*)value) = AccStartVal;
      break;

    case SPP_ACC_FREQUENCY_CHAR:
      *((uint8*)value) = AccFreqVal;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  return ( ret );
}

/*********************************************************************
 * @fn          sensorProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 */
static bStatus_t sensorProfile_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                          uint16_t maxLen, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint16_t uuid;

  if (offset > 0) return (ATT_ERR_ATTR_NOT_LONG);

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    switch (uuid)
    {
      case SPP_ADC_NOTIFY_UUID:
      case SPP_GYR_NOTIFY_UUID:
      case SPP_ACC_NOTIFY_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
      
      case SPP_TELEMETRY_UUID:
        *pLen = SPP_TELEMETRY_LEN;
        memcpy(pValue, pAttr->pValue, SPP_TELEMETRY_LEN);
        break;
      
      default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return (status);
}

/*********************************************************************
 * @fn      sensorProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 */
static bStatus_t sensorProfile_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len, uint16_t offset,
                                           uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SPP_ADC_START_THROUGHPUT_UUID:
      case SPP_ADC_FREQUENCY_UUID:
      case SPP_ADC_SENSORS_UUID:
      case SPP_GYR_START_THROUGHPUT_UUID:
      case SPP_GYR_FREQUENCY_UUID:
      case SPP_ACC_START_THROUGHPUT_UUID:
      case SPP_ACC_FREQUENCY_UUID:
        
        if ( offset == 0 ) {
          if ( len != 1 ) status = ATT_ERR_INVALID_VALUE_SIZE;
        } else {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          *pCurValue = pValue[0];

          if      ( pAttr->pValue == &AdcStartVal ) notifyApp = SPP_ADC_START_THROUGHPUT_CHAR;
          else if ( pAttr->pValue == &AdcFreqVal )  notifyApp = SPP_ADC_FREQUENCY_CHAR;
          else if ( pAttr->pValue == &AdcSensorsVal ) notifyApp = SPP_ADC_SENSORS_CHAR;
          else if ( pAttr->pValue == &GyrStartVal ) notifyApp = SPP_GYR_START_THROUGHPUT_CHAR;
          else if ( pAttr->pValue == &GyrFreqVal )  notifyApp = SPP_GYR_FREQUENCY_CHAR;
          else if ( pAttr->pValue == &AccStartVal ) notifyApp = SPP_ACC_START_THROUGHPUT_CHAR;
          else if ( pAttr->pValue == &AccFreqVal )  notifyApp = SPP_ACC_FREQUENCY_CHAR;
        }
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
      {
          status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                  offset, GATT_CLIENT_CFG_NOTIFY);
      }
      break;

      default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  if ( (notifyApp != 0xFF ) && sensorProfile_AppCBs && sensorProfile_AppCBs->pfnSensorProfileChange )
  {
    sensorProfile_AppCBs->pfnSensorProfileChange( notifyApp );
  }
  return ( status );
}
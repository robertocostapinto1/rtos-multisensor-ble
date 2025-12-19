/******************************************************************************
 @file  sensor_platform_profile.h

 @brief This file contains the Sensor Platform custom GATT profile definitions 
        and prototypes. Designed for High-Throughput Multi-Sensor Data Acquisition.

 Target Device: cc13xx_cc26xx
 *****************************************************************************/

#ifndef SENSOR_PLATFORM_PROFILE_H
#define SENSOR_PLATFORM_PROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"

/*********************************************************************
 * CONSTANTS
 */

// Profile Parameters (Characteristic Indices for Set/Get Parameter)
#define SPP_ADC_START_THROUGHPUT_CHAR             0
#define SPP_ADC_FREQUENCY_CHAR                    1
#define SPP_ADC_SENSORS_CHAR                      2
#define SPP_ADC_NOTIFY_CHAR                       3
#define SPP_GYR_START_THROUGHPUT_CHAR             4
#define SPP_GYR_FREQUENCY_CHAR                    5
#define SPP_GYR_NOTIFY_CHAR                       6
#define SPP_ACC_START_THROUGHPUT_CHAR             7
#define SPP_ACC_FREQUENCY_CHAR                    8
#define SPP_ACC_NOTIFY_CHAR                       9
#define SPP_TELEMETRY_CHAR                        10

// Sensor Platform Service UUID
#define SENSOR_PROFILE_SERV_UUID                  0xFFF0

// Characteristic UUIDs
#define SPP_ADC_START_THROUGHPUT_UUID             0xFFF1
#define SPP_ADC_FREQUENCY_UUID                    0xFFF2
#define SPP_ADC_SENSORS_UUID                      0xFFF3
#define SPP_ADC_NOTIFY_UUID                       0xFFF4

#define SPP_GYR_START_THROUGHPUT_UUID             0xFFF5
#define SPP_GYR_FREQUENCY_UUID                    0xFFF6
#define SPP_GYR_NOTIFY_UUID                       0xFFF7

#define SPP_ACC_START_THROUGHPUT_UUID             0xFFF8
#define SPP_ACC_FREQUENCY_UUID                    0xFFF9
#define SPP_ACC_NOTIFY_UUID                       0xFFFA

#define SPP_TELEMETRY_UUID                        0xFFFB

// Profile Service bit fields
#define SENSOR_PROFILE_SERVICE                    0x00000001

// Data Length Configurations
#define SPP_DATA_CHAR_LEN                         251  // Max MTU - Overhead
#define SPP_TELEMETRY_LEN                         62   // Structured performance data

/*********************************************************************
 * TYPEDEFS
 */

// Callback when a characteristic value has changed
typedef void (*sensorProfileChange_t)( uint8 paramID );

typedef struct
{
  sensorProfileChange_t  pfnSensorProfileChange;  // Called when characteristic value changes
} sensorPlatformProfileCBs_t;

/*********************************************************************
 * API FUNCTIONS
 */

/*
 * SensorPlatformProfile_AddService - Initializes the GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t SensorPlatformProfile_AddService( uint32 services );

/*
 * SensorPlatformProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 */
extern bStatus_t SensorPlatformProfile_RegisterAppCBs( sensorPlatformProfileCBs_t *appCallbacks );

/*
 * SensorPlatformProfile_SetParameter - Set a GATT Profile parameter.
 */
extern bStatus_t SensorPlatformProfile_SetParameter( uint8 param, uint8 len, void *value );

/*
 * SensorPlatformProfile_GetParameter - Get a GATT Profile parameter.
 */
extern bStatus_t SensorPlatformProfile_GetParameter( uint8 param, void *value );

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_PLATFORM_PROFILE_H */
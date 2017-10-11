/*******************************************************************************
  Filename:       st_util.h
  Revised:        $Date: 2014-06-17 00:12:16 +0200 (ti, 17 jun 2014) $
  Revision:       $Revision: 39036 $

  Description:    Utilities for Sensor Tag services


  Copyright 2012 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

#ifndef ST_UTIL_H
#define ST_UTIL_H

/*********************************************************************
 * MACROS
 */
#include "bcomdef.h"
#include "gatt.h"

/*********************************************************************
 * MACROS
 */
#ifdef GATT_TI_UUID_128_BIT

// TI Base 128-bit UUID: F000XXXX-0451-4000-B000-000000000000
#define TI_UUID_SIZE        ATT_UUID_SIZE
#define TI_UUID(uuid)       TI_BASE_UUID_128(uuid)

#else

// Using 16-bit UUID
#define TI_UUID_SIZE        ATT_BT_UUID_SIZE
#define TI_UUID(uuid)       LO_UINT16(uuid), HI_UINT16(uuid)

#endif

// Utility string macros
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Profile Parameter Identifiers
#define SENSOR_DATA                     0  
#define SENSOR_CONF                     1  
#define SENSOR_PERI                     2  

// Data readout periods (range 100 - 2550 ms)
#define SENSOR_MIN_UPDATE_PERIOD        100     // Minimum 100 milliseconds
#define SENSOR_PERIOD_RESOLUTION        10      // Resolution 10 milliseconds

// Common values for turning a sensor on and off + config/status
#define ST_CFG_SENSOR_DISABLE           0x00
#define ST_CFG_SENSOR_ENABLE            0x01
#define ST_CFG_ERROR                    0xFF

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*sensorChange_t)(uint8_t paramID);

typedef struct
{
  sensorChange_t pfnSensorChange;  // Called when characteristic value changes
} sensorCBs_t;

/*-------------------------------------------------------------------
 * FUNCTIONS
 */

extern bStatus_t utilExtractUuid16(gattAttribute_t *pAttr, uint16_t *pValue);

#endif /* ST_UTIL_H */


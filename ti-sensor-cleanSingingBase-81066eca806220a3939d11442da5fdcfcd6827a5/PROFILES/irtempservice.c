/*******************************************************************************
  Filename:       irtempservice.c
  Revised:        $Date: 2014-06-17 00:12:16 +0200 (ti, 17 jun 2014) $
  Revision:       $Revision: 39036 $

  Description:    IR Temperature service.


  Copyright 2012 - 2015 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "string.h"

#include "irtempservice.h"
#include "st_util.h"
#include "irtempdata.h"
#include "irtempconfig.h"
#include "irtempperiod.h"
#include "irtempreadoutpos.h"
#include "irtempmeasurementcount.h"
#include "irtempcontractid.h"
#include "irtempstarttime.h"
#include "irtempstartcount.h"
#include "irtempsensorcount.h"
#include "irtempcalibration.h"
#include "irtempsinglemeasurement.h"
#include "irtemphash.h"

#include "SensorTag_Tmp.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// The temperature sensor does not support the 100 ms update rate
#undef SENSOR_MIN_UPDATE_PERIOD
#define SENSOR_MIN_UPDATE_PERIOD  300 // Minimum 300 milliseconds

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID
static CONST uint8_t sensorServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_SERV_UUID),
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static sensorCBs_t *sensor_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t sensorService = { TI_UUID_SIZE, sensorServiceUUID };

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t sensorAttrTable[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8_t *)&sensorService                 /* pValue */
  },

  IRTEMPDATA_CHAR_DECL,
  IRTEMPDATA_CHAR_VALUE,

  /*// Characteristic configuration
  {
    { ATT_BT_UUID_SIZE, clientCharCfgUUID },
    GATT_PERMIT_READ | GATT_PERMIT_WRITE,
    0,
    (uint8_t *)&sensorDataConfig
  },*/

#ifdef USER_DESCRIPTION
  IRTEMPDATA_CHAR_USER_DESC,
#endif

  IRTEMPCONFIG_CHAR_DECL,
  IRTEMPCONFIG_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPCONFIG_CHAR_USER_DESC,
#endif

  IRTEMPPERIOD_CHAR_DECL,
  IRTEMPPERIOD_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPPERIOD_CHAR_USER_DESC,
#endif

  IRTEMPREADOUTPOS_CHAR_DECL,
  IRTEMPREADOUTPOS_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPREADOUTPOS_CHAR_USER_DESC,
#endif

  IRTEMPMEASUREMENTCOUNT_CHAR_DECL,
  IRTEMPMEASUREMENTCOUNT_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPMEASUREMENTCOUNT_CHAR_USER_DESC,
#endif

  IRTEMPCONTRACTID_CHAR_DECL,
  IRTEMPCONTRACTID_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPCONTRACTID_CHAR_USER_DESC,
#endif

  IRTEMPSTARTTIME_CHAR_DECL,
  IRTEMPSTARTTIME_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPSTARTTIME_CHAR_USER_DESC,
#endif

  IRTEMPSTARTCOUNT_CHAR_DECL,
  IRTEMPSTARTCOUNT_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPSTARTCOUNT_CHAR_USER_DESC,
#endif

  IRTEMPSENSORCOUNT_CHAR_DECL,
  IRTEMPSENSORCOUNT_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPSENSORCOUNT_CHAR_USER_DESC,
#endif

  IRTEMPCALIBRATION_CHAR_DECL,
  IRTEMPCALIBRATION_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPCALIBRATION_CHAR_USER_DESC,
#endif

  IRTEMPSINGLE_MEASUREMENT_CHAR_DECL,
  IRTEMPSINGLE_MEASUREMENT_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPSINGLE_MEASUREMENT_CHAR_USER_DESC,
#endif

  IRTEMPHASH_CHAR_DECL,
  IRTEMPHASH_CHAR_VALUE,

#ifdef USER_DESCRIPTION
  IRTEMPHASH_CHAR_USER_DESC,
#endif




};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method);
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// IR Temperature Profile Service Callbacks
static CONST gattServiceCBs_t sensorCBs =
{
  sensor_ReadAttrCB,  // Read callback function pointer
  sensor_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

uint32_t IRTemp_getMeasurementCount()
{
  return measurementCount;
}

void IRTemp_setMeasurementCount(uint32_t count)
{
  measurementCount = count;
}

void *IRTemp_getMeasurementRamLoc()
{
  if (!readoutMeasurements) readoutMeasurements = (TempMeasurement *)ICall_malloc(
      IRTEMPERATURE_BLE_AGGREGATION_COUNT * sizeof(TempMeasurement));

  return readoutMeasurements;
}

uint32_t IRTemp_getReadoutPos()
{
  return sensorReadoutPos;
}

void IRTemp_incMeasurementCount(int count)
{
  measurementCount += count;
}

uint32_t IRTemp_getReadoutLength()
{
  uint32_t readoutLength = IRTEMPERATURE_BLE_AGGREGATION_COUNT;
  if (measurementCount <= sensorReadoutPos + readoutLength)
    readoutLength = measurementCount - sensorReadoutPos;

  return readoutLength;
}

double *IRTemp_getCalibrationCoefficientsPtr()
{
  return sensorCalibrationCoefficients;
}

void IRTemp_respondData()
{
  int len = IRTemp_getReadoutLength() * sizeof(TempMeasurement);

  attReadRsp_t response;
  response.pValue = GATT_bm_alloc(dataReadConnHandle, ATT_READ_RSP, len, NULL);

  if (response.pValue)
  {
    response.len = len;
    memcpy(response.pValue, readoutMeasurements, len);

    uint8_t status = ATT_ReadRsp(dataReadConnHandle, &response);
    if(status != SUCCESS)
    {
      GATT_bm_free((gattMsg_t*)&response, ATT_READ_RSP);
    }
  }
}

void IRTemp_respondHashData()
{
  // TODO set length to 32 (only on 16 for debugging)
  int len = 32;
  attReadRsp_t response;
  response.pValue = GATT_bm_alloc(dataReadHashConnHandle, ATT_READ_RSP, len, NULL);

  if (response.pValue)
  {
    response.len = len;
    memcpy(response.pValue, (const void *)&hashValue, len);

    uint8_t status = ATT_ReadRsp(dataReadConnHandle, &response);

    if(status != SUCCESS)
    {
      GATT_bm_free((gattMsg_t*)&response, ATT_READ_RSP);
    }
  }
}

void IRTemp_getStartTime(uint8_t *startTime)
{
  memcpy(startTime, sensorStartTime, 8);
}

void IRTemp_setStartTime(uint8_t *startTime)
{
  memcpy(sensorStartTime, startTime, 8);
}

void IRTemp_setContractId(uint8_t *contractId)
{
  memcpy(sensorContractId, contractId, CONTRACT_ID_SIZE);
}

void IRTemp_getContractId(uint8_t *contractId)
{
  memcpy(contractId, sensorContractId, CONTRACT_ID_SIZE);
}

void IRTemp_setRestartCounter(uint8_t counter)
{
  sensorStartCount = counter;
}

void IRTemp_setHashValue(uint8_t *value)
{
  memcpy(hashValue, value, 32);
}
/*********************************************************************
 * @fn      IRTemp_addService
 *
 * @brief   Initializes the IR Temperature Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t IRTemp_addService(void)
{
  /*// Allocate Client Characteristic Configuration table
  sensorDataConfig = (gattCharCfg_t *)ICall_malloc(sizeof(gattCharCfg_t) *
                                                    linkDBNumConns);
  if (sensorDataConfig == NULL)
  {
    return (bleMemAllocError);
  }
  
  // Register with Link DB to receive link status change callback
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, sensorDataConfig);*/

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( sensorAttrTable,
                                      GATT_NUM_ATTRS (sensorAttrTable),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &sensorCBs );
}


/*********************************************************************
 * @fn      IRTemp_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t IRTemp_registerAppCBs(sensorCBs_t *appCallbacks)
{
  if (sensor_AppCBs == NULL)
  {
    if (appCallbacks != NULL)
    {
      sensor_AppCBs = appCallbacks;
    }

    return (SUCCESS);
  }

  return (bleAlreadyInRequestedMode);
}

/*********************************************************************
 * @fn      IRTemp_setParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t IRTemp_setParameter(uint8_t param, uint8_t len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case SENSOR_CONF:
      irtempconfig_SetParameter(len, value, &ret);
      break;

    case SENSOR_PERI:
      irtempperiod_SetParameter(len, value, &ret);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn      IRTemp_getParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  bStatus_t
 */
bStatus_t IRTemp_getParameter(uint8_t param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case SENSOR_CONF:
      irtempconfig_GetParameter(value);
      break;

    case SENSOR_PERI:
      irtempperiod_GetParameter(value);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}

/*********************************************************************
 * @fn          sensor_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message 
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t sensor_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                   uint8_t *pValue, uint16_t *pLen, 
                                   uint16_t offset, uint16_t maxLen,
                                   uint8_t method)
{
  uint16_t uuid;
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if (gattPermitAuthorRead(pAttr->permissions))
  {
    // Insufficient authorization
    return (ATT_ERR_INSUFFICIENT_AUTHOR);
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case IRTEMPERATURE_DATA_UUID:
      status = blePending;
      dataReadConnHandle = connHandle;
      SensorTag_enqueueMsg(ST_READ_OUT_FLASH, 0, 0);
      break;

    case IRTEMPERATURE_SINGLE_MEASUREMENT_UUID:
      {
        status = SUCCESS;
        *pLen = sizeof(TempMeasurement);
        TempMeasurement tmp = SensorTagTmp_readSingleMeasurement();
        memcpy(pValue, &tmp, *pLen);
      }
      break;

    case IRTEMPERATURE_CONF_UUID:
    case IRTEMPERATURE_PERI_UUID:
    case IRTEMPERATURE_SENSOR_COUNT_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;

    case IRTEMPERATURE_READOUT_POS_UUID:
      irtempreadoutpos_ReadAttribute(pValue, pLen);
      break;

    case IRTEMPERATURE_DATA_COUNT_UUID:
      irtempmeasurementcount_ReadAttribute(pValue, pLen);
      break;

    case IRTEMPERATURE_CONTRACT_ID_UUID:
      irtempcontractid_ReadAttribute(pValue, pLen);
      break;

    case IRTEMPERATURE_START_TIME_UUID:
      irtempstarttime_ReadAttribute(pValue, pLen);
      break;

    case IRTEMPERATURE_START_COUNT_UUID:
      irtempstartcount_ReadAttribute(pValue, pLen);
      break;

    case IRTEMPERATURE_CALIBRATION_UUID:
      irtempCalibration_ReadAttribute(pValue, pLen);
      break;

    case IRTEMPERATURE_HASH_UUID:
      status = blePending;
      dataReadHashConnHandle = connHandle;
      SensorTag_enqueueMsg(ST_READ_OUT_FLASH_HASH, 0, 0);
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return (status);
}



/*********************************************************************
 * @fn      sensor_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t sensor_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t len,
                                    uint16_t offset, uint8_t method)
{
  bStatus_t status = SUCCESS;
  uint8_t notifyApp = 0xFF;
  uint16_t uuid;

  // If attribute permissions require authorization to write, return error
  if (gattPermitAuthorWrite(pAttr->permissions))
  {
    // Insufficient authorization
    return (ATT_ERR_INSUFFICIENT_AUTHOR);
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch (uuid)
  {
    case IRTEMPERATURE_CONF_UUID:
      irtempconfig_WriteAttribute(pAttr, pValue, len, offset, &status, &notifyApp);
      break;

    case IRTEMPERATURE_PERI_UUID:
      if (sensorCfg)
      {
        status = ATT_ERR_WRITE_NOT_PERMITTED;
      }
      else
      {
        irtempperiod_WriteAttribute(pAttr, pValue, len, offset, &status, &notifyApp);
      }
      break;

    case IRTEMPERATURE_READOUT_POS_UUID:
      if (sensorCfg)
      {
        status = ATT_ERR_WRITE_NOT_PERMITTED;
      }
      else
      {
        irtempreadoutpos_WriteAttribute(pAttr, pValue, len, offset, &status, &notifyApp);
      }
      break;

    case IRTEMPERATURE_CONTRACT_ID_UUID:
      if (sensorCfg)
      {
        status = ATT_ERR_WRITE_NOT_PERMITTED;
      }
      else
      {
        irtempcontractid_WriteAttribute(pAttr, pValue, len, offset, &status, &notifyApp);
        SensorTag_enqueueMsg(ST_ERASE_FLASH, 0, 0);
      }
      break;

    case IRTEMPERATURE_START_TIME_UUID:
      if (sensorCfg)
      {
        status = ATT_ERR_WRITE_NOT_PERMITTED;
      }
      else
      {
        irtempstarttime_WriteAttribute(pAttr, pValue, len, offset, &status, &notifyApp);
        SensorTag_enqueueMsg(ST_ERASE_FLASH, 0, 0);
      }
      break;

    case IRTEMPERATURE_SENSOR_COUNT_UUID:
      irtempSensorCount_WriteAttribute(pAttr, pValue, len, offset, &status, &notifyApp);
      break;

    case IRTEMPERATURE_CALIBRATION_UUID:
      if (sensorCfg)
      {
	status = ATT_ERR_WRITE_NOT_PERMITTED;
      }
      else
      {
        irtempCalibration_WriteAttribute(pAttr, pValue, len, offset, &status, &notifyApp);
        SensorTag_enqueueMsg(ST_ERASE_FLASH, 0, 0);
      }
      break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      /*status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY);*/
      // we shouldn't get here since notifications are disabled
      break;

    default:
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a characteristic value changed then callback function 
  // to notify application of change
  if ((notifyApp != 0xFF ) && sensor_AppCBs && sensor_AppCBs->pfnSensorChange)
  {
    sensor_AppCBs->pfnSensorChange(notifyApp);
  }

  return (status);
}

/*********************************************************************
*********************************************************************/

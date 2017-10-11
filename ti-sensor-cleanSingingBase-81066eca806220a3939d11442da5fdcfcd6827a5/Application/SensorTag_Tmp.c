/*******************************************************************************
  Filename:       SensorTag_Tmp.c
  Revised:        $Date: 2013-11-06 17:27:44 +0100 (on, 06 nov 2013) $
  Revision:       $Revision: 35922 $

  Description:    This file contains the Sensor Tag sample application,
                  IR Temperature part, for use with the TI Bluetooth Low
                  Energy Protocol Stack.

  Copyright 2015  Texas Instruments Incorporated. All rights reserved.

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
#include "gatt.h"
#include "gattservapp.h"

#include "irtempservice.h"
#include "SensorTag_Tmp.h"
#include "sensor_tmp007.h"
#include "sensor.h"
#include "Board.h"

#include "string.h"
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include "sha256.h"
#include "uECC.h"



/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define TEMP_SENSOR_PERIOD_RESOLUTION (1000 * 60)

// Delay from sensor enable to reading measurement
// (allow for 250 ms conversion time)
#define TEMP_MEAS_DELAY         275

// Event flag for this sensor
#define SENSOR_EVT              ST_IRTEMPERATURE_SENSOR_EVT

// Task configuration
#define SENSOR_TASK_PRIORITY    1
#define SENSOR_TASK_STACK_SIZE  600

#define MEASUREMENT_CACHE_COUNT  4

// Size of bytes allocated for reading out measurements for signing
#define TEMP_MEAS_READOUT_SIZE  100
#define SHA256_BLOCK_LENGTH  64
#define SHA256_RESULT_LENGTH 32

// Private Key for Signing
const uint8_t privkeyStr[] = {0x22, 0x1D, 0x5B, 0x34, 0xBE, 0x02, 0xC6, 0xC7, 0xC6, 0xEC, 0xD4, 0x7E, 0x37, 0x5D, 0x0E, 0x60, 0xE1, 0x87, 0x0B, 0xAB, 0xF6, 0xBA, 0xF3, 0xBB, 0x7B, 0x64, 0xC6, 0xD1, 0xF5, 0x58, 0x0A, 0xEA};


/*********************************************************************
 * TYPEDEFS
 */

// hash_context Structure for sigingin (see uECC.h)

typedef struct SHA256_HashContext {
    uECC_HashContext uECC;
    SHA256_CTX ctx;
} SHA256_HashContext;

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
// Entity ID used to check for source and/or destination of messages
static ICall_EntityID sensorSelfEntity;

// Semaphore used to post events to the application thread
static ICall_Semaphore sensorSem;

// Task setup
static Task_Struct sensorTask;
static Char sensorTaskStack[SENSOR_TASK_STACK_SIZE];

/*********************************************************************
 * STORING MEASUREMENTS
 */
extern uint8 ICallBleTestWrite(uint32_t addr, uint16_t len, uint8_t *data);
extern uint8 ICallBleTestRead(uint8_t page, uint16_t offset, uint16_t len, void *pBuf);
extern uint8 ICallBleEraseFlashPage(uint8_t page);

typedef struct
{
  uint8_t startCounter;
  uint8_t interval;
  uint8_t enable;
  uint8_t startTime[8];
  uint8_t contractId[20];
} SensorFlashDebugInfo;

static TempMeasurement cachedMeasurements[MEASUREMENT_CACHE_COUNT] = {0xff, 0xff, 0xff, 0xff};
static int enableCount = 0;

#define DEBUG_INFO_START_POS      16
#define MEASUREMENT_START_POS     48

#define PAGE_SIZE               0x1000
#define MEASUREMENT_WRITABLE_PAGE_START        (ICALL_STACK0_ADDR - PAGE_SIZE)
#define MEASUREMENT_WRITABLE_PAGE_END          (ICALL_STACK0_ADDR)
#define MEASUREMENT_WRITABLE_PAGE_NO           ((ICALL_STACK0_ADDR / 0x1000) - 1)
#define MEASUREMENT_WRITABLE_COUNT    ((MEASUREMENT_WRITABLE_PAGE_END - MEASUREMENT_WRITABLE_PAGE_START - MEASUREMENT_START_POS) / sizeof(TempMeasurement))


static void SensorTagTmp_writeCalibrationCoefficients()
{
  ICallBleTestWrite(MEASUREMENT_WRITABLE_PAGE_START, CALIBRATION_COEFFICIENT_COUNT * sizeof(double), (uint8_t *)IRTemp_getCalibrationCoefficientsPtr());
}

void SensorTagTmp_eraseMeasurements()
{
  ICallBleEraseFlashPage(MEASUREMENT_WRITABLE_PAGE_NO);
  IRTemp_setMeasurementCount(0);
  SensorTagTmp_writeCalibrationCoefficients();
}

static void writeDebugInfo(SensorFlashDebugInfo *debugInfo)
{
  ICallBleTestWrite(MEASUREMENT_WRITABLE_PAGE_START + DEBUG_INFO_START_POS, sizeof(SensorFlashDebugInfo), (uint8_t *)debugInfo);
}

static void readDebugInfo(SensorFlashDebugInfo *debugInfo)
{
  ICallBleTestRead(
            MEASUREMENT_WRITABLE_PAGE_NO,
            DEBUG_INFO_START_POS,
            sizeof(SensorFlashDebugInfo),
            debugInfo);
}

void SensorTagTmp_disableRecordingOnFlash()
{
  SensorFlashDebugInfo debugInfo;
  readDebugInfo(&debugInfo);
  debugInfo.enable = 0;
  writeDebugInfo(&debugInfo);
}

static void SensorTagTmp_respondData()
{
  IRTemp_respondData();
}

static void SensorTagTmp_respondHashData()
{
  IRTemp_respondHashData();
}

static void SensorTagTmp_putMeasurements()
{
  if (IRTemp_getMeasurementCount() * sizeof(TempMeasurement) +
      MEASUREMENT_WRITABLE_PAGE_START +
      MEASUREMENT_START_POS
      <=
      MEASUREMENT_WRITABLE_PAGE_END)
  {
    if (IRTemp_getMeasurementCount() == 0)
    {
      SensorTagTmp_eraseMeasurements();

      SensorFlashDebugInfo debugInfo;
      IRTemp_getParameter(SENSOR_PERI, &debugInfo.interval);
      IRTemp_getStartTime((uint8_t *)&debugInfo.startTime);
      IRTemp_getContractId((uint8_t *)&debugInfo.contractId);
      debugInfo.enable = ST_CFG_SENSOR_ENABLE;
      debugInfo.startCounter = 0xff;

      IRTemp_setRestartCounter(debugInfo.startCounter);

      writeDebugInfo(&debugInfo);
    }

    uint32_t pos = MEASUREMENT_WRITABLE_PAGE_START +
        MEASUREMENT_START_POS +
        MEASUREMENT_CACHE_COUNT * (IRTemp_getMeasurementCount() / MEASUREMENT_CACHE_COUNT);

    ICallBleTestWrite(pos, MEASUREMENT_CACHE_COUNT * sizeof(TempMeasurement), (uint8_t *)cachedMeasurements);

    IRTemp_incMeasurementCount(1);
  }
}

void SensorTagTmp_readMeasurements(int pos, int count, TempMeasurement *targetLoc)
{
  ICallBleTestRead(
        MEASUREMENT_WRITABLE_PAGE_NO,
        MEASUREMENT_START_POS + pos * sizeof(TempMeasurement),
        count * sizeof(TempMeasurement),
        targetLoc);
}

void SensorTagTmp_getMeasurements()
{
  SensorTagTmp_readMeasurements(IRTemp_getReadoutPos(), IRTemp_getReadoutLength(), IRTemp_getMeasurementRamLoc());

  SensorTagTmp_respondData();
}

static void init_SHA256(const uECC_HashContext *base)
{
    SHA256_HashContext *context = (SHA256_HashContext *)base;
    sha256_init(&context->ctx);
}

static void update_SHA256(const uECC_HashContext *base, const uint8_t *message, unsigned message_size)
{
    SHA256_HashContext *context = (SHA256_HashContext *)base;
    sha256_update(&context->ctx, message, message_size);
}

static void finish_SHA256(const uECC_HashContext *base, uint8_t *hash_result)
{
    SHA256_HashContext *context = (SHA256_HashContext *)base;
    sha256_final(&context->ctx, hash_result);
}

void SensorTagTmp_getMeasurementHash()
{
  uint8_t *measurementPointer=malloc(TEMP_MEAS_READOUT_SIZE);
  BYTE buf[SHA256_BLOCK_SIZE];
  int i;

  if(measurementPointer)
  {
    SHA256_HashContext* ctx_sign = malloc(sizeof(SHA256_HashContext));

    if(ctx_sign)
    {
      sha256_init(&ctx_sign->ctx);

      int timesToReadOut;
      int restMeasurements;
      double resultToRound;
      resultToRound = IRTemp_getMeasurementCount()/TEMP_MEAS_READOUT_SIZE;
      timesToReadOut = (int)resultToRound;

      restMeasurements = IRTemp_getMeasurementCount() % TEMP_MEAS_READOUT_SIZE;
      // never tested as times to readout = 0 with no temperatures
      for(i = 0; i <= (timesToReadOut -1); i++)
      {
        SensorTagTmp_readMeasurements(i*TEMP_MEAS_READOUT_SIZE,TEMP_MEAS_READOUT_SIZE,(TempMeasurement *)measurementPointer);
    	sha256_update(&ctx_sign->ctx, measurementPointer, TEMP_MEAS_READOUT_SIZE);
      }

      // Reading of remaining measurements
      SensorTagTmp_readMeasurements(i*TEMP_MEAS_READOUT_SIZE, restMeasurements, (TempMeasurement *)measurementPointer);

      sha256_update(&ctx_sign->ctx, measurementPointer, restMeasurements);

      sha256_final(&ctx_sign->ctx, buf);

      uint8_t* sig = malloc(64);

      if(sig)
      {
    	// curve = secp256k1
        const struct uECC_Curve_t* curve = uECC_secp256k1();

    	//(2 * result_size + block_size)
    	uint8_t tmp[2 * SHA256_RESULT_LENGTH + SHA256_BLOCK_LENGTH];

    	ctx_sign->uECC.init_hash = &init_SHA256;
    	ctx_sign->uECC.update_hash = &update_SHA256;
    	ctx_sign->uECC.finish_hash = &finish_SHA256;
    	ctx_sign->uECC.block_size = SHA256_BLOCK_LENGTH;
    	ctx_sign->uECC.result_size = SHA256_RESULT_LENGTH;
        ctx_sign->uECC.tmp = tmp;

    	if(uECC_sign_deterministic(privkeyStr, buf, SHA256_RESULT_LENGTH, &ctx_sign->uECC, sig, curve) > 0.9)
    	{
    	  IRTemp_setHashValue(sig);
    	}
    	else
    	{
    	  for(i=0;i<=31;i++)
    	  {
    	    buf[i] = 0;
    	  }
    	  IRTemp_setHashValue(buf);
    	}

      	free(sig);

      }else
    	// if(sig) else
        {
    	  for(i=0;i<=31;i++)
    	  {
    	    buf[i] = 0;
    	  }
    	  IRTemp_setHashValue(buf);
    	}
      free(ctx_sign);

    }else
    //ctx_sign else
    {
	  for(i=0;i<=31;i++)
	  {
	    buf[i] = 0;
	  }
      IRTemp_setHashValue(buf);
    }

    free(measurementPointer);

  }else //if(measurementPointer) else
  {
	  for(i=0;i<=31;i++)
	  {
	    buf[i] = 0;
	  }
	IRTemp_setHashValue(buf);
  }

  SensorTagTmp_respondHashData();
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void sensorTaskFxn(UArg a0, UArg a1);
static void sensorConfigChangeCB(uint8_t paramID);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen);

/*********************************************************************
 * PROFILE CALLBACKS
 */
static sensorCBs_t sensorCallbacks =
{
  sensorConfigChangeCB,  // Characteristic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SensorTagTmp_createTask
 *
 * @brief   Task creation function for the SensorTag
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTmp_createTask(void)
{
  Task_Params taskParames;

  // Create the task for the state machine
  Task_Params_init(&taskParames);
  taskParames.stack = sensorTaskStack;
  taskParames.stackSize = SENSOR_TASK_STACK_SIZE;
  taskParames.priority = SENSOR_TASK_PRIORITY;

  Task_construct(&sensorTask, sensorTaskFxn, &taskParames, NULL);
}

/*********************************************************************
 * @fn      SensorTagTmp_processCharChangeEvt
 *
 * @brief   SensorTag IR temperature event handling
 *
 */
void SensorTagTmp_processCharChangeEvt(uint8_t paramID)
{
  uint8_t newValue;

  switch (paramID)
  {
  case SENSOR_CONF:
    if ((sensorTestResult() & ST_IRTEMP) == 0)
    {
      initCharacteristicValue(SENSOR_CONF, ST_CFG_ERROR, sizeof ( uint8_t ));
    }
    else
    {
      IRTemp_getParameter(SENSOR_CONF, &newValue);

      if (newValue == ST_CFG_SENSOR_DISABLE)
      {
        // Deactivate task
        Task_setPri(Task_handle(&sensorTask), -1);

        SensorTag_enqueueMsg(ST_DISABLE_RECORDING_ON_FLASH, 0, 0);
      }
      else
      {
        ++enableCount;
        Task_setPri(Task_handle(&sensorTask), SENSOR_TASK_PRIORITY);
      }
    }

    // Make sure sensor is disabled
    sensorTmp007Enable(false);
    break;

  case SENSOR_PERI:
    break;

  default:
    // Should not get here
    break;
  }
}

/*********************************************************************
 * @fn      SensorTagTmp_reset
 *
 * @brief   Reset characteristics
 *
 * @param   none
 *
 * @return  none
 */
void SensorTagTmp_reset(void)
{
  initCharacteristicValue(SENSOR_CONF, ST_CFG_SENSOR_DISABLE, sizeof(uint8_t));

  // Initialize the driver
  sensorTmp007Init();
}


/*********************************************************************
* Private functions
*/

static void incStartCounter(SensorFlashDebugInfo *debugInfo)
{
  if (debugInfo->startCounter == 0xff)
  {
    debugInfo->startCounter = 0xfe;
  }
  else
  {
    debugInfo->startCounter &= debugInfo->startCounter << 1;
  }

  writeDebugInfo(debugInfo);
}

static uint32_t countMeasurements()
{
  uint32_t count = 0;
  TempMeasurement *measurements = IRTemp_getMeasurementRamLoc();

  while (1)
  {
    // use a multiple of four to allow for easy copying of the last measurements into the measurement cache
    int READOUT_CHUNK_SIZE = MEASUREMENT_CACHE_COUNT * (IRTEMPERATURE_BLE_AGGREGATION_COUNT / MEASUREMENT_CACHE_COUNT);

    int readableCount = READOUT_CHUNK_SIZE;

    if (count + readableCount > MEASUREMENT_WRITABLE_COUNT)
    {
      readableCount = MEASUREMENT_WRITABLE_COUNT - count;
    }
    if (readableCount <= 0) break;

    SensorTagTmp_readMeasurements(count, readableCount, IRTemp_getMeasurementRamLoc());

    uint8_t *bytePtr = (uint8_t *)measurements;
    int endNotReached = 1;
    while (bytePtr < (uint8_t *)&measurements[readableCount] && (endNotReached = *bytePtr++ != 0xff)) count++;

    if (!endNotReached)
    {
      // Read existing measurements into cache because flash memory doesn't seem to like writing leading 0xff.
      int currentMeasurementCount = count % MEASUREMENT_CACHE_COUNT;
      int currentMeasurementPos = (count - currentMeasurementCount) % READOUT_CHUNK_SIZE;
      memcpy((void *)cachedMeasurements, (void *)&measurements[currentMeasurementPos], currentMeasurementCount * sizeof(TempMeasurement));
      break;
    }
  }

  return count;
}

/*********************************************************************
 * @fn      sensorTaskInit
 *
 * @brief   Initialization function for the SensorTag IR temperature sensor
 *
 */
static void sensorTaskInit(void)
{
  // Add service
  IRTemp_addService();

  // Register callbacks with profile
  IRTemp_registerAppCBs(&sensorCallbacks);

  // Initialize characteristics and sensor driver
  SensorTagTmp_reset();
  initCharacteristicValue(SENSOR_PERI,
                          DEFAULT_TEMP_SENSOR_PERIOD,
                          sizeof ( uint8_t ));
}

static void enableFromFlash()
{
  double *calibrationCoefficients = IRTemp_getCalibrationCoefficientsPtr();
  ICallBleTestRead(
            MEASUREMENT_WRITABLE_PAGE_NO,
            0,
            CALIBRATION_COEFFICIENT_COUNT * sizeof(double),
            calibrationCoefficients);

  SensorFlashDebugInfo debugInfo;
  readDebugInfo(&debugInfo);

  // restart sensor after reboot if enabled in flash
  if (debugInfo.enable == ST_CFG_SENSOR_ENABLE)
  {
    IRTemp_setMeasurementCount(countMeasurements());
    IRTemp_setParameter(SENSOR_PERI, sizeof(uint8_t), &debugInfo.interval);
    IRTemp_setStartTime((uint8_t *)&debugInfo.startTime);
    IRTemp_setContractId((uint8_t *)&debugInfo.contractId);
    SensorTag_updateAdvertDataContractId((uint8_t *)&debugInfo.contractId, CONTRACT_ID_SIZE);

    incStartCounter(&debugInfo);

    IRTemp_setRestartCounter(debugInfo.startCounter);

    initCharacteristicValue(SENSOR_CONF, ST_CFG_SENSOR_ENABLE, sizeof(uint8_t));
    SensorTagTmp_processCharChangeEvt(SENSOR_CONF);
  }
  else
  {
    // Deactivate task (active only when measurement is enabled)
    Task_setPri(Task_handle(&sensorTask), -1);
  }
}


/*********************************************************************
 * TODO: analyze for numerical sources of error, find if the microcontroller uses one's or two's complement
 *
 * The sensor returns a short [-2^16, 2^16-1] that, when divided by 128, represents a temperature
 * in [-256, 255]. We crop to [-10, 50] and map this range to an unsigned byte.
 */
static uint8_t convert(uint16_t data)
{
  double temp = data / 128.0;

  double slope = *IRTemp_getCalibrationCoefficientsPtr();
  double intercept = *(IRTemp_getCalibrationCoefficientsPtr() + 1);
  temp = slope * temp + intercept;

  // TODO create special out of range value
  if (temp < -10) temp = -10;
  if (temp > 50) temp = 50;

  double tmp = temp;
  tmp += 10;
  tmp /= 60.0;
  tmp *= 0xff;

  return (uint8_t)tmp;
}

static TempMeasurement readMeasurement()
{
  typedef union
  {
    struct
    {
      uint16_t tempTarget, tempLocal;
    } v;
    uint16_t a[2];
  } Data_t;

  Data_t data;

  // Read data
  sensorTmp007Enable(true);
  delay_ms(TEMP_MEAS_DELAY);
  sensorTmp007Read(&data.v.tempLocal, &data.v.tempTarget);
  sensorTmp007Enable(false);

  TempMeasurement tmp;
  tmp.temp = convert(data.v.tempLocal);

  return tmp;
}

static void resetMeasurementCache(TempMeasurement **currentMeasurementPtr)
{
  *currentMeasurementPtr = &cachedMeasurements[0];
  memset(&cachedMeasurements[0], 0xff, MEASUREMENT_CACHE_COUNT * sizeof(TempMeasurement));
}

/*********************************************************************
 * @fn      sensorTaskFxn
 *
 * @brief   The task loop of the temperature readout task
 *
 * @return  none
 */
static void sensorTaskFxn(UArg a0, UArg a1)
{
  // Register task with BLE stack
  ICall_registerApp(&sensorSelfEntity, &sensorSem);

  // Initialize the task
  sensorTaskInit();

  enableFromFlash();

  TempMeasurement *currentMeasurement = &cachedMeasurements[0];
  currentMeasurement += IRTemp_getMeasurementCount() % MEASUREMENT_CACHE_COUNT;
  int previouslyDisabled = IRTemp_getMeasurementCount() == 0;
  int previousEnableCount = enableCount;

  // Task loop
  while (true)
  {
    previousEnableCount = enableCount;
    uint8_t enable;
    IRTemp_getParameter(SENSOR_CONF, &enable);

    if (enable == ST_CFG_SENSOR_ENABLE)
    {
      if (previouslyDisabled || currentMeasurement >= &cachedMeasurements[MEASUREMENT_CACHE_COUNT])
      {
        resetMeasurementCache(&currentMeasurement);
      }
      previouslyDisabled = FALSE;

      // It sort of appears like writing to flash memory doesn't like leading 0xff. If we memset 0x00 here,
      // the measurement is written to the correct position in flash with leading 0x00 as expected, but if we
      // memset 0xff nothing appears to be written to flash at all.
      //memset(cachedMeasurements, 0xff, currentMeasurement - cachedMeasurements);

      *currentMeasurement++ = readMeasurement();

      if (enableCount != previousEnableCount)
      {
        IRTemp_setMeasurementCount(0);
        previouslyDisabled = TRUE;
        continue;
      }
      else
      {
        SensorTagTmp_putMeasurements();
      }

      // Next cycle
      uint8_t period;
      IRTemp_getParameter(SENSOR_PERI, &period);
      uint32_t period32 = (uint32_t)period * TEMP_SENSOR_PERIOD_RESOLUTION;
      uint32_t periodSecs = period32 / 1000;

      // delay for 1s at a time only so that after disabling and reenabling the recording it takes max 1s for the
      // first measurement to occur
      int i;
      for (i = 0; i < periodSecs && enable == ST_CFG_SENSOR_ENABLE; ++i)
      {
        if (enableCount != previousEnableCount)
        {
          previouslyDisabled = TRUE;
          break;
        }

        IRTemp_getParameter(SENSOR_CONF, &enable);
        uint32_t delay = (i == 0) ? 1000 - TEMP_MEAS_DELAY : 1000;
        delay_ms(delay);
      }
    }
    else
    {
      previouslyDisabled = TRUE;
      // Don't delay at all. The task is/will be disabled anyway.
      //delay_ms(TEMP_SENSOR_PERIOD_RESOLUTION * DEFAULT_TEMP_SENSOR_PERIOD);
    }
  }
}


/*********************************************************************
 * @fn      sensorChangeCB
 *
 * @brief   Callback from IR Temperature Service indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void sensorConfigChangeCB(uint8_t paramID)
{
  // Wake up the application thread
  SensorTag_charValueChangeCB(SERVICE_ID_TMP, paramID);
}


/*********************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 */
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
                                    uint8_t paramLen)
{
  if (paramLen <= sizeof(uint8_t))
  {
    uint8_t data[sizeof(uint8_t)];
    memset(data,value,paramLen);
    IRTemp_setParameter( paramID, paramLen, data);
  }
  else
  {
    // doesn't happen currently in the project
    uint8_t *data = ICall_malloc(paramLen);
    if (data)
    {
      memset(data,value,paramLen);
      IRTemp_setParameter( paramID, paramLen, data);
      ICall_free(data);
    }
    else
    {
      // TODO look into handling error case here
    }
  }
}



TempMeasurement SensorTagTmp_readSingleMeasurement()
{
  return readMeasurement();
}



/*********************************************************************
*********************************************************************/

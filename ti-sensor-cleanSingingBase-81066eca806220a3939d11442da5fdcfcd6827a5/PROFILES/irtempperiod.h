#ifndef IRTEMPPERIOD_H
#define IRTEMPPERIOD_H

#include "irtempservice.h"

// Characteristic UUID: period
static CONST uint8_t sensorPeriodUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_PERI_UUID),
};

#define SENSOR_PERIOD_DESCR     "Temp. Period"

// Characteristic Properties: period
static uint8_t sensorPeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period
static uint8_t sensorPeriod = DEFAULT_TEMP_SENSOR_PERIOD;

#ifdef USER_DESCRIPTION
// Characteristic User Description: period
static uint8_t sensorPeriodUserDescr[] = SENSOR_PERIOD_DESCR;
#endif

// Characteristic Declaration "Period"
#define IRTEMPPERIOD_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorPeriodProps \
}

// Characteristic Value "Period"
#define IRTEMPPERIOD_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorPeriodUUID }, \
  GATT_PERMIT_READ | GATT_PERMIT_WRITE, \
  0, \
  &sensorPeriod \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description "Period"
#define IRTEMPPERIOD_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorPeriodUserDescr \
}
#endif

/**
 * Set parameter called from above from application layer.
 * Mainly used to initialize this value.
 */
void irtempperiod_SetParameter(uint8_t len, void *value, bStatus_t *ret)
{
  if (len == sizeof(uint8_t))
  {
    sensorPeriod = *((uint8_t*)value);
  }
  else
  {
    *ret = bleInvalidRange;
  }
}

/**
 * Get parameter called from above from application layer.
 */
void irtempperiod_GetParameter(void *value)
{
  *((uint8_t*)value) = sensorPeriod;
}

/**
 * Called from below from Bluetooth GATT layer.
 * Notify application layer about changing measurement period.
 */
void irtempperiod_WriteAttribute(gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, bStatus_t *status, uint8_t *notifyApp)
{
  // Validate the value
  // Make sure it's not a blob oper
  if (offset == 0)
  {
    if (len != 1)
    {
      *status = ATT_ERR_INVALID_VALUE_SIZE;
    }
  }
  else
  {
    *status = ATT_ERR_ATTR_NOT_LONG;
  }
  // Write the value
  if (*status == SUCCESS)
  {
    uint8_t *pCurValue = (uint8_t *)pAttr->pValue;
    *pCurValue = pValue[0];

    if (pAttr->pValue == &sensorPeriod)
    {
      *notifyApp = SENSOR_PERI;
    }
  }
}

#endif

#ifndef IRTEMPSENSORCOUNT_H
#define IRTEMPSENSORCOUNT_H

#include "irtempservice.h"
#include "sensorTag.h"

// Characteristic UUID: readoutPos
static CONST uint8_t sensorCountUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_SENSOR_COUNT_UUID),
};

// gatt table vars
static uint8_t sensorCountProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t sensorCount = 0;

#ifdef USER_DESCRIPTION
static uint8_t sensorCountUserDescr[] = '\0';
#endif

// Characteristic Declaration
#define IRTEMPSENSORCOUNT_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorCountProps \
}

// Characteristic Value "Readout Pos"
#define IRTEMPSENSORCOUNT_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorCountUUID }, \
  GATT_PERMIT_READ | GATT_PERMIT_WRITE, \
  0, \
  &sensorCount \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPSENSORCOUNT_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorCountUserDescr \
}
#endif


/**
 * Called from below from Bluetooth GATT layer.
 */
void irtempSensorCount_WriteAttribute(gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, bStatus_t *status, uint8_t *notifyApp)
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

    SensorTag_updateAdvertDataSensorCount(pCurValue);
  }
}

#endif

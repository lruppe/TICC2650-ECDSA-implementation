#ifndef IRTEMPSTARTTIME_H
#define IRTEMPSTARTTIME_H

#include "irtempservice.h"

#define START_TIME_SIZE 8

// Characteristic UUID: readoutPos
static CONST uint8_t sensorStartTimeUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_START_TIME_UUID),
};

// gatt table vars
static uint8_t sensorStartTimeProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t sensorStartTime[START_TIME_SIZE] = {0,0,0,0,0,0,0,0};

#ifdef USER_DESCRIPTION
static uint8_t sensorStartTimeUserDescr[] = '\0';
#endif

// Characteristic Declaration
#define IRTEMPSTARTTIME_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorStartTimeProps \
}

// Characteristic Value "Readout Pos"
#define IRTEMPSTARTTIME_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorStartTimeUUID }, \
  GATT_PERMIT_READ | GATT_PERMIT_WRITE, \
  0, \
  sensorStartTime \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPSTARTTIME_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorStartTimeUserDescr \
}
#endif


/**
 * Called from below from the Bluetooth GATT layer.
 */
void irtempstarttime_ReadAttribute(uint8_t *pValue, uint16_t *pLen)
{
  *pLen = START_TIME_SIZE;
  memcpy(pValue, sensorStartTime, *pLen);
}

/**
 * Called from below from Bluetooth GATT layer.
 */
void irtempstarttime_WriteAttribute(gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, bStatus_t *status, uint8_t *notifyApp)
{
  // Validate the value
  // Make sure it's not a blob oper
  if (offset == 0)
  {
    if (len != START_TIME_SIZE)
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
    memcpy(pAttr->pValue, pValue, len);
  }
}

#endif

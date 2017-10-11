#ifndef IRTEMPREADOUTPOS_H_
#define IRTEMPREADOUTPOS_H_

#include "irtempservice.h"

// Characteristic UUID: readoutPos
static CONST uint8_t sensorReadoutPosUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_READOUT_POS_UUID),
};

// gatt table vars
static uint8_t sensorReadoutPosProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint32_t sensorReadoutPos = 0;
#ifdef USER_DESCRIPTION
static uint8_t sensorReadoutPosUserDescr[] = '\0';
#endif

// Characteristic Declaration
#define IRTEMPREADOUTPOS_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorReadoutPosProps \
}

// Characteristic Value "Readout Pos"
#define IRTEMPREADOUTPOS_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorReadoutPosUUID }, \
  GATT_PERMIT_READ | GATT_PERMIT_WRITE, \
  0, \
  (uint8_t *)&sensorReadoutPos \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPREADOUTPOS_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorReadoutPosUserDescr \
}
#endif


/**
 * Called from below from the Bluetooth GATT layer.
 */
void irtempreadoutpos_ReadAttribute(uint8_t *pValue, uint16_t *pLen)
{
  *pLen = sizeof(uint32_t);
  *(uint32_t *)pValue = sensorReadoutPos;
}

/**
 * Called from below from Bluetooth GATT layer.
 */
void irtempreadoutpos_WriteAttribute(gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, bStatus_t *status, uint8_t *notifyApp)
{
  // Validate the value
  // Make sure it's not a blob oper
  if (offset == 0)
  {
    if (len != sizeof(uint32_t))
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
    memcpy(pAttr->pValue, pValue, sizeof(uint32_t));
  }
}

#endif

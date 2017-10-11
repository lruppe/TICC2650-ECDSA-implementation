#ifndef IRTEMPCONTRACTID_H
#define IRTEMPCONTRACTID_H

#include "irtempservice.h"
#include "sensorTag.h"

// Characteristic UUID: readoutPos
static CONST uint8_t sensorContractIdPosUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_CONTRACT_ID_UUID),
};

// gatt table vars
static uint8_t sensorContractIdProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8_t sensorContractId[CONTRACT_ID_SIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

#ifdef USER_DESCRIPTION
static uint8_t sensorContractIdUserDescr[] = { 0 };
#endif

// Characteristic Declaration
#define IRTEMPCONTRACTID_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorContractIdProps \
}

// Characteristic Value "Readout Pos"
#define IRTEMPCONTRACTID_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorContractIdPosUUID }, \
  GATT_PERMIT_READ | GATT_PERMIT_WRITE, \
  0, \
  sensorContractId \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPCONTRACTID_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorContractIdUserDescr \
}
#endif


/**
 * Called from below from the Bluetooth GATT layer.
 */
void irtempcontractid_ReadAttribute(uint8_t *pValue, uint16_t *pLen)
{
  *pLen = CONTRACT_ID_SIZE;
  memcpy(pValue, sensorContractId, CONTRACT_ID_SIZE);
}

/**
 * Called from below from Bluetooth GATT layer.
 */
void irtempcontractid_WriteAttribute(gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, bStatus_t *status, uint8_t *notifyApp)
{
  // Validate the value
  // Make sure it's not a blob oper
  if (offset == 0)
  {
    if (len > CONTRACT_ID_SIZE)
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

    SensorTag_updateAdvertDataContractId(sensorContractId, CONTRACT_ID_SIZE);
  }
}

#endif

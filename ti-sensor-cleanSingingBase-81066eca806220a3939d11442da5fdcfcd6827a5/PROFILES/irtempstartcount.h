#ifndef IRTEMPSTARTCOUNT_H
#define IRTEMPSTARTCOUNT_H

#include "irtempservice.h"

// Characteristic UUID: readoutPos
static CONST uint8_t sensorStartCountUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_START_COUNT_UUID),
};

// gatt table vars
static uint8_t sensorStartCountProps = GATT_PROP_READ;
static uint8_t sensorStartCount = 0xff;

#ifdef USER_DESCRIPTION
static uint8_t sensorStartCountUserDescr[] = '\0';
#endif

// Characteristic Declaration
#define IRTEMPSTARTCOUNT_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorStartCountProps \
}

// Characteristic Value "Readout Pos"
#define IRTEMPSTARTCOUNT_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorStartCountUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorStartCount \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPSTARTCOUNT_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorStartCountUserDescr \
}
#endif


/**
 * Called from below from the Bluetooth GATT layer.
 */
void irtempstartcount_ReadAttribute(uint8_t *pValue, uint16_t *pLen)
{
  *pLen = sizeof(uint8_t);
  memcpy(pValue, &sensorStartCount, *pLen);
}

#endif

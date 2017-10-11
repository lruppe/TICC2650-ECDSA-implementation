#ifndef IRTEMPMEASUREMENTCOUNT_H
#define IRTEMPMEASUREMENTCOUNT_H

#include "irtempservice.h"

// Characteristic UUID: measurementCount
static CONST uint8_t sensorMeasurementCountUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_DATA_COUNT_UUID),
};

// gatt table vars
static uint8_t sensorMeasurementCountProps = GATT_PROP_READ;
static uint32_t measurementCount = 0;
#ifdef USER_DESCRIPTION
static uint8_t sensorMeasurementCountUserDescr[] = '\0';
#endif

// Characteristic Declaration
#define IRTEMPMEASUREMENTCOUNT_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorMeasurementCountProps \
}

// Characteristic Value "Measurement Count"
#define IRTEMPMEASUREMENTCOUNT_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorMeasurementCountUUID }, \
  GATT_PERMIT_READ, \
  0, \
  (uint8 *)&measurementCount \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPMEASUREMENTCOUNT_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorMeasurementCountUserDescr \
}
#endif


/**
 * Called from below from the Bluetooth GATT layer.
 */
void irtempmeasurementcount_ReadAttribute(uint8_t *pValue, uint16_t *pLen)
{
  *pLen = sizeof(uint32_t);
  memcpy(pValue, (const void *)&measurementCount, sizeof(uint32_t));
}

#endif

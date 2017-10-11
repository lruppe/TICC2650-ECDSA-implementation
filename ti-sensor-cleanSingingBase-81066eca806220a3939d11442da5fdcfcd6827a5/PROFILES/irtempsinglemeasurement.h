#ifndef IRTEMPSINGLEMEASUREMENT_H
#define IRTEMPSINGLEMEASUREMENT_H

#include "irtempservice.h"
#include "bleUserConfig.h"

// Characteristic UUID: data
static CONST uint8_t sensorSingleMeasurementUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_SINGLE_MEASUREMENT_UUID),
};

// Characteristic Properties: data
static uint8_t sensorSingleMeasurementProps = GATT_PROP_READ;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8_t sensorSingleMeasurementUserDescr[] = '\0';
#endif

// Characteristic Declaration
#define IRTEMPSINGLE_MEASUREMENT_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorSingleMeasurementProps \
}

// Characteristic Value "Data"
#define IRTEMPSINGLE_MEASUREMENT_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorSingleMeasurementUUID }, \
  GATT_PERMIT_READ, \
  0, \
  NULL\
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPSINGLE_MEASUREMENT_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorSingleMeasurementUserDescr \
}
#endif

#endif

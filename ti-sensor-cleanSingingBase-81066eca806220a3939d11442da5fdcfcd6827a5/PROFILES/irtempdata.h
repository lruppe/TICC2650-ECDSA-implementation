#ifndef IRTEMPDATA_H
#define IRTEMPDATA_H

#include "irtempservice.h"
#include "bleUserConfig.h"

// Characteristic UUID: data
static CONST uint8_t sensorDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_DATA_UUID),
};

#define SENSOR_DATA_DESCR       "Temp. Data"

// Characteristic Properties: data
static uint8_t sensorDataProps = GATT_PROP_READ;

// TODO: take out all code that only has to do with notifications
// Characteristic Configuration: data
// static gattCharCfg_t *sensorDataConfig;
static TempMeasurement *readoutMeasurements = NULL;
static uint16_t dataReadConnHandle = 0;
#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8_t sensorDataUserDescr[] = SENSOR_DATA_DESCR;
#endif

// Characteristic Declaration
#define IRTEMPDATA_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorDataProps \
}

// Characteristic Value "Data"
#define IRTEMPDATA_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorDataUUID }, \
  GATT_PERMIT_READ, \
  0, \
  NULL\
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPDATA_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorDataUserDescr \
}
#endif

#endif

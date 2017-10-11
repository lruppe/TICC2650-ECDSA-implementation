#ifndef IRTEMPCONFIG_H
#define IRTEMPCONFIG_H

#include "irtempservice.h"

// Characteristic UUID: config
static CONST uint8_t sensorCfgUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_CONF_UUID),
};

#define SENSOR_CONFIG_DESCR     "Temp. Conf."

// Characteristic Properties: configuration
static uint8_t sensorCfgProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8_t sensorCfg = 0;

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8_t sensorCfgUserDescr[] = SENSOR_CONFIG_DESCR;
#endif

// Characteristic Declaration
#define IRTEMPCONFIG_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorCfgProps \
}

// Characteristic Value "Configuration"
#define IRTEMPCONFIG_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorCfgUUID }, \
  GATT_PERMIT_READ | GATT_PERMIT_WRITE, \
  0, \
  &sensorCfg \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPCONFIG_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorCfgUserDescr \
}
#endif


/**
 * Set parameter called from above from application layer.
 * Mainly used to initialize this value.
 */
void irtempconfig_SetParameter(uint8_t len, void *value, bStatus_t *ret)
{
  if (len == sizeof(uint8_t))
  {
    sensorCfg = *((uint8_t*)value);
  }
  else
  {
    *ret = bleInvalidRange;
  }
}

/**
 * Get parameter called from above from application layer.
 * Depending on what is returned here the SensorTag_Tmp.c task will be activated or deactivated.
 */
void irtempconfig_GetParameter(void *value)
{
  *((uint8_t*)value) = sensorCfg;
}

/**
 * Called from below from Bluetooth GATT layer.
 * Notify application layer about enabling/disabling of service.
 */
void irtempconfig_WriteAttribute(gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, bStatus_t *status, uint8_t *notifyApp)
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
    if (sensorCfg == ST_CFG_SENSOR_ENABLE)
    {
      IRTemp_setMeasurementCount(0);
    }

    if (pAttr->pValue == &sensorCfg)
    {
      *notifyApp = SENSOR_CONF;
    }
  }
}

#endif

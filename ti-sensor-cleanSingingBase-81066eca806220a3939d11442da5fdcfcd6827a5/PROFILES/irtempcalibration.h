#ifndef IRTEMPCALIBRATION_H
#define IRTEMPCALIBRATION_H

#include "irtempservice.h"

// Characteristic UUID: readoutPos
static CONST uint8_t sensorCalibrationUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_CALIBRATION_UUID),
};

// gatt table vars
static uint8_t sensorCalibrationProps = GATT_PROP_READ | GATT_PROP_WRITE;
static double sensorCalibrationCoefficients[CALIBRATION_COEFFICIENT_COUNT] = {1.0, 0.0};

#ifdef USER_DESCRIPTION
static uint8_t sensorCalibrationUserDescr[] = '\0';
#endif

// Characteristic Declaration
#define IRTEMPCALIBRATION_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &sensorCalibrationProps \
}

// Characteristic Value "Readout Pos"
#define IRTEMPCALIBRATION_CHAR_VALUE { \
  { TI_UUID_SIZE, sensorCalibrationUUID }, \
  GATT_PERMIT_READ | GATT_PERMIT_WRITE, \
  0, \
  (uint8_t *)sensorCalibrationCoefficients \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPCALIBRATION_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  sensorCalibrationUserDescr \
}
#endif


/**
 * Called from below from the Bluetooth GATT layer.
 */
void irtempCalibration_ReadAttribute(uint8_t *pValue, uint16_t *pLen)
{
  *pLen = sizeof(double) * CALIBRATION_COEFFICIENT_COUNT;
  memcpy(pValue, sensorCalibrationCoefficients, *pLen);
}

/**
 * Called from below from Bluetooth GATT layer.
 */
void irtempCalibration_WriteAttribute(gattAttribute_t *pAttr, uint8_t *pValue, uint16_t len, uint16_t offset, bStatus_t *status, uint8_t *notifyApp)
{
  // Validate the value
  // Make sure it's not a blob oper
  if (offset == 0)
  {
    if (len != sizeof(double) * CALIBRATION_COEFFICIENT_COUNT)
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
    memcpy(sensorCalibrationCoefficients, pValue, len);
  }
}

#endif

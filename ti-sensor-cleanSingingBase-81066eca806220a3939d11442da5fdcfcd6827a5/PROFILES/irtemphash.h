#ifndef IRTEMPHASH_H
#define IRTEMPHASH_H

#include "irtempservice.h"
#include "sensorTag.h"

// Characteristic UUID: readoutPos
static CONST uint8_t hashUUID[TI_UUID_SIZE] =
{
  TI_UUID(IRTEMPERATURE_HASH_UUID),
};

// gatt table vars
static uint8_t hashProps = GATT_PROP_READ;
static uint8_t hashValue[32];

static uint16_t dataReadHashConnHandle = 0;

#ifdef USER_DESCRIPTION
static uint8_t hashDescr[] = 'hash';
#endif

// Characteristic Declaration
#define IRTEMPHASH_CHAR_DECL { \
  { ATT_BT_UUID_SIZE, characterUUID }, \
  GATT_PERMIT_READ, \
  0, \
  &hashProps \
}

// Characteristic Value "Readout Pos" GATT_PERMIT_AUTHEN_READ
#define IRTEMPHASH_CHAR_VALUE { \
  { TI_UUID_SIZE, hashUUID }, \
  GATT_PERMIT_READ, \
  0, \
  NULL \
}

#ifdef USER_DESCRIPTION
// Characteristic User Description
#define IRTEMPHASH_CHAR_USER_DESC { \
  { ATT_BT_UUID_SIZE, charUserDescUUID }, \
  GATT_PERMIT_READ, \
  0, \
  hashDescr \
}
#endif

#endif

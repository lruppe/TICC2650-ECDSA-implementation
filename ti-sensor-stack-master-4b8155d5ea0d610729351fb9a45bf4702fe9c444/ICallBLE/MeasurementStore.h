#ifndef MEASUREMENT_STORE_H
#define MEASUREMENT_STORE_H

/* Hal Drivers */
#include "hal_types.h"
#include "hal_flash.h"
#include "hal_mcu.h"

#ifdef NV_VOLTAGE_CHECK
#define SNV_CHECK_VOLTAGE()  (PWRMON_check(MIN_VDD_FLASH))
#else
#define SNV_CHECK_VOLTAGE()  (TRUE)
#endif //NV_VOLTAGE_CHECK



static void testWriteFlashHal(uint32_t addr, uint16_t len, uint8_t *data)
{
  HalFlashWrite(addr, data, len);
}

static uint8 *flashGetAddressCustom( uint8 pg, uint16 offset )
{
#if FALSE //#ifndef FEATURE_OAD
  // Calculate the offset into the containing flash bank as it gets mapped into XDATA.
  uint8 *flashAddr = (uint8 *)(offset + HAL_NV_START_ADDR) + ((pg % HAL_NV_PAGE_BEG )* HAL_FLASH_PAGE_SIZE);

  return flashAddr;
#else //FEATURE_OAD
  // The actual address is a 4-KiloByte multiple of the page number plus offset in bytes.
  return (uint8*)((pg << 12) + offset);
#endif //FEATURE_OAD
}

static void HalFlashReadCustom(uint8 pg, uint16 offset, uint8 *buf, uint16 cnt)
{
  halIntState_t cs;

  // Calculate the offset into the containing flash bank as it gets mapped into XDATA.
  uint8 *ptr = flashGetAddressCustom(pg, offset);

  // Enter Critical Section.
  HAL_ENTER_CRITICAL_SECTION(cs);

  // Read data.
  while (cnt--)
  {
    *buf++ = *ptr++;
  }

  // Exit Critical Section.
  HAL_EXIT_CRITICAL_SECTION(cs);
}

static void testReadFlashHal(uint8_t page, uint16_t offset, uint16_t len, uint8_t *data)
{
  HalFlashReadCustom(page, offset, data, len);
}

static void testEraseFlashHal(uint8_t page, uint8 *failFatal, uint8 *failNonfatal)
{
  *failFatal = 0;
  *failNonfatal = 0;

  if (SNV_CHECK_VOLTAGE() == FALSE)
  {
    // Power monitor indicates low voltage
    *failNonfatal = TRUE;
    return;
  }
  else
  {
    halIntState_t cs;
    uint32_t err;

    HAL_ENTER_CRITICAL_SECTION(cs);

    // Erase the page.
    err = FlashSectorErase( (uint32)flashGetAddressCustom(page, 0));

    // Page erase failed, further usage is unsafe.
    if (err != FAPI_STATUS_SUCCESS)
    {
      *failFatal = *failNonfatal = TRUE;
    }

    HAL_EXIT_CRITICAL_SECTION(cs);
  }
}

#endif

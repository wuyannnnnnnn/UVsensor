
#ifdef ENABLE_DEBUG_FLASH_MANAGER
#define ENABLE_DEBUG
#endif

#include "FlashManagerTask.h"
// #include "config.h"
#include "MicroDebug.h"

TaskHandle_t FlashManagerTask::xThisTask = NULL;

bool FlashManagerTask::readPageCache(uint32_t addr, void *buf, uint32_t numOfBytes) {
  if (addr >= cacheAddress && addr + numOfBytes - 1 <= cacheAddress + sizeof(pageCache)) {
    memcpy(buf, pageCache + addr - cacheAddress, numOfBytes);
    return true;
  }
  return false;
}

bool FlashManagerTask::writePageCache(uint32_t addr, const void *buf, uint32_t numOfBytes) {
  if (addr >= cacheAddress && addr + numOfBytes - 1 <= cacheAddress + sizeof(pageCache)) {
    memcpy(pageCache + addr - cacheAddress, buf, numOfBytes);
    isCacheDirty = true;
    return true;
  }
  return false;
}

bool FlashManagerTask::flushPageCache() {
  if (true == isCacheDirty) {
    _Addr = cacheAddress;
    _Range = 256;
    _writeBuf = pageCache;
    isCacheDirty = false;
    state = FlashManagerState_WriteBytes;
    xTaskNotifyGive(xThisTask);
  }
  return true;
}

bool FlashManagerTask::loadPageCache(uint32_t addr) {
  bool result = false;
  Lock();
  while (flash->busy()) {delay(1);}
  result = flash->readBytes(addr, pageCache, 256);
  Unlock();
  return result;
}

bool FlashManagerTask::ReadConfigRegion(uint32_t offset, void *buf, uint32_t numOfBytes) {
  bool result = false;
  if (offset + numOfBytes - 1 <= CONFIG_REGION_SIZE) {
    Lock();
    while (flash->busy()) {delay(1);}
    result = flash->readBytes(CONFIG_REGION_START + offset, buf, numOfBytes);
    Unlock();
  }
  return result;
}

bool FlashManagerTask::WriteConfigRegion(uint32_t offset, const void *buf, uint32_t numOfBytes) {
  Lock();
  while (flash->busy()) {nrf_delay_us(100);}
  flash->writeBytes(CONFIG_REGION_START + offset, buf, numOfBytes);
  Unlock();
  return true;
  // if (offset + numOfBytes - 1 <= CONFIG_REGION_SIZE) {
  //   _Addr = CONFIG_REGION_START + offset;
  //   _Range = numOfBytes;
  //   _writeBuf = buf;

  //   state = FlashManagerState_WriteBytes;
  //   xTaskNotifyGive(xThisTask);
  //   return true;
  // }
  // else {
  //   DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
  //   Unlock();
  //   return false;
  // }
}

bool FlashManagerTask::EraseConfigRegion(uint32_t offset, uint32_t numOfBytes) {
  Lock();
  if (numOfBytes == (numOfBytes>>12)<<12 && offset + numOfBytes - 1 <= CONFIG_REGION_SIZE) {
    _Addr = CONFIG_REGION_START + offset;
    _Range = numOfBytes;
    state = FlashManagerState_EraseBlock;
    xTaskNotifyGive(xThisTask);
    return true;
  }
  else {
    DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
    Unlock();
    return false;
  }
}

bool FlashManagerTask::ReadLifeStyleConfigRegion(uint32_t offset, void *buf, uint16_t numOfBytes) {
  bool result = false;
  if (offset + numOfBytes - 1 <= LIFESTYLE_CONFIG_REGION_SIZE) {
    Lock();
    while (flash->busy()) {delay(1);}
    result = flash->readBytes(LIFESTYLE_CONFIG_REGION_START + offset, buf, numOfBytes);
    Unlock();
  }
  return result;
}

bool FlashManagerTask::ReadLifeStyleHistoryRegion(uint32_t offset, void *buf, uint16_t numOfBytes) 
{
  bool result = false;
  if (offset + numOfBytes - 1 <= LIFESTYLE_HISTORY_REGION_SIZE) {
    Lock();
    while (flash->busy()) {delay(1);}
    result = flash->readBytes(LIFESTYLE_HISTORY_REGION_START + offset, buf, numOfBytes);
    Unlock();
  }
  return result;
}

bool FlashManagerTask::ReadUserLoggingRegion(uint32_t offset, void *buf, uint32_t numOfBytes) {
  bool result = false;
  
  if (offset + numOfBytes - 1 <= USER_LOGGING_REGION_SIZE) {
    Lock();
    while (flash->busy()) {delay(1);}
    result = flash->readBytes(USER_LOGGING_REGION_START + offset, buf, numOfBytes);
    Unlock();
  }
  return result;
}

bool FlashManagerTask::WriteUserLoggingRegion(uint32_t offset, const void *buf, uint32_t numOfBytes) {
  
  Lock();
  while (flash->busy()) {nrf_delay_us(100);}
  flash->writeBytes(USER_LOGGING_REGION_START + offset, buf, numOfBytes);
  Unlock();
  return true;
  // if (offset + numOfBytes - 1 <= USER_LOGGING_REGION_SIZE) {
  //   _Addr = USER_LOGGING_REGION_START + offset;
  //   _Range = numOfBytes;
  //   _writeBuf = buf;
  //   state = FlashManagerState_WriteBytes;
  //   xTaskNotifyGive(xThisTask);
  //   return true;
  // }
  // else {
  //   DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
  //   Unlock();
  //   return false;
  // }
}

bool FlashManagerTask::EraseUserLoggingRegion(uint32_t offset, uint32_t numOfBytes) {
  Lock();
  if (numOfBytes == (numOfBytes>>12)<<12 && offset + numOfBytes - 1 <= USER_LOGGING_REGION_SIZE) {
    _Addr = USER_LOGGING_REGION_START + offset;
    _Range = numOfBytes;  
    state = FlashManagerState_EraseBlock;
    xTaskNotifyGive(xThisTask);
    return true;
  }
  else {
    DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
    Unlock();
    return false;
  }
}

bool FlashManagerTask::ReadHashIndexRegion(uint32_t offset, void *buf, uint32_t numOfBytes) {
  bool result = false;
  
  if (offset + numOfBytes - 1 <= HASH_INDEX_TABLE_REGION_SIZE) {
    Lock();
    while (flash->busy()) {delay(1);}
    result = flash->readBytes(HASH_INDEX_TABLE_REGION_START + offset, buf, numOfBytes);
    Unlock();
  }
  return result;
}

bool FlashManagerTask::WriteHashIndexRegion(uint32_t offset, const void *buf, uint32_t numOfBytes) {
  
  Lock();
  while (flash->busy()) {nrf_delay_us(100);}
  flash->writeBytes(HASH_INDEX_TABLE_REGION_START + offset, buf, numOfBytes);
  Unlock();
  return true;
  // if (offset + numOfBytes - 1 <= HASH_INDEX_TABLE_REGION_SIZE) {
  //   _Addr = HASH_INDEX_TABLE_REGION_START + offset;
  //   _Range = numOfBytes;
  //   _writeBuf = buf;
  //   state = FlashManagerState_WriteBytes;
  //   xTaskNotifyGive(xThisTask);
  //   return true;
  // }
  // else {
  //   DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
  //   Unlock();
  //   return false;
  // }
}

bool FlashManagerTask::EraseHashIndexRegion(uint32_t offset, uint32_t numOfBytes) {
  
  Lock();
  if (numOfBytes == (numOfBytes>>12)<<12 && offset + numOfBytes - 1 <= HASH_INDEX_TABLE_REGION_SIZE) {
    _Addr = HASH_INDEX_TABLE_REGION_START + offset;
    _Range = numOfBytes;
    state = FlashManagerState_EraseBlock;
    xTaskNotifyGive(xThisTask);
    return true;
  }
  else {
    DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
    Unlock();
    return false;
  }
}

bool FlashManagerTask::ReadBarcodeRegion(uint32_t offset, void *buf, uint32_t numOfBytes) {
  bool result = false;
  
  if (offset + numOfBytes - 1 <= BARCODES_DATA_TABLE_REGION_SIZE) {
    Lock();
    while (flash->busy()) {nrf_delay_us(100);}
    result = flash->readBytes(BARCODES_DATA_TABLE_REGION_START + offset, buf, numOfBytes);
    Unlock();
  }
  return result;
}

bool FlashManagerTask::WriteBarcodeRegion(uint32_t offset, const void *buf, uint32_t numOfBytes) {
  
  Lock();
  while (flash->busy()) {nrf_delay_us(100);}
  flash->writeBytes(BARCODES_DATA_TABLE_REGION_START + offset, buf, numOfBytes);
  Unlock();
  return true;
  // if (offset + numOfBytes - 1 <= BARCODES_DATA_TABLE_REGION_SIZE) {
  //   _Addr = BARCODES_DATA_TABLE_REGION_START + offset;
  //   _Range = numOfBytes;
  //   _writeBuf = buf;
  //   state = FlashManagerState_WriteBytes;
  //   xTaskNotifyGive(xThisTask);
  //   return true;
  // }
  // else {
  //   DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
  //   Unlock();
  //   return false;
  // }
}

bool FlashManagerTask::EraseBarcodeRegion(uint32_t offset, uint32_t numOfBytes) {
  Lock();
  if (numOfBytes == (numOfBytes>>12)<<12 && offset + numOfBytes - 1 <= BARCODES_DATA_TABLE_REGION_SIZE) {
    _Addr = BARCODES_DATA_TABLE_REGION_START + offset;
    _Range = numOfBytes;

    state = FlashManagerState_EraseBlock;
    xTaskNotifyGive(xThisTask);
    return true;
  }
  else {
    DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
    Unlock();
    return false;
  }
}

bool FlashManagerTask::EraseHashBarcodeRegion(uint32_t offset, uint32_t numOfBytes) {
  Lock();
  if (numOfBytes == (numOfBytes>>12)<<12 && offset + numOfBytes - 1 <= HASH_INDEX_TABLE_REGION_SIZE + BARCODES_DATA_TABLE_REGION_SIZE) {
    _Addr = HASH_INDEX_TABLE_REGION_START + offset;
    _Range = numOfBytes;

    state = FlashManagerState_EraseBlock;
    xTaskNotifyGive(xThisTask);
    return true;
  }
  else {
    DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
    Unlock();
    return false;
  }
}

FlashManagerTask::FlashManagerTask(SPIFlash *flash, SemaphoreHandle_t *flash_mem_lock)
    : flash(flash), flash_mem_lock(flash_mem_lock) {}

bool FlashManagerTask::Init() { return flash->initialize(); }
void FlashManagerTask::Lock(bool isInISR) {
  DBUGLN("[Flash]Wait for mutex");
  if (isInISR) {
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(*flash_mem_lock, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  else {
    xSemaphoreTake(*flash_mem_lock, portMAX_DELAY);
  }
  DBUGLN("[Flash]mutex obtained");
}

void FlashManagerTask::Unlock(bool isInISR) {
  if (isInISR) {
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(*flash_mem_lock, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  else {
    xSemaphoreGive(*flash_mem_lock);
  }
  DBUGLN("[Flash]mutex returned");
}
// bool FlashManagerTask::getBarcode(Barcode &barcode);
// // bool setBarcode(Barcode barcode);

// // bool getHistoryInfo(uint32_t &length, uint32_t &basetime);
// bool FlashManagerTask::addHistory(Barcode barcode, HistoryState state);
// // bool getHistory(uint32_t index, uint32_t &offset, Barcode &barcode,
// HistoryState &state); bool FlashManagerTask::clearHistory();

// bool FlashManagerTask::getSlot(uint8_t slot, String &id);
// bool FlashManagerTask::setSlot(uint8_t slot, String id);

// bool FlashManagerTask::getTime(uint32_t &time);
// bool FlashManagerTask::setTime(uint32_t time);


void FlashManagerTask::loop() {
  if (NULL == xThisTask) {xThisTask = xTaskGetCurrentTaskHandle();}
  // DBUGF("%s\r\n", __PRETTY_FUNCTION__);
  // DBUGVAR(state);
  switch (state) {
    case FlashManagerState_Idle:
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      break;
    case FlashManagerState_WriteBytes:
      while (flash->busy()) {delay(1);};
      flash->writeBytes(_Addr, _writeBuf, _Range);
      while (flash->busy()) {delay(1);};
      state = FlashManagerState_Idle;
      Unlock();
      break;
    case FlashManagerState_EraseBlock:
      if (false == flash->busy()) {
        // Flash command not issue yet
        if (_Addr == (_Addr >> 16) << 16) {
          // Address is 64k aligned
          if (_Range >= 0x10000) {
            flash->blockErase64K(_Addr);
            _Addr += 0x10000;
            _Range -= 0x10000;
            while (flash->busy()) {delay(FLASH_64KErase_CHECK_INTERVAL);}
          }
          else if (_Range >= 0x8000) {
          // Erase less than 4K
            flash->blockErase32K(_Addr);
            _Addr += 0x8000;
            _Range -= 0x8000;
            while (flash->busy()) {delay(FLASH_32KErase_CHECK_INTERVAL);}
          } 
          else if (_Range >= 0x1000) {
            flash->blockErase4K(_Addr);
            _Addr += 0x1000;
            _Range -= 0x1000;
            while (flash->busy()) {delay(FLASH_4KErase_CHECK_INTERVAL);}
          }
          else {
            if (0 != _Range) {DBUGLN("ERROR! NOT ALL REGION ERASED!");}
            state = FlashManagerState_Idle;
            Unlock();
          }
        }  
        else if (_Addr == (_Addr >> 15) <<15 ) {
          // Address is 32k aligned
          if (_Range >= 0x8000) {
          // Erase less than 4K
            flash->blockErase32K(_Addr);
            _Addr += 0x8000;
            _Range -= 0x8000;
            while (flash->busy()) {delay(FLASH_32KErase_CHECK_INTERVAL);}
          } 
          else if (_Range >= 0x1000) {
            flash->blockErase4K(_Addr);
            _Addr += 0x1000;
            _Range -= 0x1000;
            while (flash->busy()) {delay(FLASH_4KErase_CHECK_INTERVAL);}
          }
          else {
            if (0 != _Range) {DBUGLN("ERROR! NOT ALL REGION ERASED!");}
            state = FlashManagerState_Idle;
            Unlock();
          }
        } 
        else if (_Addr == (_Addr >> 12) <<12 ) {
          // Address is 4k aligned
          if (_Range >= 0x1000) {
            flash->blockErase4K(_Addr);
            _Addr += 0x1000;
            _Range -= 0x1000;
            while (flash->busy()) {delay(FLASH_4KErase_CHECK_INTERVAL);}
          }
          else {
            if (0 != _Range) {DBUGLN("ERROR! NOT ALL REGION ERASED!");}
            state = FlashManagerState_Idle;
            Unlock();
          }
        }
        else {
          // Erase less then 4K. Not allowed
          // Should reject at entry API
          DBUGLN("ERROR! TRYING TO ERASE LESS THAN 4KB!");
        }
      }
      break;
    case FlashManagerState_EraseChip:
      if (false == flash->busy()) {
        flash->chipErase();
      }
      // chip erase takes 140s typically
      while (flash->busy()) {delay(FLASH_ChipErase_CHECK_INTERVAL);}
      state = FlashManagerState_Idle;
      Unlock();
      
      break;
    default:
      break;
  }
}

bool FlashManagerTask::Uninit() { 
  while (state != FlashManagerState_Idle) {
    delay(1);
  }
  while (flash->busy()) {nrf_delay_us(100);}
  Lock();
  return flash->sleep(); 
}
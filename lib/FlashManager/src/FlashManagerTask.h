#ifndef FLASH_MANAGER_TASK_H
#define FLASH_MANAGER_TASK_H

#include <Arduino.h>

#include "SPIFlash.h"

#define FLASH_ADDRESS_WIDTH (int)(25)

////////// START -- Memory Mapping //////////
//Growing order
#define CONFIG_REGION_START 0x00000
#define CONFIG_REGION_SIZE (0x1000)

#define MPU_CONFIG_ADDRESS_START (0x0000)

#define LIFESTYLE_CONFIG_REGION_START (0x50)
#define LIFESTYLE_CONFIG_REGION_SIZE (0xB0)

#define LIFESTYLE_HISTORY_REGION_START (LIFESTYLE_CONFIG_REGION_START + LIFESTYLE_CONFIG_REGION_SIZE)
#define LIFESTYLE_HISTORY_REGION_SIZE (0x300)

#define DEVICE_CONFIG_ADDRESS_START (0x0400)

#define USER_LOGGING_REGION_START CONFIG_REGION_SIZE
#define USER_LOGGING_REGION_SIZE (0x8000)

#ifdef BLE_BAND_MATCHING
#define BAND_MATCHING_DNA_DATA_REGION_START        0x0800
#define BAND_MATCHING_DNA_DATA_REGION_SIZE         0x100
#define BAND_MATCHING_HISTORY_REGION_START  (BAND_MATCHING_DNA_DATA_REGION_START+BAND_MATCHING_DNA_DATA_REGION_SIZE) 
#define BAND_MATCIHNG_HISTORY_REGION_SIZE   (CONFIG_REGION_SIZE - BAND_MATCHING_HISTORY_REGION_START)
// For V1 NudgeMatch Compatibility#
#define BAND_DNA_DATA_VALID_SIZE           1 //! Value 1 - Set, Value != 1  - Not Set
#define BAND_DNA_DATA_SIZE                 8  //! 16 * 4 = 64 bits = 8 bytes

#endif //BLE_BAND_MATCHING

#define HASH_INDEX_TABLE_REGION_START (USER_LOGGING_REGION_START + USER_LOGGING_REGION_SIZE)
#define HASH_INDEX_TABLE_REGION_SIZE (0x40000)

#define BARCODES_DATA_TABLE_REGION_START (HASH_INDEX_TABLE_REGION_START + HASH_INDEX_TABLE_REGION_SIZE)
#define BARCODES_DATA_TABLE_REGION_SIZE (0x2000000 - BARCODES_DATA_TABLE_REGION_START)

////////// END -- Memory Mapping //////////

#define FLASH_BLOCK_SIZE 0x1000 // 4 kByte block size
#define FLASH_4KErase_CHECK_INTERVAL 50 // check every 50 ms. Erase 4kB sector typcially needs 50ms according to DS
#define FLASH_32KErase_CHECK_INTERVAL 50//190 // check every 190 ms. Erase 32kB sector typcially needs 190ms according to DS
#define FLASH_64KErase_CHECK_INTERVAL 50//270 // check every 270 ms. Erase 64kB sector typcially needs 270ms according to DS
#define FLASH_ChipErase_CHECK_INTERVAL 140*1000 // check every 140s. Erase whole chip typcially needs 140s according to DS

// enum FLASH_ERASE_SIZE_T { SIZE_4K = 1, SIZE_32K = 8, SIZE_64K = 16 };

// enum FLASH_REGION_T {
//   MPU_CONFIG_REGION = MPU_CONFIG_ADDRESS_START,
//   LIFESTYLE_CONFIG_REGION = LIFESTYLE_CONFIG_REGION_START,
//   LIFESTYLE_HISTORY_REGION = LIFESTYLE_HISTORY_REGION_START,
//   DEVICE_CONFIG_REGION = DEVICE_CONFIG_ADDRESS_START,
//   BAND_MATCHING_REGION = BAND_MATCHING_DNA_DATA_REGION_START,
//   USER_LOGGING_REGION = USER_LOGGING_REGION_START,
//   HASH_INDEX_TABLE_REGION = HASH_INDEX_TABLE_REGION_START,
//   BARCODES_DATA_TABLE_REGION = BARCODES_DATA_TABLE_REGION_START
// };

extern SPIClass SPI;

#define DEVICE_NAME_SIZE                   11 //! Prefix - 5, iphen - 1, BandID - 5

#ifdef BLE_BAND_MATCHING

#define BAND_DNA_TRAITS_MAX_GROUP_COUNT    8  // Support maximum 8 DNA groups as there is only a 3-bit field for group index

#define BAND_DNA_DATA_NOT_SET_IN_FLASH    0xFF
#define BAND_DNA_DATA_SET_IN_FLASH        0xFE

#define MAX_FLASH_WRITE_BYTES             256

//! Structure to Store Bandmatching Status
typedef struct __attribute__((__packed__))
{
  uint32_t time;
  char peerBandName[DEVICE_NAME_SIZE+1];
  uint8_t matchStatus[BAND_DNA_TRAITS_MAX_GROUP_COUNT];
}TSBandMatchStatus;

#define MAX_BANDMATCHING_RECORDS           (BAND_MATCIHNG_HISTORY_REGION_SIZE / sizeof(TSBandMatchStatus))

//! Structure to Store Bandmatching History
typedef struct __attribute__((__packed__))
{
 uint8_t u8NumOfRecards;
 TSBandMatchStatus strBandMatchStatus[MAX_BANDMATCHING_RECORDS];
}TSBandMatchHistory;

//! Structure to Store Bandmatching History and DNA Data in Flash
typedef struct __attribute__((__packed__))
{
  //! A Flag To Store Whether DNA Data has been set or not
  uint8_t dnaDataSetInFlash;
  uint8_t autoSleepTime;
  int8_t rssiThreshold;
  uint8_t bandDnaDataSize;
  uint8_t bandDnaData[BAND_MATCHING_DNA_DATA_REGION_SIZE];
  TSBandMatchHistory strBandMatchHistory;
}TSBandMatchFlashTable;

#endif //BLE_BAND_MATCHING


enum FlashManagerState { 
  FlashManagerState_Idle, 
  FlashManagerState_WriteBytes, 
  FlashManagerState_EraseBlock, 
  FlashManagerState_EraseChip
  };

class FlashManagerTask {
private:
  SPIFlash *flash;

  uint32_t _Addr;
  uint32_t _Range;
  const void *_writeBuf;
  FlashManagerState state;
  // FLASH_REGION_T FLASH_REGION;
  // bool WriteBarcodeToHashTableBucket(Barcode barcode);
  // bool WriteBarcodeToHashTable(Barcode barcode);
  // uint32_t getBarcodeHashTableAddress(Barcode barcode);
  // uint32_t getBarcodeNumber(Barcode barcode);
  // bool VerifyHashTableWrite();
  // Barcode * ReadHashTableBucket(Barcode barcode);
  // Barcode * ProcessHashTableBucket();
  SemaphoreHandle_t *flash_mem_lock;
  static TaskHandle_t xThisTask;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint8_t pageCache[256];
  static uint32_t cacheAddress;
  static bool isCacheDirty;
public:
  FlashManagerTask(SPIFlash *flash, SemaphoreHandle_t *flash_mem_lock);
  bool readPageCache(uint32_t addr, void *buf, uint32_t numOfBytes);
  bool writePageCache(uint32_t addr, const void *buf, uint32_t numOfBytes);
  bool flushPageCache();
  bool loadPageCache(uint32_t addr);

  // bool ReadHashTableBucketHeader(uint32_t offset, void *buf, uint8_t
  // numOfBytes); bool WriteHashTableBucketHeader(uint32_t offset, const void
  // *buf, uint8_t numOfBytes); bool setState(FlashManagerState newState);
  bool ReadConfigRegion(uint32_t offset, void *buf, uint32_t numOfBytes);
  bool WriteConfigRegion(uint32_t offset, const void *buf, uint32_t numOfBytes);
  bool EraseConfigRegion(uint32_t offset, uint32_t numOfBytes);

  bool ReadLifeStyleConfigRegion(uint32_t Address, void *buf, uint16_t numOfBytes);
  bool ReadLifeStyleHistoryRegion(uint32_t Address, void *buf, uint16_t numOfBytes);

  bool ReadUserLoggingRegion(uint32_t offset, void *buf, uint32_t numOfBytes);
  bool WriteUserLoggingRegion(uint32_t offset, const void *buf, uint32_t numOfBytes);
  bool EraseUserLoggingRegion(uint32_t offset, uint32_t numOfBytes);

  bool ReadHashIndexRegion(uint32_t offset, void *buf, uint32_t numOfBytes);
  bool WriteHashIndexRegion(uint32_t offset, const void *buf, uint32_t numOfBytes);
  bool EraseHashIndexRegion(uint32_t offset, uint32_t numOfBytes);

  bool ReadBarcodeRegion(uint32_t offset, void *buf, uint32_t numOfBytes);
  bool WriteBarcodeRegion(uint32_t offset, const void *buf, uint32_t numOfBytes);
  bool EraseBarcodeRegion(uint32_t offset, uint32_t numOfBytes);
  bool EraseHashBarcodeRegion(uint32_t offset, uint32_t numOfBytes);

  bool isFlashReady() {return FlashManagerState_Idle == state; };
  bool Init();
  bool Uninit();
  void Lock(bool isFromISR = false);
  void Unlock(bool isFromISR = false);

  void setup();
  void loop();
  // bool getBarcode(Barcode &barcode);
  // // bool setBarcode(Barcode barcode);

  // // bool getHistoryInfo(uint32_t &length, uint32_t &basetime);
  // bool addHistory(Barcode barcode, HistoryState state);
  // // bool getHistory(uint32_t index, uint32_t &offset, Barcode &barcode,
  // HistoryState &state); bool clearHistory();

  // bool getSlot(uint8_t slot, String &id);
  // bool setSlot(uint8_t slot, String id);

  // bool getTime(uint32_t &time);
  // bool setTime(uint32_t time);
};

#endif // !FLASH_MANAGER_TASK_H
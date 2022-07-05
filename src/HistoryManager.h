#ifndef HISTORY_MANAGER_H
#define HISTORY_MANAGER_H
#include "Barcode.h"
#include "FlashManagerTask.h"
#include "config.h"
#include <Arduino.h>

enum HistoryState : uint8_t {
  HistoryState_Buy,
  HistoryState_NotBuy,
  HistoryState_NotSet,
  HistoryState_Unknown,
  HistoryState_AmberBuy,
  HistoryState_AmberNotBuy
};

struct History {
  //Barcode barcode;
  uint64_t histBarcode;
  uint32_t time;
  HistoryState buy;
  uint8_t userIndexes;
  uint16_t reserved;
  //uint8_t reserved[(
  //    16 - ((sizeof(uint32_t)) + sizeof(Barcode) + sizeof(HistoryState)))];
} __attribute__((packed));

struct HistoryFlashHeader {
  uint16_t blockMap;
  uint16_t subBlocks[sizeof(uint8_t) * 8];
};

enum HistoryIndexSelect { HISTORY_HEAD, HISTORY_TAIL, HISTORY_SYNC };

#define SIZEOF_FULL_HISTORY_FLASH_HEADER                                       \
  int(((sizeof(HistoryFlashHeader) + 8 * (2 * 16)) >> 3)                       \
      << 3) // make it a 8 byte boundary
#define START_ADDRESS_OF_HISTORY_DATA                                          \
  int(3 * SIZEOF_FULL_HISTORY_FLASH_HEADER) // start after 3xfull headers for
                                            // head, tail and sync
// #define START_ADDRESS_OF_HISTORY_DATA (int)0x400
#define SIZEOF_HISTORY_DATA USER_LOGGING_REGION_SIZE

#define NUM_IN_FLASH_HISTORY                                                   \
  int(((0x1 << 15) - START_ADDRESS_OF_HISTORY_DATA) /                          \
      sizeof(History)) // number of history entries of 16 bytes each
#define NUM_IN_BLOCK                                                           \
  int((0x1 << 12) >> 4) // number of history entries of 16 bytes each
#define NUM_OF_BITS_IN_BLOCK int(8) // number of bits in a block = 8
#define NUM_IN_SUB_BLOCK                                                       \
  int((0x1 << 8) /                                                             \
      sizeof(History)) // number of history entries of 16 bytes each
#define NUM_OF_BITS_IN_SUBBLOCK int(4) // number of bits in a subblock = 4

enum HistoryManagerState {
  HistoryManagerState_Idle,
  HistoryManagerState_eraseHistory
};

class HistoryManager {
public:
  HistoryManager(FlashManagerTask *flashManager);
  bool getUnSyncedInfo(uint32_t &length, uint32_t &baseTime);
  bool writeHistory(const History history, const uint16_t historyHeader);
  bool readHistory(History &history, const uint16_t historyHeader);
  bool getHistoryHeaderTail(uint16_t &historyHeader, uint16_t &historyTail);
  bool setHistoryHeaderTail(const uint16_t historyHeader,
                            const uint16_t historyTail);
  bool getHistorySyncLocation(uint16_t &historySyncLoc);
  bool setHistorySyncLocation(const uint16_t historySyncLoc);
  bool eraseHistory();
  bool eraseHistoryBlock(uint8_t blkIdx);

private:
  FlashManagerTask *flashManager;
  int erase_itration;
  uint8_t blockNum;
  uint8_t subBlock;
  uint8_t subBlockIndex;
  uint16_t historyPtr;
  bool readHistoryHeadInfo(HistoryIndexSelect historyIndexSelect);
  bool writeHistoryHeadInfo(const HistoryFlashHeader *historyHeader,
                            HistoryIndexSelect historyIndexSelect);
  bool writeHistoryPosition(const uint16_t HistoryPosition,
                            const HistoryIndexSelect historyIndexSelect);
  bool convertHistoryPosition(const uint16_t HistoryPosition);
  bool incHistoryPtr();
  bool incHistorySyncPtr();
  uint8_t getCount(uint16_t EntryCountMap);
  HistoryManagerState state;
};
#endif // !HISTORY_MANAGER_H
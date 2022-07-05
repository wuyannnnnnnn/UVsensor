
#ifndef ENABLE_DEBUG_HISTORY_MANAGER
#undef ENABLE_DEBUG
#endif

#include "HistoryManager.h"
#include "debug.h"

HistoryManager::HistoryManager(FlashManagerTask *flashManager)
    : flashManager(flashManager) {}

bool HistoryManager::getUnSyncedInfo(uint32_t &length, uint32_t &baseTime) {
  return true;
}

bool HistoryManager::writeHistory(const History history,
                                  const uint16_t historyHeader) {
  uint32_t Address;
  Address = START_ADDRESS_OF_HISTORY_DATA + historyHeader * (sizeof(History));

  bool result =
      flashManager->WriteUserLoggingRegion(Address, &history, sizeof(History));
  return result;
}

bool HistoryManager::readHistory(History &history,
                                 const uint16_t historyHeader) {
  uint32_t Address;
  Address = START_ADDRESS_OF_HISTORY_DATA + historyHeader * (sizeof(History));
  bool result =
      flashManager->ReadUserLoggingRegion(Address, &history, sizeof(History));
  return result;
}

bool HistoryManager::incHistoryPtr() { return true; }

bool HistoryManager::readHistoryHeadInfo(
    HistoryIndexSelect historyIndexSelect) {
  bool result;
  uint32_t Address;
  uint16_t entryMap;
  uint8_t numOfBytes = sizeof(HistoryFlashHeader);
  HistoryFlashHeader historyHeader;
  switch (historyIndexSelect) {
  case HISTORY_HEAD:
    Address = 0;
    break;
  case HISTORY_TAIL:
    Address = SIZEOF_FULL_HISTORY_FLASH_HEADER;
    break;
  default:
    Address = 2 * SIZEOF_FULL_HISTORY_FLASH_HEADER;
    break;
  }
  result =
      flashManager->ReadUserLoggingRegion(Address, &historyHeader, numOfBytes);
  blockNum = getCount(historyHeader.blockMap);
  if (blockNum > 0) {
    subBlock = getCount(historyHeader.subBlocks[blockNum - 1]);
    Address += sizeof(HistoryFlashHeader) +
               (blockNum - 1) * (16 * sizeof(uint16_t)) +
               (subBlock - 1) * sizeof(uint16_t);
    result = flashManager->ReadUserLoggingRegion(Address, &entryMap,
                                                sizeof(uint16_t));
    subBlockIndex = getCount(entryMap);
    // if blockNum == 0 then this is the first time
    uint8_t sub = (subBlock > 1) || (blockNum > 1) ? 1 : 0;
    historyPtr = ((uint16_t)(blockNum - 1) * (uint16_t)NUM_IN_BLOCK +
                  NUM_IN_SUB_BLOCK * (uint16_t)(subBlock - 1) +
                  (uint16_t)(subBlockIndex - sub));
  } else {
    // if blockNum == 0 then this is the first time
    historyPtr = 0;
  }
  return result;
}

bool HistoryManager::writeHistoryHeadInfo(
    const HistoryFlashHeader *historyHeader,
    HistoryIndexSelect historyIndexSelect) {
  bool result;
  uint32_t Address;
  uint8_t numOfBytes = sizeof(HistoryFlashHeader);
  switch (historyIndexSelect) {
  case HISTORY_HEAD:
    Address = 0;
    break;
  case HISTORY_TAIL:
    Address = SIZEOF_FULL_HISTORY_FLASH_HEADER;
    break;
  default:
    Address = 2 * SIZEOF_FULL_HISTORY_FLASH_HEADER;
    break;
  }
  result =
      flashManager->WriteUserLoggingRegion(Address, historyHeader, numOfBytes);
  return result;
}

bool HistoryManager::writeHistoryPosition(
    const uint16_t HistoryPosition,
    const HistoryIndexSelect historyIndexSelect) {
  bool result;
  if (HistoryPosition == 0)
    return true; // nothing to do if it is 0
  uint32_t Address;
  // uint8_t numOfBytes = sizeof(HistoryFlashHeader);
  switch (historyIndexSelect) {
  case HISTORY_HEAD:
    Address = 0;
    break;
  case HISTORY_TAIL:
    Address = SIZEOF_FULL_HISTORY_FLASH_HEADER;
    break;
  default:
    Address = 2 * SIZEOF_FULL_HISTORY_FLASH_HEADER;
    break;
  }
  convertHistoryPosition(HistoryPosition);
  // write the blockMap uint8_t
  uint16_t blockMap = (0xFFFF << blockNum);
  result =
      flashManager->WriteUserLoggingRegion(Address, &blockMap, sizeof(uint16_t));
  // write the SubblockMap uint16_t
  uint16_t subBlockMap = (0xFFFF << subBlock);
  uint32_t Address2 =
      Address + (sizeof(uint16_t) + sizeof(uint16_t) * (blockNum - 1));
  result = flashManager->WriteUserLoggingRegion(Address2, &subBlockMap,
                                               sizeof(uint16_t));
  // write the SubBlockIndexMap uint16_t
  uint16_t subBlockIndexMap = (0xFFFF << subBlockIndex);
  Address +=
      (sizeof(HistoryFlashHeader) + sizeof(uint16_t) * (blockNum - 1) * 16 +
       sizeof(uint16_t) * (subBlock - 1));
  result = flashManager->WriteUserLoggingRegion(Address, &subBlockIndexMap,
                                               sizeof(uint16_t));
  return result;
}

bool HistoryManager::convertHistoryPosition(const uint16_t HistoryPosition) {
  uint8_t adder = HistoryPosition > 0 ? 1 : 0;
  blockNum = (HistoryPosition >> NUM_OF_BITS_IN_BLOCK) + adder;
  subBlock =
      ((HistoryPosition % NUM_IN_BLOCK) >> NUM_OF_BITS_IN_SUBBLOCK) + adder;
  uint8_t adder2 = subBlock > 1 || blockNum > 1 ? 1 : 0;
  subBlockIndex = HistoryPosition % NUM_IN_SUB_BLOCK + adder2;
  ;
  return true;
}

bool HistoryManager::eraseHistory() {

  if (false == flashManager->EraseUserLoggingRegion(0, USER_LOGGING_REGION_SIZE)) {
    DBUGF("%s failed! \r\n", __PRETTY_FUNCTION__);
  }
  else {
    DBUGLN("Erase all shopping history done.");
  }

  return true;
}

bool HistoryManager::eraseHistoryBlock(uint8_t blkIdx) {

  flashManager->EraseUserLoggingRegion(blkIdx * 0x1000, 0x1000);
  DBUGF("Erase 4k History block %d.", blkIdx);
  return true;
}

uint8_t HistoryManager::getCount(uint16_t EntryCountMap) {
  uint8_t entryCnt = 0;
  for (uint8_t i = 0; i < NUM_IN_SUB_BLOCK; i++) {
    if ((EntryCountMap & (0x1 << i)) == 0) {
      entryCnt = i + 1;
    } else {
      entryCnt = i;
      break;
    }
  }
  return entryCnt;
}

bool HistoryManager::getHistoryHeaderTail(uint16_t &historyHeader,
                                          uint16_t &historyTail) {
  if (readHistoryHeadInfo(HISTORY_HEAD) == true) {
    historyHeader = historyPtr;
    if (readHistoryHeadInfo(HISTORY_TAIL) == true) {
      historyTail = historyPtr;
      return true;
    }
  }
  return false;
}

bool HistoryManager::setHistoryHeaderTail(const uint16_t historyHeader,
                                          const uint16_t historyTail) {
  if (writeHistoryPosition(historyHeader, HISTORY_HEAD) == true) {
    if (writeHistoryPosition(historyTail, HISTORY_TAIL) == true) {
      return true;
    }
  }
  return false;
}

bool HistoryManager::getHistorySyncLocation(uint16_t &historySyncLoc) {
  if (readHistoryHeadInfo(HISTORY_SYNC) == true) {
    historySyncLoc = historyPtr;
    return true;
  }
  return false;
}

bool HistoryManager::setHistorySyncLocation(const uint16_t historySyncLoc) {
  if (writeHistoryPosition(historySyncLoc, HISTORY_SYNC) == true) {
    return true;
  }
  return false;
}

#ifndef ENABLE_DEBUG_DATA_STORE
#undef ENABLE_DEBUG
#endif

#include "DataStore.h"
#include "config.h"
#include "debug.h"

#ifdef ENABLE_TEST_DATA
#include "TestBarcodes.h"
#endif

uint16_t historyHead = 0;
uint16_t historyTail = 0;

int writeHistoryFailCount = 0;
int readHistoryFailCount = 0;

#define HISTORY_NEXT_SLOT(i) (((i) + 1) % NUM_IN_FLASH_HISTORY)
#define HISTORY_CURRENT_4K_BLOCK(i) (((i * sizeof(History))+START_ADDRESS_OF_HISTORY_DATA) / 0x1000)
#define HISTORY_NEXT_4K_BOUNDRAY(i) (((HISTORY_CURRENT_4K_BLOCK(i) + 1) * 0x1000) % SIZEOF_HISTORY_DATA)
#define HISTORY_NEXT_4K_START_SLOT(i) (((0 == HISTORY_NEXT_4K_BOUNDRAY(i)?START_ADDRESS_OF_HISTORY_DATA:HISTORY_NEXT_4K_BOUNDRAY(i)) - START_ADDRESS_OF_HISTORY_DATA) / sizeof(History))

#ifndef ENABLE_EXT_RTC
  __attribute__((section(".noinit"))) static uint32_t secondsTime = 0;
  __attribute__((section(".noinit"))) static uint32_t secondsTimeSet = millis();
#endif

// static String slotIds[NUMBER_SLOTS];

DataStore::DataStore(BarcodeManagerTask *barcodeManager,
                     HistoryManager *historyManager)
    : barcodes(barcodeManager), historyManager(historyManager) {}

DataStore::DataStore(Barcode *barcode, HistoryManager *historyManager, DeviceManager *deviceManager, ExtRTCTask *rtc)
    : barcode(barcode), historyManager(historyManager), deviceManager(deviceManager), rtc(rtc) {}

bool DataStore::getBarcode(Barcode &barcode) {
#ifdef ENABLE_TEST_DATA

  for (int i = 0; i < NUMBER_CODES; i++) {
    if (barcodes[i] == barcode) {
      barcode.setRecommendation(barcodes[i]);
      return true;
    }
  }

  return false;

#else

  return false; // barcode.getBarcode(scanBarcode);

#endif
}

bool DataStore::setBarcode(Barcode barcode) {
#ifdef ENABLE_TEST_DATA

  // Update an existing code
  for (int i = 0; i < NUMBER_CODES; i++) {
    if (barcodes[i] == barcode) {
      barcodes[i].setRecommendation(barcode);
      return true;
    }
  }

  // Not in out data find an empty slot
  for (int i = 0; i < NUMBER_CODES; i++) {
    if (!barcodes[i]) {
      barcodes[i] = barcode;
      return true;
    }
  }

  return false;

#else

  return true; // barcodes->setBarcode(barcode);

#endif
}

bool DataStore::getHistoryInfo(uint32_t &length, uint32_t &baseTime) {
  History tmpHistory;
  if (historyManager->getHistoryHeaderTail(historyHead, historyTail)) {
    length = historyHead >= historyTail
              ? historyHead - historyTail
              : (NUM_IN_FLASH_HISTORY - historyTail) + historyHead;
    if (historyManager->readHistory(tmpHistory, historyTail)) {
      if (length > 0) {
        baseTime = tmpHistory.time;
      } else {
        getTime(baseTime);
      }
    }
    else {
      getTime(baseTime);
    }
  }
  else {
    length = 0;
    getTime(baseTime);
  }

  return true;
}

bool DataStore::addHistory(Barcode barcode, HistoryState state, bool familyScanEnabled) {

  // DBUGVAR(barcode.toString().c_str());
  uint32_t time;
  getTime(time);
  // DBUGVAR(time);

  bool result;
  History tmpHistory;
  result = historyManager->getHistoryHeaderTail(historyHead, historyTail);
  if (result == false) {
    readHistoryFailCount++;
  }
  if (historyHead >= 255) {
    DBUGVAR(historyHead);
  }
  tmpHistory.time = time;
  tmpHistory.histBarcode = barcode.getCode();
  tmpHistory.buy = state;
  tmpHistory.userIndexes = familyScanEnabled ? deviceManager->getEnabledIndexes() : 1;
  DBUGF("tmpHistory.userIndexes = 0x%x\r", tmpHistory.userIndexes);
  result = historyManager->writeHistory(tmpHistory, historyHead);
  if (result == false) {
    writeHistoryFailCount++;
  }

  historyHead = HISTORY_NEXT_SLOT(historyHead);
  if (historyHead == historyTail) {
    DBUGLN("HISTORY REGION FULL");
    historyManager->eraseHistoryBlock(HISTORY_CURRENT_4K_BLOCK(historyHead));
    historyTail = HISTORY_NEXT_4K_START_SLOT(historyTail);
    if (0 == historyTail) {
      // circular buffer full
      historyManager->eraseHistoryBlock(HISTORY_CURRENT_4K_BLOCK(historyTail));
    }
    DBUGVAR(historyTail);
  }
  result = historyManager->setHistoryHeaderTail(historyHead, historyTail);
  if (result == false) {
    writeHistoryFailCount++;
  }

  return true;
}

bool DataStore::getHistory(uint32_t index, uint32_t &offset, Barcode &barcode,
                           HistoryState &state, uint8_t &userIndexes) {
  uint32_t length;
  uint32_t baseTime;

  getHistoryInfo(length, baseTime);

  DBUGVAR(length);

  if (index < length) {
    uint32_t ptr = (historyTail + index) % NUM_IN_FLASH_HISTORY;
    DBUGVAR(ptr);
    bool result;
    History tmpHistory;
    result = historyManager->readHistory(tmpHistory, ptr);
    offset = tmpHistory.time - baseTime;
    barcode.setCode(tmpHistory.histBarcode);
    state = tmpHistory.buy;
    userIndexes = tmpHistory.userIndexes;
    DBUGVAR(offset);
    DBUGVAR(barcode.toString().c_str());
    DBUGVAR(state);

    return true;
  }

  return false;
}

bool DataStore::clearHistory() {
  historyManager->eraseHistory();
  return true;
}

// bool DataStore::getSlot(uint8_t slot, String &id) {
//   if (slot < NUMBER_SLOTS) {
//     id = slotIds[slot];
//     return true;
//   }

//   return false;
// }

// bool DataStore::setSlot(uint8_t slot, String id) {
//   if (slot < NUMBER_SLOTS) {
//     slotIds[slot] = id;
//     return true;
//   }

//   return false;
// }

bool DataStore::getTime(uint32_t &time) {
  #ifdef ENABLE_EXT_RTC
    time = (rtc->getCurrentTime()).unixtime();
  #else
  uint32_t ellapsed = millis();
  if (ellapsed > secondsTimeSet) { // we only subtract if result is not -ve
    ellapsed -= secondsTimeSet;
  }
  time = secondsTime + (ellapsed / 1000);
  #endif
  return true; 
}

bool DataStore::setTime(uint32_t time) {
  #ifdef ENABLE_EXT_RTC
    rtc->setCurrentTime(time);
    rtc->printTime(rtc->getCurrentTime());
  #else
    secondsTime = time;
    secondsTimeSet = millis();
  #endif

  return true;
}

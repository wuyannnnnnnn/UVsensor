#ifndef DATA_STORE_H
#define DATA_STORE_H

#include <Arduino.h>

#include "Barcode.h"
#include "BarcodeManagerTask.h"
#include "HistoryManager.h"
#include "DeviceManager.h"
#ifdef ENABLE_EXT_RTC
  #include "ExtRTCTask.h"
#endif

class DataStore
{
  private:
    BarcodeManagerTask *barcodes;
    Barcode *barcode;
    HistoryManager *historyManager;
    DeviceManager *deviceManager;
    ExtRTCTask *rtc;

  public:
    DataStore(BarcodeManagerTask *barcodes, HistoryManager *historyManager);
    DataStore(Barcode *barcode, HistoryManager *historyManager, DeviceManager *deviceManager, ExtRTCTask *rtc);

    bool getBarcode(Barcode &barcode);
    bool setBarcode(Barcode barcode);

    bool getHistoryInfo(uint32_t &length, uint32_t &basetime);
    bool addHistory(Barcode barcode, HistoryState state, bool familyScanEnabled);
    bool getHistory(uint32_t index, uint32_t &offset, Barcode &barcode, HistoryState &state, uint8_t &userIndexes);
    bool clearHistory();

    bool getSlot(uint8_t slot, String &id);
    bool setSlot(uint8_t slot, String id);

    bool getTime(uint32_t &time);
    bool setTime(uint32_t time);
};

#endif // !DATA_STORE_H

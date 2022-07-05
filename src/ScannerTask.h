#ifndef SCANNER_TASK_H
#define SCANNER_TASK_H

#include "DeviceManager.h"
#include "DataStore.h"
#include "LedManagerTask.h"
#include "LifeStyle.h"

#ifndef SERIAL_SPEED
#define SERIAL_SPEED 115200
#endif

#undef min
#undef max
#include <functional>
enum ScannerState
{
  ScannerState_TriggerByButton = 1,
  ScannerState_TriggerByGesture,
  ScannerState_Decoded
};

typedef std::function<void(void)> detected_scanning_callback_t;

class UARTBoxComTask;

class ScannerTask
{
  friend UARTBoxComTask; 
  private:
    DeviceManager *deviceManager;
    BarcodeManagerTask *barcodeManager;
    DataStore *dataStore;
    LifeStyle *lifeStyle;
    LedManagerTask *led;
    TimerHandle_t *sleepAlarm;

    bool forceTrigger;
    bool isLifeStyle;
    bool isPenaltied;
    Barcode lastCode;
    uint32_t timeout;
    detected_scanning_callback_t detected_scanning_callback;
    bool scanner_flag;
    UARTBoxComTask *uartBoxCom;
    uint8_t allergySlotIndex;
    bool isScanning;

    void Pwr_pinMode( uint32_t ulPin, uint32_t ulMode );
    static TaskHandle_t xThisTask;
    static void vDecode_ISR();
  public:
    ScannerTask(DeviceManager *deviceManager, DataStore *dataStore, LedManagerTask *led,  BarcodeManagerTask *barcodeManager, 
                LifeStyle *lifeStyle, TimerHandle_t *sleepAlarm);

    void setup();
    void loop();

    void trigger(bool triggerByButton=false);

    void setScanningCallback(detected_scanning_callback_t callback) {
      detected_scanning_callback = callback;
    }

    void updateAllergySlotIndex() {
      allergySlotIndex = deviceManager->getAllergySlotIndex(); 
    }

};

#endif //  SCANNER_TASK_H

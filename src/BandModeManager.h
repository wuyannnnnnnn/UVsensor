#ifndef BAND_MODE_MANAGER_H
#define BAND_MODE_MANAGER_H

#include "LedManagerTask.h"
#include "MpuManagerTask.h"
#include "BleManagerTask.h"
#include "LifeStyle.h"
#include "BatteryMonitor.h"

enum BandMode { Mode_Shopping, Mode_Idel, Mode_MAX };

class BandModeManager {
  private:
    LedManagerTask *led;
    ScannerTask *scanner;
    MpuManagerTask *mpuManager;
    TimerHandle_t *sleepAlarm;
    LifeStyle *lifeStyle;
    BleManagerTask *bleManager;
    BatteryMonitor *battery;
    FlashManagerTask *flashManager;
    
    BandMode mode;
    uint32_t buttonPressedTime;
    uint32_t buttonPressedDuration;
    bool longPress;
    bool goToSleep;
    bool deepSleepReady;
    bool externalSleep;

    void enable();
    static SoftwareTimer btnTimer;
    static TaskHandle_t xThisTask;
    static void vButton_ISR();
    static void vBtnTimer_ISR(TimerHandle_t xTimerID);
  public:
    BandModeManager(LedManagerTask *led, ScannerTask *scanner, MpuManagerTask *mpuManager, 
      TimerHandle_t *sleepAlarm, LifeStyle *lifeStyle, BleManagerTask *bleManager, BatteryMonitor *battery, FlashManagerTask *flashManager);
    void setup();
    void loop();
    void sleep();
    void spi_pulldown();

};

#endif // BAND_MODE_MANAGER_H

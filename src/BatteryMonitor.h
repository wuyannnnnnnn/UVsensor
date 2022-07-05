#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include "LedManagerTask.h"
#include "LifeStyle.h"

#define DOUBLE_CHECK_TIMES 1

#define CHARGER_ON_CHECK_INTERVAL 1000
#define CHARGER_OFF_CHECK_INTERVAL 1000
#define NUMBER_OF_BATTERY_SAMPLE  50

class BandModeManager;
class BleManagerTask;

class BatteryMonitor
{
friend BandModeManager;
friend BleManagerTask;

private:
  LedManagerTask *led;
  TimerHandle_t *sleepAlarm;
  LifeStyle *lifeStyle;
  BandModeManager *bandMode;
  BleManagerTask *bleManager;
  DataStore *dataStore;

  int16_t voltageCheckCounter;
  bool pullDown, pullUp;
  bool isLifeStyleSuspend;
  bool isChargerOn;
  bool isStartup;
  bool isFirstMeasure = true;
  TimerHandle_t *battTimer;
  void PinSTAT_pinMode( uint32_t ulPin, uint32_t ulMode );
public:
  BatteryMonitor(LedManagerTask *led, TimerHandle_t *sleepAlarm, LifeStyle *lifeStyle, TimerHandle_t *battTimer, DataStore *dataStore);
  void checkState();

  float ReadBatteryVoltage(bool isCalibrate = false);
  bool getChargerStatus();
  void stopTask();
};

#endif //  BATTERY_MONITOR_H

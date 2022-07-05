#ifndef WATCHDOG_RESET_H
#define WATCHDOG_RESET_H

#include <Arduino.h>

class WatchDogResetTask
{
private:
  SoftwareTimer watchDogTimer;
public:
  WatchDogResetTask(void);

  //! Initialize WatchDog Timer
  void init_watchdog(void);
  //! Reload WatchDog Timer
  static void reload_watchdog(TimerHandle_t xTimerID);
};

#endif //  WATCHDOG_RESET_H

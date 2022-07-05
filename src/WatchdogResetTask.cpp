#ifndef ENABLE_DEBUG_WATCHDOG_RESET
#undef ENABLE_DEBUG
#endif 

#include "WatchdogResetTask.h"

#include "config.h"
#include "debug.h"
#include "nrf52.h"

// Counter is in 32kHz ticks. (NRF_WDT->CRV + 1)/32768 seconds
#define WATCHDOG_COUNTER        32768
#define WATCHDOG_TIMEOUT        10     //! In seconds

WatchDogResetTask::WatchDogResetTask(void) {}

uint32_t times = 0;

//! Initialize WatchDog Timer
void WatchDogResetTask::init_watchdog(void)
{
    DBUGLN("Init Watchdog Timer");
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
    NRF_WDT->CRV = WATCHDOG_COUNTER * WATCHDOG_TIMEOUT-1;    
    NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;
    NRF_WDT->TASKS_START = 1;

    watchDogTimer.begin(3000, reload_watchdog); // Auto reload set by RTOS.h
    watchDogTimer.start();
}

//! Reload WatchDog Timer
void WatchDogResetTask::reload_watchdog(TimerHandle_t xTimerID)
{
  // DBUGLN(times++);
  (void) xTimerID;
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
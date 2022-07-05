#ifndef EXT_RTC_TASK_TASK_H
#define EXT_RTC_TASK_TASK_H

#include "M41T62.h"

#undef min
#undef max
#include <functional>

typedef std::function<void(void)> ext_RTC_alarm_callback_t;

class ExtRTCTask
{
  private:

    RTC_M41T62 rtc;
    DateTime now;
    ext_RTC_alarm_callback_t _extRTC_alarm_CB;

    SemaphoreHandle_t *twi_comms_lock;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t _xHigherPriorityTaskWoken = pdFALSE;
  public:
    ExtRTCTask(SemaphoreHandle_t *twi_comms_lock);

    void printTime(DateTime myTime);

    void setAlarm(DateTime alarmTime, bool isInISR = false);
    DateTime getCurrentTime(bool isInISR = false);
    void setCurrentTime(uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
    void setCurrentTime(uint32_t secondsTime);
    void setExtRTCAlarmCallback(ext_RTC_alarm_callback_t callback);
    void disableAlarm(bool isInISR = false);
    
    void setup();
    
  };

#endif //  EXT_RTC_TASK_TASK_H

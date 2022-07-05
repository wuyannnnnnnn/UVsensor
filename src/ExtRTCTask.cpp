#ifndef ENABLE_DEBUG_EXTRTC
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>

#include "config.h"
#include "debug.h"
#include "ExtRTCTask.h"
#include <Wire.h>

#define FRIE_SECONDS 1

void ExtRTCTask::printTime(DateTime myTime){
  DBUG(myTime.year(), DEC);
  DBUG('/');
  DBUG(myTime.month(), DEC);
  DBUG('/');
  DBUG(myTime.day(), DEC);
  DBUG(' ');
  DBUG(myTime.hour(), DEC);
  DBUG(':');
  DBUG(myTime.minute(), DEC);
  DBUG(':');
  DBUG(myTime.second(), DEC);
  DBUGLN();
}

void ExtRTCTask::setExtRTCAlarmCallback(ext_RTC_alarm_callback_t callback) {
  _extRTC_alarm_CB = callback;
}

DateTime ExtRTCTask::getCurrentTime(bool isInISR) {
  DateTime temp;
  if (isInISR) {
    _xHigherPriorityTaskWoken = pdFALSE;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(*twi_comms_lock, &_xHigherPriorityTaskWoken);
    xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
    temp = rtc.now();
    _xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(*twi_comms_lock, &_xHigherPriorityTaskWoken);
    xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  else
  {    
    xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
    temp = rtc.now();
    xSemaphoreGive(*twi_comms_lock);
  }
  return temp;
}

void ExtRTCTask::setCurrentTime(uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour, uint8_t min, uint8_t sec) {
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  rtc.adjust(DateTime(year, month, day, hour, min, sec));
  xSemaphoreGive(*twi_comms_lock);
}

void ExtRTCTask::setCurrentTime(uint32_t secondsTime) {
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  rtc.adjust(DateTime(secondsTime));
  xSemaphoreGive(*twi_comms_lock);
}

void ExtRTCTask::disableAlarm(bool isInISR) {
  // // Detach Interrupt
  // extRTCInterrupt.Deregister(&extRTCEventListener);
  // extRTCInterrupt.Dettach();
  if (isInISR) {
    _xHigherPriorityTaskWoken = pdFALSE;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(*twi_comms_lock, &_xHigherPriorityTaskWoken);
    xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
    rtc.alarmEnable(0);
    _xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(*twi_comms_lock, &_xHigherPriorityTaskWoken);
    xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  else
  {    
    xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
    rtc.alarmEnable(0);
    xSemaphoreGive(*twi_comms_lock);
  }
  DBUGLN("[RTC Alarm Disabled]");
}

void ExtRTCTask::setAlarm(DateTime alarmTime, bool isInISR) {
  if (isInISR) {
    _xHigherPriorityTaskWoken = pdFALSE;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(*twi_comms_lock, &_xHigherPriorityTaskWoken);
    xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
    rtc.alarmRepeat(4); // set alarm repeat mode (once per day) 
    now = rtc.now(); // get current rtc time
    #ifdef ENABLE_DEBUG_EXTRTC
    DBUGLN("Current Time: ");
    printTime(now);
    #endif
    rtc.alarmSet(alarmTime);
    rtc.alarmEnable(1); // enable the alarm
    _xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(*twi_comms_lock, &_xHigherPriorityTaskWoken);
    xHigherPriorityTaskWoken |= _xHigherPriorityTaskWoken;
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
  else
  {    
    xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
    rtc.alarmRepeat(4); // set alarm repeat mode (once per day) 
    now = rtc.now(); // get current rtc time
    #ifdef ENABLE_DEBUG_EXTRTC
    DBUGLN("Current Time: ");
    printTime(now);
    #endif
    rtc.alarmSet(alarmTime);
    rtc.alarmEnable(1); // enable the alarm
    xSemaphoreGive(*twi_comms_lock);
  }
  // // Attach Interrupt
  // extRTCInterrupt.Register(&extRTCEventListener);
  // extRTCInterrupt.Attach();

  #ifdef ENABLE_DEBUG_EXTRTC
  DBUGLN("Alarm Set to: ");
  printTime(alarmTime);
  DBUG("readSqwPinMode: ");
  DBUGLN(rtc.readSqwPinMode());
  #endif
}

ExtRTCTask::ExtRTCTask(SemaphoreHandle_t *twi_comms_lock) :
  _extRTC_alarm_CB(NULL),
  twi_comms_lock(twi_comms_lock)
{
}

void ExtRTCTask::setup()
{
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  rtc.begin();
  rtc.checkFlags(); // clear any rtc interrupt flags
  // Hack: if not adjust before setting alarm, there won't be any interrupt
  #ifdef ENABLE_TEST_LIFESTYLE
  rtc.adjust(DateTime(2056,7,1,1,1,1));
  #else
  rtc.adjust(rtc.now());
  #endif
  rtc.alarmEnable(0); // Disable alarm. It will be reset depending on situation
  rtc.writeSqwPinMode(SqwNONE);
  xSemaphoreGive(*twi_comms_lock);
  // Setup rtc interrupt input pin
  // Attach Interrupt


  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // rtc.alarmRepeat(6); // set alarm repeat mode (once per year)
  // Serial.print("Repeat Mode Set: ");
  // Serial.println(rtc.alarmRepeat()); // print the alarm mode we just set

  // now = rtc.now(); // get current rtc time
  // // set alarm for 90 seconds in the future
  // DateTime alarmTime (now.unixtime()+FRIE_SECONDS);
  // rtc.alarmSet(alarmTime); 
  // Serial.print("Alarm Set to: ");
  // printTime(alarmTime);
  // rtc.alarmEnable(1); // enable the alarm
  // Serial.print("readSqwPinMode: ");
  // Serial.println(rtc.readSqwPinMode());
  DBUGLN("Finish setting up external RTC alarm");
}
#ifndef ENABLE_DEBUG_UVsensor
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>

#include "config.h"
#include "debug.h"
#include "UVsensor.h"
#include <Wire.h>


void UVTask::printALS(uint32_t myALS){
  DBUG('ALS is ');
  DBUG(myALS, DEC);
  DBUGLN();
}

void UVTask::printUVI(uint32_t myUVI){
  DBUG('UVI is ');
  DBUG(myUVI, DEC);
  DBUGLN();
}

// void UVTask::vTaskALS( void *pvParameters )
// {
// const char *pcTaskName = "ALS data is \r\n";
// volatile uint32_t ul; /* volatile to ensure ul is not optimized away. */
//  /* As per most tasks, this task is implemented in an infinite loop. */
//  for( ;; )
//  {
//     ul = sensor.readALS();

//     /* Print out the name of this task. */
//     Serial.printf( "%s", pcTaskName );
//     Serial.println(ul);

//     /* Delay for a minute. */
//     delay(60);
//  }
// }

// void UVTask::vTaskUVI( void *pvParameters )
// {
// const char *pcTaskName = "UVI data is \r\n";
// volatile uint32_t ul; /* volatile to ensure ul is not optimized away. */
//  /* As per most tasks, this task is implemented in an infinite loop. */
//  for( ;; )
//  {
//     ul = sensor.readUVI();

//     /* Print out the name of this task. */
//     Serial.printf( "%s", pcTaskName );
//     Serial.println(ul);

//     /* Delay for a minute. */
//     delay(60);
//  }
// }

void UVTask::loopALS()
{
if (false == sensor.readALS()) {
  Serial.println("ALS Read Failed");
  sensor.begin();
}
else {
         Serial.println(sensor.readALS());
    /* Delay for a minute. */
    //delay(60);
  // }
}
}

void UVTask::loopUVI()
{
// if (false == sensor.readUVI()) sensor.begin();

if (false == sensor.readUVI()) {
  Serial.println("UVI Read Failed");
  sensor.begin();
}
else {

//  for( ;; )
//  {
  Serial.println(sensor.readUVI());
    /* Delay for a minute. */
    //delay(60);
}
}

void UVTask::setup()
{
  sensor.begin();
  sensor.writeEnable(1);
  // delay(100);
  // Serial.println(sensor.readEnable(),HEX);
  sensor.readPower();
  sensor.readInterrupt();  // clear any rtc interrupt flags
  sensor.readDataStatus(); 
  
  #ifdef ENABLE_DEBUG_UVsensor_ALS
  sensor.writeMode(LTR390UV_MODE_ALS);
  //sensor.configInterrupt(1, LTR390UV_MODE_ALS, 0);
  #endif

  #ifdef ENABLE_DEBUG_UVsensor_UVI
  sensor.writeMode(LTR390UV_MODE_UVI);
  //sensor.configInterrupt(1, LTR390UV_MODE_UVI, 0);
  #endif
  
  if (sensor.readMode()==LTR390UV_MODE_ALS){
    Serial.println("Mode Set to: ALS");
  }
  else if (sensor.readMode()==LTR390UV_MODE_UVI){
    Serial.println("Mode Set to: UVI");
  }
  else {Serial.println("Mode Set Failed");}

  //xSemaphoreGive(*twi_comms_lock);
  // Setup rtc interrupt input pin
  // Attach Interrupt

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
  DBUGLN("Finish setting up UV sensor");
}
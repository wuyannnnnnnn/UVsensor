#ifndef ENABLE_DEBUG_UVsensor
#undef ENABLE_DEBUG
#endif

#include <Arduino.h>

#include "config.h"
#include "debug.h"
#include "UVsensor.h"
#include <Wire.h>

UVTask::UVTask(SemaphoreHandle_t *twi_comms_lock) :
 twi_comms_lock(twi_comms_lock){}


// bool reset;
TaskHandle_t UVTask::xThisTask = NULL;

void UVTask::vISR_UV () {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  configASSERT( UVTask::xThisTask != NULL );
  vTaskNotifyGiveFromISR( UVTask::xThisTask , &xHigherPriorityTaskWoken) ;
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void UVTask::init() {
  if (NULL == xThisTask) {UVTask::xThisTask = xTaskGetCurrentTaskHandle();}
  
  // initialize device
  DBUGLN(F("Initializing UV sensor..."));
  xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
  sensor.begin();
  sensor.readPower();
  sensor.readInterrupt();  // clear any rtc interrupt flags
  sensor.readDataStatus(); 
  xSemaphoreGive(*twi_comms_lock);
  isInitialised = true;
}


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


// void UVTask::loopALS()
// {
// if (false == sensor.readALS()) {
//   Serial.println("ALS Read Failed");
//   sensor.begin();
// }
// else {
//          Serial.println(sensor.readALS());
//     /* Delay for a minute. */
//     //delay(60);
//   // }
// }
// }

// UVTask::data=sensor.readALS();
// void TaskA( void *pvParameters )  
// {  
//     int16_t SendNum = sensor.readALS(); 
//     for( ;; )  
//     {  
//         vTaskDelay( 2000/portTICK_RATE_MS );  
//         /* 向队列中填充内容 */  
//         xQueueSend( xQueue, ( void* )&SendNum, 0 );  
//         SendNum++;  
//     }  
// }  
// void TaskB( void *pvParameters )  
// {  
//     int16_t ReceiveNum = 0;  
//     for( ;; )  
//     {  
//         /* 从队列中获取内容 */  
//         if( xQueueReceive( xQueue, &ReceiveNum, 100/portTICK_RATE_MS ) == pdPASS)  
//         {  
//             Serial.println(ReceiveNum);  
//         }  
//     }  
// }  

// void UVTask::loopALS(void)
// {  
//     /* 建立任务 */  
//     xTaskCreate( TaskA, "TaskA", 1000,  
//                            ( void * )  100, 1, NULL );  
//     xTaskCreate( TaskB,  "TaskB", 1000,  
//                             ( void * ) 1000, 2, NULL );  
//     /* 启动OS */  
//     vTaskStartScheduler();  
// }


void UVTask::loopALS()
 {
  if (false == isInitialised) init();

  //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  for(;;)
  {
    //xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
    // get read mode
    DataStatus = sensor.readDataStatus();
    //xSemaphoreGive(*twi_comms_lock);
    if (true == DataStatus)
    {
      xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
      Serial.print("Current ambiant light is: ");
      Serial.println(sensor.readALS());;
      xSemaphoreGive(*twi_comms_lock);
      //delay(1000);
    }
    delay(1000);
  }
  
 }

void UVTask::loopUVI()
 {
  if (false == isInitialised) init();

  //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
   for(;;)
  {
    //xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
    // get read mode
    DataStatus = sensor.readDataStatus();
    //xSemaphoreGive(*twi_comms_lock);
    if (true == DataStatus)
    {
      xSemaphoreTake(*twi_comms_lock, portMAX_DELAY);
      Serial.print("Current UV light is: ");
      Serial.println(sensor.readUVI());;
      xSemaphoreGive(*twi_comms_lock);
      //delay(1000);
    }
    delay(1000);
  }
 }
// void UVTask::loopUVI()
// {
// // if (false == sensor.readUVI()) sensor.begin();

// // if (false == sensor.readUVI()) {
// //   Serial.println("UVI Read Failed");
// //   sensor.begin();
// // }
// // else {

//   Serial.println(sensor.readUVI());
// // }
// }

void UVTask::setup()
{
  sensor.writeEnable(1);
  // delay(100);
  // Serial.println(sensor.readEnable(),HEX);
  
  #ifdef ENABLE_DEBUG_UVsensor_ALS
  sensor.writeMode(LTR390UV_MODE_ALS);
  //sensor.configInterrupt(1, LTR390UV_MODE_ALS, 0);
  #endif

  #ifdef ENABLE_DEBUG_UVsensor_UVI
  sensor.writeMode(LTR390UV_MODE_UVI);
  sensor.configInterrupt(1, LTR390UV_MODE_UVI, 0);
  #endif

  //Serial.println(sensor.readMode());
  
  if (sensor.readMode()==LTR390UV_MODE_ALS){
    Serial.println("Mode Set to: ALS");
  }
  else if (sensor.readMode()==LTR390UV_MODE_UVI){
    Serial.println("Mode Set to: UVI");
  }
  else {Serial.println("Mode Set Failed");}
}
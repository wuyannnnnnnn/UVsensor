#ifndef _UVsensor_H
#define _UVsensor_H

#include "LTR390UV.h"

//#include <functional>

//typedef std::function<void(void)> ext_RTC_alarm_callback_t;


class UVTask
{
  private:
    LTR390UV sensor;
    SemaphoreHandle_t *twi_comms_lock;
    //static double_Tap_Detected_callback_t  _doubleTap_Detected_CB;
    static TaskHandle_t xThisTask; 

    bool isInitialised = false;
    int DataStatus;   // holds data status (old or new)
    //QueueHandle_t xQueue=xQueueCreate( 10, sizeof( uint32_t ) );
  public:
    UVTask(SemaphoreHandle_t *twi_comms_lock);


// void TaskA(void *pvParameters);
// void TaskB(void *pvParameters);
    void init();
    void setup(); 

    void printALS(uint32_t myALS);
    void printUVI(uint32_t myUVI);

    void loopALS();
    void loopUVI();
    static void vISR_UV();
  };
// void TaskA(void *pvParameters);
// void TaskB(void *pvParameters);

// QueueHandle_t xQueue=xQueueCreate( 10, sizeof( uint32_t ) );
#endif //  _UVsensor_H